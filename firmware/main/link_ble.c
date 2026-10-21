#include "link_ble.h"
#include "config.h"

#if DATA_LINK == DATA_LINK_BLE

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "host/ble_hs_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"
#include "os/os_mbuf.h"

void ble_store_config_init(void);

static const char *TAG = "link";

// Same service the laptop already scans for. Char 01 is device → host,
// char 02 is host → device. Both are a raw byte stream of the USB packets.
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x00, 0x59, 0x4f, 0x59);
static const ble_uuid128_t tx_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x01, 0x59, 0x4f, 0x59);
static const ble_uuid128_t rx_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x02, 0x59, 0x4f, 0x59);

#define BLE_NAME "Voyage"
#define TX_CAP   8192
#define RX_CAP   4096
#define LINK_BARRIER() __asm__ __volatile__("" ::: "memory")

static uint16_t tx_val_handle;
static uint16_t conn = BLE_HS_CONN_HANDLE_NONE;
static volatile bool tx_notify;
static uint16_t chunk = 20;
static uint8_t own_addr_type;

static uint8_t tx_ring[TX_CAP];
static volatile uint32_t tx_w, tx_r;
static uint8_t rx_ring[RX_CAP];
static volatile uint32_t rx_w, rx_r;
static TaskHandle_t tx_task;

static int gap_event(struct ble_gap_event *event, void *arg);

static void advertise(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)BLE_NAME;
    fields.name_len = sizeof BLE_NAME - 1;
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv = {0};
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv.itvl_min = 160;  // 100 ms
    adv.itvl_max = 240;  // 150 ms
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv, gap_event, NULL);
}

static int gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    uint16_t n = OS_MBUF_PKTLEN(ctxt->om);
    if (n == 0) return 0;
    if ((rx_w - rx_r) + n > RX_CAP) return 0;
    uint8_t tmp[244];
    uint16_t off = 0;
    while (off < n) {
        uint16_t take = n - off;
        if (take > sizeof tmp) take = sizeof tmp;
        if (os_mbuf_copydata(ctxt->om, off, take, tmp) != 0) break;
        uint32_t w = rx_w;
        for (uint16_t i = 0; i < take; i++) rx_ring[(w + i) & (RX_CAP - 1)] = tmp[i];
        LINK_BARRIER();
        rx_w = w + take;
        off += take;
    }
    return 0;
}

static const struct ble_gatt_svc_def svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &tx_uuid.u,
                .access_cb = gatt_access,
                .val_handle = &tx_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = &rx_uuid.u,
                .access_cb = gatt_access,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {0},
        },
    },
    {0},
};

static void tx_pump(void *arg)
{
    (void)arg;
    uint8_t buf[244];
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        while (tx_notify && conn != BLE_HS_CONN_HANDLE_NONE) {
            uint32_t w = tx_w;
            LINK_BARRIER();
            uint32_t have = w - tx_r;
            if (have == 0) break;
            uint16_t n = chunk;
            if (n > sizeof buf) n = sizeof buf;
            if (have < n) n = (uint16_t)have;
            uint32_t r = tx_r;
            for (uint16_t i = 0; i < n; i++) buf[i] = tx_ring[(r + i) & (TX_CAP - 1)];
            struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, n);
            if (!om) {
                vTaskDelay(pdMS_TO_TICKS(2));
                break;
            }
            int rc = ble_gatts_notify_custom(conn, tx_val_handle, om);
            if (rc == BLE_HS_ENOMEM) {  // mbuf already consumed
                vTaskDelay(pdMS_TO_TICKS(2));
                break;
            }
            if (rc != 0) break;
            tx_r = r + n;
        }
    }
}

static void on_reset(int reason)
{
    (void)reason;
    tx_notify = false;
    conn = BLE_HS_CONN_HANDLE_NONE;
}

static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &own_addr_type);
    advertise();
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            advertise();
            break;
        }
        conn = event->connect.conn_handle;
        chunk = 20;
        {
            struct ble_gap_upd_params p = {
                .itvl_min = 8,
                .itvl_max = 12,
                .latency = 0,
                .supervision_timeout = 200,
            };
            ble_gap_update_params(conn, &p);
            ble_gap_set_data_len(conn, 251, 2120);
            ble_gap_set_prefered_le_phy(conn, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0);
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        tx_notify = false;
        conn = BLE_HS_CONN_HANDLE_NONE;
        tx_r = tx_w;
        rx_r = rx_w;
        advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == tx_val_handle) {
            tx_notify = event->subscribe.cur_notify;
            if (tx_notify && tx_task) xTaskNotifyGive(tx_task);
        }
        break;
    case BLE_GAP_EVENT_MTU:
        if (event->mtu.value > 23) {
            uint16_t n = event->mtu.value - 3;
            if (n > 244) n = 244;
            chunk = n;
        }
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        break;
    default:
        break;
    }
    return 0;
}

static void host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t link_ble_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(nimble_port_init());
    ble_svc_gap_init();
    ble_svc_gap_device_name_set(BLE_NAME);
    ble_svc_gatt_init();
    ble_gatts_count_cfg(svcs);
    ble_gatts_add_svcs(svcs);
    ble_att_set_preferred_mtu(247);
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();
    xTaskCreate(tx_pump, "bletx", 3072, NULL, 7, &tx_task);
    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "ready");
    return ESP_OK;
}

void link_ble_send(const void *buf, size_t n)
{
    if (!tx_notify || n == 0 || n > TX_CAP) return;
    if ((tx_w - tx_r) + n > TX_CAP) return;
    const uint8_t *p = buf;
    uint32_t w = tx_w;
    for (size_t i = 0; i < n; i++) tx_ring[(w + i) & (TX_CAP - 1)] = p[i];
    LINK_BARRIER();
    tx_w = w + n;
    if (tx_task) xTaskNotifyGive(tx_task);
}

int link_ble_read(uint8_t *dst, size_t max, int timeout_ms)
{
    TickType_t t0 = xTaskGetTickCount();
    for (;;) {
        uint32_t w = rx_w;
        LINK_BARRIER();
        uint32_t have = w - rx_r;
        if (have) {
            if (have > max) have = (uint32_t)max;
            uint32_t r = rx_r;
            for (uint32_t i = 0; i < have; i++) dst[i] = rx_ring[(r + i) & (RX_CAP - 1)];
            rx_r = r + have;
            return (int)have;
        }
        if ((int)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS) >= timeout_ms) return 0;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

#else

esp_err_t link_ble_init(void)
{
    return ESP_OK;
}

void link_ble_send(const void *buf, size_t n)
{
    (void)buf;
    (void)n;
}

int link_ble_read(uint8_t *dst, size_t max, int timeout_ms)
{
    (void)dst;
    (void)max;
    (void)timeout_ms;
    return 0;
}

#endif
