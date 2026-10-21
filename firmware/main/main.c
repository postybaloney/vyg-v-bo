#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_crc.h"
#include "esp_rom_sys.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "config.h"
#include "fpga.h"
#include "impedance.h"
#include "audio.h"
#include "link_ble.h"

static const char *TAG = "eeg";

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t frame_bytes;
    uint8_t  chip;             // CHIP_EMG or CHIP_EEG, matches the packet type
    uint64_t mcu_time_us;
    uint64_t sample_index;     // counts this chip only
    int32_t  channels[FRAME_CHANNELS];
    uint32_t crc;
} frame_t;

_Static_assert(sizeof(frame_t) == 61, "frame_t layout");

#define LOG_LINE_MAX 192

static ads_chip_t ads[2];
static TaskHandle_t acq_task_h;
static QueueHandle_t frame_q;
static int fast_i, slow_i;

static volatile uint8_t z_mode_req;
static TaskHandle_t z_tx_h;
static z_packet_t z_mail;
static volatile uint32_t z_seq;
static void run_impedance(void);

static SemaphoreHandle_t usb_lock;
static TaskHandle_t log_task;          // task currently inside a USB write
static char log_line[LOG_LINE_MAX];
static size_t log_len;

static void IRAM_ATTR drdy_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(acq_task_h, &hp);
    portYIELD_FROM_ISR(hp);
}

static bool drdy_low(void)
{
    return gpio_get_level(PIN_DRDY) == 0;
}

static esp_err_t configure_chip(int i)
{
    ads_chip_t *a = &ads[i];
    const ads_chip_cfg_t *c = &kChipCfg[i];
    const uint8_t gain = ads_gain_code(c->gain);
    const uint8_t mux = c->input == ADS_INPUT_TEST    ? ADS_MUX_TEST
                      : c->input == ADS_INPUT_SHORTED ? ADS_MUX_SHORT
                                                      : ADS_MUX_NORMAL;

    const uint8_t cfg123[3] = {
        ADS_CONFIG1_BASE | ADS_CONFIG1_MULTI_READBACK | c->rate |
            ((ADS_SHARED_CLOCK && i == 0) ? ADS_CONFIG1_CLK_EN : 0),
        c->input == ADS_INPUT_TEST ? ADS_CONFIG2_TEST : ADS_CONFIG2_NORMAL,
        c->bias ? ADS_CONFIG3_REF_BIAS : ADS_CONFIG3_REF,
    };

    uint8_t chs[10];  // CH1SET..CH8SET, BIAS_SENSP, BIAS_SENSN (contiguous 0x05..0x0E)
    const uint8_t bias_mask = i == 0 ? EMG_BIAS_MASK : EEG_BIAS_MASK;
    for (int ch = 0; ch < 8; ch++)
        chs[ch] = ((c->channel_mask >> ch) & 1) ? (uint8_t)((gain << 4) | mux) : ADS_CHSET_OFF;
    chs[8] = c->bias ? bias_mask : 0;
    // Monopolar: all INxN sit on SRB1 (the reference), so don't feed them into bias.
    chs[9] = (c->bias && c->mode == ADS_BIPOLAR) ? c->channel_mask : 0;

    const uint8_t misc1 = (c->mode == ADS_MONOPOLAR) ? ADS_MISC1_SRB1 : 0x00;

    ESP_RETURN_ON_ERROR(ads_cmd(a, ADS_CMD_SDATAC), TAG, "sdatac");
    ESP_RETURN_ON_ERROR(ads_wreg(a, ADS_REG_CONFIG1, cfg123, 3), TAG, "cfg");
    vTaskDelay(pdMS_TO_TICKS(150));  // internal reference settle
    ESP_RETURN_ON_ERROR(ads_wreg(a, ADS_REG_CH1SET, chs, sizeof chs), TAG, "chset");
    ESP_RETURN_ON_ERROR(ads_wreg(a, ADS_REG_MISC1, &misc1, 1), TAG, "misc1");

    uint8_t rb[15], rb_misc1;
    ESP_RETURN_ON_ERROR(ads_rreg(a, ADS_REG_ID, rb, sizeof rb), TAG, "rreg");
    ESP_RETURN_ON_ERROR(ads_rreg(a, ADS_REG_MISC1, &rb_misc1, 1), TAG, "rreg");
    const bool ok = rb[0] == ADS_ID_ADS1299_8 &&
                    memcmp(&rb[1], cfg123, 3) == 0 &&
                    memcmp(&rb[5], chs, sizeof chs) == 0 &&
                    rb_misc1 == misc1;
    ESP_LOGI(TAG, "%s id=%02X cfg=%02X %02X %02X misc1=%02X %s",
             a->name, rb[0], rb[1], rb[2], rb[3], rb_misc1, ok ? "ok" : "MISMATCH");
    return ok ? ESP_OK : ESP_FAIL;
}

// ID plus a different CH1SET value on each chip. Writing both before reading either
// back catches one CS line that hits both devices: the second write would overwrite
// the first chip, and that chip's readback would show the other sentinel.
static esp_err_t link_test(void)
{
    const uint8_t want[2] = { 0xA5, 0x5A };
    uint8_t id[2] = { 0xFF, 0xFF };
    esp_err_t bus[2] = { ESP_OK, ESP_OK };
    bool pass = true;

    for (int i = 0; i < 2; i++) {
        if (!(i == 0 ? EMG_ADC_ENABLED : EEG_ADC_ENABLED)) continue;
        bus[i] = ads_cmd(&ads[i], ADS_CMD_SDATAC);
        if (bus[i] == ESP_OK) bus[i] = ads_rreg(&ads[i], ADS_REG_ID, &id[i], 1);
        if (bus[i] == ESP_OK) bus[i] = ads_wreg(&ads[i], ADS_REG_CH1SET, &want[i], 1);
    }
    for (int i = 0; i < 2; i++) {
        if (!(i == 0 ? EMG_ADC_ENABLED : EEG_ADC_ENABLED)) continue;
        uint8_t rb = 0xFF;
        if (bus[i] == ESP_OK) bus[i] = ads_rreg(&ads[i], ADS_REG_CH1SET, &rb, 1);
        const bool id_ok = id[i] == ADS_ID_ADS1299_8;
        const bool rb_ok = rb == want[i];
        if (bus[i] != ESP_OK || !id_ok || !rb_ok) {
            ESP_LOGE(TAG, "%s FAIL: id=%02X want %02X, ch1set=%02X want %02X%s%s",
                     ads[i].name, id[i], ADS_ID_ADS1299_8, rb, want[i],
                     bus[i] != ESP_OK ? " " : "",
                     bus[i] != ESP_OK ? esp_err_to_name(bus[i]) : "");
            pass = false;
        } else {
            ESP_LOGI(TAG, "%s PASS: id=%02X ch1set=%02X", ads[i].name, id[i], rb);
        }
    }
    if (pass) ESP_LOGI(TAG, "LINK PASS");
    else ESP_LOGE(TAG, "LINK FAIL");
    return pass ? ESP_OK : ESP_FAIL;
}

static esp_err_t ads_bringup(void)
{
    // PWDN is tied off-board. There is no RESET pin; the SPI RESET command is the reset.
    esp_err_t err = fpga_set_start(false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FPGA FAIL: START slave 0x%02X %s", FPGA_ADDR_START, esp_err_to_name(err));
        return err;
    }
    err = fpga_cs(FPGA_CS_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FPGA FAIL: CS slave 0x%02X %s", FPGA_ADDR_CS, esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(150));        // tPOR

    // EMG has its own clock (CLKSEL=1). EEG does not, until EMG's CLK pin is running.
#if EMG_ADC_ENABLED
    ESP_RETURN_ON_ERROR(ads_cmd(&ads[0], ADS_CMD_SDATAC), TAG, "EMG sdatac");
    ESP_RETURN_ON_ERROR(ads_cmd(&ads[0], ADS_CMD_RESET), TAG, "EMG reset");
    vTaskDelay(pdMS_TO_TICKS(1));
#endif

#if ADS_SHARED_CLOCK && EEG_ADC_ENABLED
    const uint8_t c1 = ADS_CONFIG1_BASE | ADS_CONFIG1_MULTI_READBACK | ADS_CONFIG1_CLK_EN | kChipCfg[0].rate;
    ESP_RETURN_ON_ERROR(ads_cmd(&ads[0], ADS_CMD_SDATAC), TAG, "EMG sdatac");
    ESP_RETURN_ON_ERROR(ads_wreg(&ads[0], ADS_REG_CONFIG1, &c1, 1), TAG, "EMG clk_en");
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_RETURN_ON_ERROR(ads_cmd(&ads[1], ADS_CMD_RESET), TAG, "EEG reset");
    vTaskDelay(pdMS_TO_TICKS(1));
#elif EEG_ADC_ENABLED
    ESP_RETURN_ON_ERROR(ads_cmd(&ads[1], ADS_CMD_SDATAC), TAG, "EEG sdatac");
    ESP_RETURN_ON_ERROR(ads_cmd(&ads[1], ADS_CMD_RESET), TAG, "EEG reset");
    vTaskDelay(pdMS_TO_TICKS(1));
#endif

    if (link_test() != ESP_OK) return ESP_FAIL;

    for (int i = 0; i < 2; i++) {
        if (!(i == 0 ? EMG_ADC_ENABLED : EEG_ADC_ENABLED)) continue;
        int tries = 0;
        while (configure_chip(i) != ESP_OK) {
            if (++tries >= 5) {
                ESP_LOGE(TAG, "%s config failed", ads[i].name);
                return ESP_FAIL;
            }
        }
    }

#if EMG_ADC_ENABLED
    ESP_RETURN_ON_ERROR(ads_cmd(&ads[0], ADS_CMD_RDATAC), TAG, "EMG rdatac");
#endif
#if EEG_ADC_ENABLED
    ESP_RETURN_ON_ERROR(ads_cmd(&ads[1], ADS_CMD_RDATAC), TAG, "EEG rdatac");
#endif
    ESP_RETURN_ON_ERROR(fpga_set_start(true), TAG, "start");  // both chips, one edge
    return ESP_OK;
}

// Read whichever chip is faster, then the other one only if its DRDY is still
// holding the shared line down. Same rate: both are due on every edge.
// ADS1299 has no data-ready bit in its registers, and RREG would have to leave
// RDATAC, so the pin level is the probe.
static bool read_ready(uint8_t raw[2][ADS_FRAME_BYTES], bool got[2])
{
    const bool same_rate = kChipCfg[0].rate == kChipCfg[1].rate;
    got[0] = got[1] = false;

    if (ads_read_frame(&ads[fast_i], raw[fast_i]) != ESP_OK) return false;
    got[fast_i] = true;
    if (slow_i < 0) return true;

    bool slow_due = same_rate;
    if (!slow_due) {
        esp_rom_delay_us(5);  // FPGA OR delay after the fast chip's DRDY rises
        slow_due = drdy_low();
    }
    if (slow_due && ads_read_frame(&ads[slow_i], raw[slow_i]) == ESP_OK)
        got[slow_i] = true;
    return true;
}

static uint32_t dropped;

static void enqueue_chip(int chip, const uint8_t raw[ADS_FRAME_BYTES], uint64_t *idx, uint32_t *bad_status)
{
    frame_t f = {
        .magic = FRAME_MAGIC,
        .version = FRAME_VERSION,
        .frame_bytes = sizeof f,
        .chip = (uint8_t)chip,
        .mcu_time_us = (uint64_t)esp_timer_get_time(),
        .sample_index = (*idx)++,
    };
    if ((raw[0] & 0xF0) != 0xC0) (*bad_status)++;
    for (int ch = 0; ch < 8; ch++)
        f.channels[ch] = ads_s24(&raw[3 + ch * 3]);
    f.crc = esp_crc32_le(0, (const uint8_t *)&f, sizeof f - sizeof f.crc);
    if (xQueueSend(frame_q, &f, 0) != pdTRUE) dropped++;
}

static void acq_task(void *arg)
{
    uint8_t raw[2][ADS_FRAME_BYTES] = {0};
    uint64_t idx[2] = {0, 0};
    uint32_t missed = 0, bad_status = 0, slow_n = 0;

    // Smaller ADS_DR_* code is the higher sample rate. Equal -> EMG first.
    // A disabled ADC is not read; its DRDY is not clocked.
#if EMG_ADC_ENABLED && EEG_ADC_ENABLED
    if (kChipCfg[1].rate < kChipCfg[0].rate) {
        fast_i = 1;
        slow_i = 0;
    } else {
        fast_i = 0;
        slow_i = 1;
    }
#elif EMG_ADC_ENABLED
    fast_i = 0;
    slow_i = -1;
#else
    fast_i = 1;
    slow_i = -1;
#endif
    ESP_LOGI(TAG, "DRDY read %s%s%s",
             ads[fast_i].name,
             slow_i < 0 ? "" : ", then ",
             slow_i < 0 ? "" : ads[slow_i].name);

    while (ads_bringup() != ESP_OK) vTaskDelay(pdMS_TO_TICKS(1000));

    for (;;) {
        if (z_mode_req) {
            run_impedance();
            while (ads_bringup() != ESP_OK) vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        const uint32_t n = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        if (!drdy_low()) {
            if (n == 0) {
                ESP_LOGW(TAG, "no DRDY for 100 ms, re-initialising");
                while (ads_bringup() != ESP_OK) vTaskDelay(pdMS_TO_TICKS(1000));
                memset(raw, 0, sizeof raw);
            }
            continue;  // stale notify: this edge was already fully read
        }
        if (n > 1) missed += n - 1;

        // Pin stays low across unread chips and across a sample that arrived
        // mid-read. Cap the drain so a stuck line cannot spin forever.
        for (int guard = 0; guard < 8 && drdy_low(); guard++) {
            bool got[2];
            if (!read_ready(raw, got)) break;
            if (slow_i >= 0 && got[slow_i]) slow_n++;
            for (int k = 0; k < 2; k++)
                if (got[k]) enqueue_chip(k, raw[k], &idx[k], &bad_status);

            if (got[fast_i] && idx[fast_i] % 5000 == 0)
                ESP_LOGI(TAG, "emg=%llu eeg=%llu missed=%lu bad_status=%lu dropped=%lu slow=%lu",
                         idx[CHIP_EMG], idx[CHIP_EEG], missed, bad_status, dropped, slow_n);
        }
        if (drdy_low()) {
            ESP_LOGW(TAG, "DRDY stuck low, re-initialising");
            while (ads_bringup() != ESP_OK) vTaskDelay(pdMS_TO_TICKS(1000));
            memset(raw, 0, sizeof raw);
        }
    }
}

// Caller holds usb_lock. Drops the packet if the host is not reading.
// log_task makes a log printed from inside the USB driver return immediately,
// instead of taking usb_lock again on this same task.
static void usb_send(const void *buf, size_t n)
{
#if DATA_LINK == DATA_LINK_BLE
    link_ble_send(buf, n);
    return;
#endif
    log_task = xTaskGetCurrentTaskHandle();
    usb_serial_jtag_write_bytes(buf, n, pdMS_TO_TICKS(20));
    log_task = NULL;
}

// Caller holds usb_lock. Writes the whole buffer or gives up; a short write
// desyncs the host until the next magic, same as a dropped frame.
static void usb_send_all(const void *buf, size_t n)
{
#if DATA_LINK == DATA_LINK_BLE
    link_ble_send(buf, n);
    return;
#endif
    const uint8_t *p = buf;
    size_t left = n;
    TickType_t t0 = xTaskGetTickCount();
    log_task = xTaskGetCurrentTaskHandle();
    while (left) {
        int w = usb_serial_jtag_write_bytes(p, left, pdMS_TO_TICKS(20));
        if (w < 0) break;
        if (w == 0) {
            if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(100)) break;
            continue;
        }
        p += (size_t)w;
        left -= (size_t)w;
    }
    log_task = NULL;
}

#define Z_BARRIER() __asm__ __volatile__("" ::: "memory")

static void z_publish(z_packet_t *pkt)
{
    pkt->crc = 0;
    pkt->crc = esp_crc32_le(0, (const uint8_t *)pkt, sizeof *pkt - sizeof pkt->crc);
    uint32_t seq = z_seq;
    z_seq = seq + 1;  // odd: copy in progress
    Z_BARRIER();
    z_mail = *pkt;
    Z_BARRIER();
    z_seq = seq + 2;  // even: stable
    if (z_tx_h) xTaskNotifyGive(z_tx_h);
}

static bool z_copy_mail(z_packet_t *out)
{
    for (int attempt = 0; attempt < 4; attempt++) {
        uint32_t s1 = z_seq;
        if (s1 & 1u) continue;
        Z_BARRIER();
        z_packet_t tmp = z_mail;
        Z_BARRIER();
        uint32_t s2 = z_seq;
        if (s1 == s2 && s1 != 0) {
            *out = tmp;
            return true;
        }
    }
    return false;
}

static void z_tx_task(void *arg)
{
    uint8_t buf[5 + sizeof(z_packet_t)];
    uint32_t magic = FRAME_MAGIC;
    memcpy(buf, &magic, 4);
    buf[4] = PKT_TYPE_Z;
    uint32_t sent = 0;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (;;) {
            uint32_t seq = z_seq;
            if ((seq & 1u) || seq == 0 || seq == sent) break;
            z_packet_t pkt;
            if (!z_copy_mail(&pkt)) break;
            memcpy(buf + 5, &pkt, sizeof pkt);
            xSemaphoreTake(usb_lock, portMAX_DELAY);
            usb_send_all(buf, sizeof buf);
            xSemaphoreGive(usb_lock);
            sent = seq & ~1u;
            if (z_seq == sent) break;
        }
    }
}

#if AUDIO_ENABLED
static void audio_tx_task(void *arg)
{
    int16_t samples[AUDIO_FRAME_SAMPLES];
    uint32_t seq = 0;
    uint8_t buf[5 + 4 + AUDIO_FRAME_SAMPLES * 2 + 4];
    uint32_t magic = FRAME_MAGIC;
    memcpy(buf, &magic, 4);
    buf[4] = PKT_TYPE_AUDIO;
    for (;;) {
        if (!audio_take_frame(samples, &seq)) continue;
        memcpy(buf + 5, &seq, 4);
        memcpy(buf + 9, samples, AUDIO_FRAME_SAMPLES * 2);
        uint32_t crc = esp_crc32_le(0, buf + 5, 4 + AUDIO_FRAME_SAMPLES * 2);
        memcpy(buf + 9 + AUDIO_FRAME_SAMPLES * 2, &crc, 4);
        xSemaphoreTake(usb_lock, portMAX_DELAY);
        usb_send_all(buf, sizeof buf);
        xSemaphoreGive(usb_lock);
    }
}
#endif

static void cmd_task(void *arg)
{
    uint8_t buf[64];
    uint8_t payload[4 + 2 + AUDIO_FRAME_SAMPLES * 2];
    int filled = 0;
    int state = 0;
    for (;;) {
#if DATA_LINK == DATA_LINK_BLE
        int n = link_ble_read(buf, sizeof buf, 50);
#else
        int n = usb_serial_jtag_read_bytes(buf, sizeof buf, pdMS_TO_TICKS(50));
#endif
        if (n <= 0) continue;
        for (int i = 0; i < n; i++) {
            uint8_t b = buf[i];
            if (state == 0) {
                state = b == HOST_CMD_MAGIC0 ? 1 : 0;
            } else if (state == 1) {
                state = b == HOST_CMD_MAGIC1 ? 2 : (b == HOST_CMD_MAGIC0 ? 1 : 0);
            } else if (state == 2) {
                if (b == HOST_CMD_Z_ON) z_mode_req = 1;
                else if (b == HOST_CMD_Z_OFF) z_mode_req = 0;
                else if (b == HOST_CMD_AUDIO) {
                    state = 3;
                    filled = 0;
                    continue;
                }
                state = 0;
                if (acq_task_h && (b == HOST_CMD_Z_ON || b == HOST_CMD_Z_OFF))
                    xTaskNotifyGive(acq_task_h);
            } else {
                payload[filled++] = b;
                if (filled < (int)sizeof payload) continue;
                state = 0;
                uint32_t seq;
                uint16_t count;
                memcpy(&seq, payload, 4);
                memcpy(&count, payload + 4, 2);
                if (count == AUDIO_FRAME_SAMPLES)
                    audio_play((const int16_t *)(payload + 6), AUDIO_FRAME_SAMPLES, seq);
            }
        }
    }
}

static void run_impedance(void)
{
    // Stay here until the host sends Z off. A failed configure must not fall
    // back into EEG streaming while the UI is still in impedance mode.
    while (z_mode_req) {
        if (z_configure(ads) != ESP_OK) {
            ESP_LOGE(TAG, "Z FAIL");
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        ESP_LOGI(TAG, "Z ON");
        uint8_t parity = 0;
        bool logged = false;
        int fails = 0;
        while (z_mode_req) {
            z_packet_t pkt;
            int64_t t_start = esp_timer_get_time();
            esp_err_t err = z_capture_cycle(ads, parity, &z_mode_req, &pkt);
            if (err == ESP_ERR_INVALID_STATE) break;
            if (err != ESP_OK || !pkt.ok) {
                if (++fails == 1 || (fails % 100) == 0)
                    ESP_LOGW(TAG, "Z capture %s", esp_err_to_name(err));
                int64_t left_us = 400000 - (esp_timer_get_time() - t_start);
                if (left_us > 0) vTaskDelay(pdMS_TO_TICKS((left_us + 999) / 1000));
                continue;
            }
            fails = 0;
            if (!logged) {
                logged = true;
                ESP_LOGI(TAG, "Z spacing us %u %u %u",
                         pkt.dt_us[0][1], pkt.dt_us[0][2], pkt.dt_us[0][3]);
            }
            z_publish(&pkt);
            parity ^= 1;
            int64_t left_us = 400000 - (esp_timer_get_time() - t_start);
            if (left_us > 0) vTaskDelay(pdMS_TO_TICKS((left_us + 999) / 1000));
        }
        ESP_LOGI(TAG, "Z OFF");
        break;
    }
    fpga_set_start(false);
    fpga_cs(FPGA_CS_NONE);
}

static void send_frame(const frame_t *f)
{
    uint8_t buf[5 + sizeof(frame_t)];
    uint32_t magic = FRAME_MAGIC;
    memcpy(buf, &magic, 4);
    buf[4] = f->chip == CHIP_EEG ? PKT_TYPE_EEG : PKT_TYPE_EMG;
    memcpy(buf + 5, f, sizeof *f);
    xSemaphoreTake(usb_lock, portMAX_DELAY);
    usb_send(buf, sizeof buf);
    xSemaphoreGive(usb_lock);
}

static void send_log(const char *s, uint16_t n)
{
    uint8_t buf[7 + LOG_LINE_MAX];
    uint32_t magic = FRAME_MAGIC;
    memcpy(buf, &magic, 4);
    buf[4] = PKT_TYPE_LOG;
    buf[5] = (uint8_t)n;
    buf[6] = (uint8_t)(n >> 8);
    memcpy(buf + 7, s, n);
    usb_send(buf, 7u + n);
}

// ESP-IDF calls this three times per line (prefix, message, newline).
// Bytes accumulate until '\n', then one type-0x02 packet goes out.
static int log_vprintf(const char *fmt, va_list ap)
{
    if (usb_lock == NULL || xPortInIsrContext()) return 0;
    if (xTaskGetCurrentTaskHandle() == log_task) return 0;

    char chunk[LOG_LINE_MAX];
    int n = vsnprintf(chunk, sizeof chunk, fmt, ap);
    if (n < 0) return n;
    size_t have = (size_t)n < sizeof chunk ? (size_t)n : sizeof chunk - 1;

    xSemaphoreTake(usb_lock, portMAX_DELAY);
    for (size_t i = 0; i < have; i++) {
        log_line[log_len++] = chunk[i];
        if (chunk[i] == '\n' || log_len == sizeof log_line) {
            send_log(log_line, (uint16_t)log_len);
            log_len = 0;
        }
    }
    xSemaphoreGive(usb_lock);
    return n;
}

static void tx_task(void *arg)
{
    frame_t f;
    for (;;) {
        xQueueReceive(frame_q, &f, portMAX_DELAY);
        send_frame(&f);
    }
}

static void park_unused_pins(void)
{
    // VBAT/VIN dividers stay high-Z. The mic (GPIO3) and I2S pins are claimed
    // by the audio driver after this.
    gpio_config_t analog = {
        .pin_bit_mask = (1ULL << PIN_VBAT) | (1ULL << PIN_VIN),
        .mode = GPIO_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&analog));
}

void app_main(void)
{
    usb_lock = xSemaphoreCreateMutex();

    usb_serial_jtag_driver_config_t usb = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb.tx_buffer_size = 8192;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    esp_log_set_vprintf(log_vprintf);

    park_unused_pins();

    // Console UART0 owns U0RXD (GPIO20). Point that input at a constant 1 so
    // DRDY edges are not parsed as console bytes, then take the pad as GPIO.
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, U0RXD_IN_IDX, false);
    gpio_config_t drdy = {
        .pin_bit_mask = 1ULL << PIN_DRDY,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&drdy));

    ESP_ERROR_CHECK(fpga_init());

    spi_bus_config_t bus = {
        .sclk_io_num = PIN_SCLK,
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED));  // 27-byte reads fit the FIFO
    ESP_ERROR_CHECK(ads_add(&ads[0], SPI2_HOST, kChipCfg[0].cs_mask, SPI_CLOCK_HZ, "EMG"));
    ESP_ERROR_CHECK(ads_add(&ads[1], SPI2_HOST, kChipCfg[1].cs_mask, SPI_CLOCK_HZ, "EEG"));
    ESP_ERROR_CHECK(ads_add_fast(&ads[0], SPI2_HOST, Z_SPI_HZ));
    ESP_ERROR_CHECK(ads_add_fast(&ads[1], SPI2_HOST, Z_SPI_HZ));

    frame_q = xQueueCreate(128, sizeof(frame_t));
    ESP_LOGI(TAG, "data link %s", DATA_LINK == DATA_LINK_BLE ? "ble" : "usb");
    ESP_LOGI(TAG, "audio %s", AUDIO_ENABLED ? "on" : "off");
#if AUDIO_ENABLED
    if (audio_init() != ESP_OK) ESP_LOGE(TAG, "audio init failed");
#endif
    if (link_ble_init() != ESP_OK) ESP_LOGE(TAG, "ble init failed");
    xTaskCreate(z_tx_task, "ztx", 5120, NULL, 8, &z_tx_h);
#if AUDIO_ENABLED
    xTaskCreate(audio_tx_task, "audtx", 4096, NULL, 6, NULL);
#endif
    xTaskCreate(tx_task, "tx", 3072, NULL, 5, NULL);
    xTaskCreate(acq_task, "acq", 8192, NULL, 10, &acq_task_h);
    xTaskCreate(cmd_task, "cmd", 4096, NULL, 3, NULL);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_DRDY, drdy_isr, NULL));
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_DRDY, GPIO_INTR_NEGEDGE));
}
