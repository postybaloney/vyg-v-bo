#include "ads1299.h"
#include "fpga.h"
#include "config.h"
#include <string.h>
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "esp_attr.h"

esp_err_t ads_add(ads_chip_t *c, spi_host_device_t host, uint8_t cs_mask, int clock_hz, const char *name)
{
    spi_device_interface_config_t d = {
        .mode = 1,                    // CPOL=0, CPHA=1
        .clock_speed_hz = clock_hz,
        .spics_io_num = -1,           // CS is an FPGA I2C register, held around the whole transfer
        .queue_size = 1,
    };
    c->name = name;
    c->cs_mask = cs_mask;
    c->dev_fast = NULL;
    return spi_bus_add_device(host, &d, &c->dev);
}

esp_err_t ads_add_fast(ads_chip_t *c, spi_host_device_t host, int clock_hz)
{
    spi_device_interface_config_t d = {
        .mode = 1,
        .clock_speed_hz = clock_hz,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    return spi_bus_add_device(host, &d, &c->dev_fast);
}

// CS high time between commands is the I2C write that releases it, plus this gap.
static esp_err_t cs_release(esp_err_t err)
{
    esp_err_t rel = fpga_cs(FPGA_CS_NONE);
    esp_rom_delay_us(3);
    return err != ESP_OK ? err : rel;
}

// Byte-by-byte under one CS assertion, with a gap after each byte.
// Satisfies tSDECODE (4 tCLK ~ 2 us) between bytes of multi-byte commands at any SCLK.
static esp_err_t xfer_bytes(ads_chip_t *c, const uint8_t *tx, uint8_t *rx, size_t n)
{
    esp_err_t err = fpga_cs(c->cs_mask);
    if (err != ESP_OK) return err;
    err = spi_device_acquire_bus(c->dev, portMAX_DELAY);
    if (err == ESP_OK) {
        for (size_t i = 0; i < n && err == ESP_OK; i++) {
            spi_transaction_t t = {
                .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
                .length = 8,
            };
            t.tx_data[0] = tx ? tx[i] : 0x00;
            err = spi_device_polling_transmit(c->dev, &t);
            if (rx) rx[i] = t.rx_data[0];
            if (i + 1 < n) esp_rom_delay_us(2);  // tSDECODE, 4 tCLK
        }
        spi_device_release_bus(c->dev);
    }
    return cs_release(err);
}

esp_err_t ads_cmd(ads_chip_t *c, uint8_t cmd)
{
    return xfer_bytes(c, &cmd, NULL, 1);
}

esp_err_t ads_wreg(ads_chip_t *c, uint8_t reg, const uint8_t *vals, size_t n)
{
    uint8_t tx[2 + 24];
    if (n == 0 || n > 24) return ESP_ERR_INVALID_ARG;
    tx[0] = ADS_CMD_WREG | reg;
    tx[1] = (uint8_t)(n - 1);
    memcpy(&tx[2], vals, n);
    return xfer_bytes(c, tx, NULL, n + 2);
}

esp_err_t ads_rreg(ads_chip_t *c, uint8_t reg, uint8_t *vals, size_t n)
{
    uint8_t tx[2 + 24] = {0}, rx[2 + 24];
    if (n == 0 || n > 24) return ESP_ERR_INVALID_ARG;
    tx[0] = ADS_CMD_RREG | reg;
    tx[1] = (uint8_t)(n - 1);
    esp_err_t err = xfer_bytes(c, tx, rx, n + 2);
    memcpy(vals, &rx[2], n);
    return err;
}

esp_err_t ads_shift_frame(ads_chip_t *c, uint8_t out[ADS_FRAME_BYTES], bool fast)
{
    static DRAM_ATTR const uint8_t zeros[ADS_FRAME_BYTES] = {0};  // MOSI low in RDATAC
    spi_device_handle_t dev = (fast && c->dev_fast) ? c->dev_fast : c->dev;
    spi_transaction_t t = {
        .length = ADS_FRAME_BYTES * 8,
        .tx_buffer = zeros,
        .rx_buffer = out,
    };
    return spi_device_polling_transmit(dev, &t);
}

esp_err_t ads_read_frame(ads_chip_t *c, uint8_t out[ADS_FRAME_BYTES])
{
    esp_err_t err = fpga_cs(c->cs_mask);
    if (err != ESP_OK) return err;
    err = ads_shift_frame(c, out, false);
    return cs_release(err);
}
