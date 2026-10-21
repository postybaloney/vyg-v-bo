#include "fpga.h"
#include "config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "fpga";

static i2c_master_dev_handle_t dev_start;
static i2c_master_dev_handle_t dev_cs;

static esp_err_t add_dev(i2c_master_bus_handle_t bus, uint16_t addr, i2c_master_dev_handle_t *out)
{
    i2c_device_config_t d = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = FPGA_I2C_HZ,
    };
    return i2c_master_bus_add_device(bus, &d, out);
}

esp_err_t fpga_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) return err;
    err = add_dev(bus, FPGA_ADDR_START, &dev_start);
    if (err != ESP_OK) return err;
    err = add_dev(bus, FPGA_ADDR_CS, &dev_cs);
    if (err != ESP_OK) return err;
    ESP_LOGI(TAG, "I2C SDA=%d SCL=%d start@%02X cs@%02X",
             PIN_I2C_SDA, PIN_I2C_SCL, FPGA_ADDR_START, FPGA_ADDR_CS);
    return ESP_OK;
}

static esp_err_t write_reg(i2c_master_dev_handle_t dev, uint8_t value)
{
    uint8_t buf[2] = { FPGA_REG, value };
    return i2c_master_transmit(dev, buf, sizeof buf, 20);
}

esp_err_t fpga_cs(uint8_t mask)
{
    return write_reg(dev_cs, mask);
}

esp_err_t fpga_set_start(bool run)
{
    return write_reg(dev_start, run ? FPGA_START_RUN : FPGA_START_STOP);
}
