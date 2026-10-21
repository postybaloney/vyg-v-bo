#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "driver/spi_master.h"

// ---- registers ----
#define ADS_REG_ID          0x00
#define ADS_REG_CONFIG1     0x01
#define ADS_REG_CONFIG2     0x02
#define ADS_REG_CONFIG3     0x03
#define ADS_REG_CH1SET      0x05   // CH1SET..CH8SET = 0x05..0x0C
#define ADS_REG_BIAS_SENSP  0x0D
#define ADS_REG_BIAS_SENSN  0x0E
#define ADS_REG_MISC1       0x15

// ---- commands ----
#define ADS_CMD_WAKEUP  0x02
#define ADS_CMD_RESET   0x06
#define ADS_CMD_START   0x08
#define ADS_CMD_STOP    0x0A
#define ADS_CMD_RDATAC  0x10
#define ADS_CMD_SDATAC  0x11
#define ADS_CMD_RDATA   0x12
#define ADS_CMD_RREG    0x20
#define ADS_CMD_WREG    0x40

#define ADS_ID_ADS1299_8  0x3E

// CONFIG1: [7]=1 [6]=DAISY_EN(1=multiple readback) [5]=CLK_EN [4:3]=10 [2:0]=DR
#define ADS_CONFIG1_BASE           0x90
#define ADS_CONFIG1_MULTI_READBACK 0x40
#define ADS_CONFIG1_CLK_EN         0x20
#define ADS_DR_16000 0
#define ADS_DR_8000  1
#define ADS_DR_4000  2
#define ADS_DR_2000  3
#define ADS_DR_1000  4
#define ADS_DR_500   5
#define ADS_DR_250   6

#define ADS_CONFIG2_NORMAL   0xC0
#define ADS_CONFIG2_TEST     0xD0   // internal test signal on, 1x amp, ~1 Hz
#define ADS_CONFIG3_REF      0xE0   // internal ref buffer on, bias off
#define ADS_CONFIG3_REF_BIAS 0xEC   // + bias buffer, internal BIASREF
#define ADS_MISC1_SRB1       0x20   // SRB1 -> all INxN

// CHnSET: [7]=PD [6:4]=gain [3]=SRB2 [2:0]=mux
#define ADS_MUX_NORMAL 0x0
#define ADS_MUX_SHORT  0x1
#define ADS_MUX_TEST   0x5
#define ADS_CHSET_OFF  0x81         // powered down, input shorted

#define ADS_FRAME_BYTES 27          // 3 status + 8 x 24-bit

typedef enum { ADS_MONOPOLAR = 0, ADS_BIPOLAR = 1 } ads_mode_t;
typedef enum { ADS_INPUT_NORMAL, ADS_INPUT_SHORTED, ADS_INPUT_TEST } ads_input_t;

typedef struct {
    spi_device_handle_t dev;
    spi_device_handle_t dev_fast;  // optional higher SCLK for impedance bursts; NULL uses dev
    const char *name;
    uint8_t cs_mask;   // FPGA chip-select value, held around each transfer
} ads_chip_t;

static inline uint8_t ads_gain_code(int gain) {
    switch (gain) {
        case 1: return 0; case 2: return 1; case 4: return 2; case 6: return 3;
        case 8: return 4; case 12: return 5; default: return 6;  // 24
    }
}

static inline int32_t ads_s24(const uint8_t *b) {
    return (int32_t)((uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8) >> 8;
}

// cs_mask is written to the FPGA (no SPI CS pin on this board).
esp_err_t ads_add(ads_chip_t *c, spi_host_device_t host, uint8_t cs_mask, int clock_hz, const char *name);
esp_err_t ads_add_fast(ads_chip_t *c, spi_host_device_t host, int clock_hz);
esp_err_t ads_cmd(ads_chip_t *c, uint8_t cmd);
esp_err_t ads_wreg(ads_chip_t *c, uint8_t reg, const uint8_t *vals, size_t n);
esp_err_t ads_rreg(ads_chip_t *c, uint8_t reg, uint8_t *vals, size_t n);
// RDATAC mode: clocks out one 27-byte frame (MOSI held low).
esp_err_t ads_read_frame(ads_chip_t *c, uint8_t out[ADS_FRAME_BYTES]);
// SPI only. Caller already asserted this chip's CS and releases it.
// fast uses dev_fast when it has been added.
esp_err_t ads_shift_frame(ads_chip_t *c, uint8_t out[ADS_FRAME_BYTES], bool fast);
