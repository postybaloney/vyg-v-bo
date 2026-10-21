#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "ads1299.h"

// One impedance cycle. Little-endian, packed, 1062 bytes (crc included).
//   parity 0: channels 13 and 14 read SRB1 and BIAS (commands 0x15, 0x11).
//   parity 1: channels 15 and 16 read them (commands 0x1D, 0x19).
//   Channels 1-12 are electrodes every cycle. The other of 13/14 and 15/16
//   measures its own electrodes. A host with Z_REF_BIAS_ENABLED 0 ignores
//   the SRB1/BIAS readings.
//   A disabled ADC's eight codes stay zero.
//   ok      1 when all four phases were captured
//   dt_us   [phase][sample] microseconds from the first sample of that phase
//   code    [phase][adc 0..15][sample] signed 24-bit samples, gain 1
//           phase 0 vr, 1 vf, 2 ir, 3 if
//           adc 0..7 = EMG (ADC channels 1..8), 8..15 = EEG (ADC channels 9..16)
//   crc     esp_crc32_le over the preceding bytes
#define Z_PHASES  4
#define Z_ADC     16
#define Z_SAMPLES 4

typedef struct __attribute__((packed)) {
    uint8_t  parity;
    uint8_t  ok;
    uint16_t dt_us[Z_PHASES][Z_SAMPLES];
    int32_t  code[Z_PHASES][Z_ADC][Z_SAMPLES];
    uint32_t crc;
} z_packet_t;

// 16 MHz frame reads. Normal acquisition stays on the slower device.
#define Z_SPI_HZ (16 * 1000 * 1000)

// Call from the task the DRDY ISR notifies. *run is cleared by the host command
// to leave the mode; capture returns ESP_ERR_INVALID_STATE once that happens.
esp_err_t z_configure(ads_chip_t ads[2]);
esp_err_t z_capture_cycle(ads_chip_t ads[2], uint8_t parity, volatile uint8_t *run, z_packet_t *out);
