#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "ads1299.h"

// ESP32-C3. GPIO2/8/9 are strapping pins and must be high at reset
// (MOSI is high-Z until the SPI driver starts; I2C needs external pull-ups).
// GPIO12-17 are in-package flash, GPIO18/19 are USB, GPIO21 is the log UART TX.
//
// package  GPIO   function
//  4       0      VBAT sense, divider already halves it
//  5       1      VIN sense, divider already halves it
//  6       2      SPI MOSI -> both ADS SDI
//  8       3      analog MEMS mic
//  9       4      SPI MISO <- both ADS SDO
// 10       5      I2S BCLK -> amp
// 12       6      SPI SCLK -> both ADS SCLK
// 13       7      I2S DOUT -> amp DIN
// 14       8      I2C SDA  -> FPGA and haptic
// 15       9      I2C SCL  -> FPGA and haptic
// 16      10      I2S LRCK -> amp
// 27      20      DRDY, active low. This is the U0RXD pad, used as GPIO.
//                 Either ADS being ready pulls it low. The FPGA does not latch it:
//                 it goes high again once every chip that was ready has been read.
#define PIN_VBAT     GPIO_NUM_0
#define PIN_VIN      GPIO_NUM_1
#define PIN_MOSI     GPIO_NUM_2
#define PIN_MIC      GPIO_NUM_3
#define PIN_MISO     GPIO_NUM_4
#define PIN_I2S_BCLK GPIO_NUM_5
#define PIN_SCLK     GPIO_NUM_6
#define PIN_I2S_DOUT GPIO_NUM_7
#define PIN_I2C_SDA  GPIO_NUM_8
#define PIN_I2C_SCL  GPIO_NUM_9
#define PIN_I2S_LRCK GPIO_NUM_10
#define PIN_DRDY     GPIO_NUM_20

#define SPI_CLOCK_HZ (4 * 1000 * 1000)
#define FPGA_I2C_HZ  400000

// FPGA GPIO router, 7-bit addresses. Each write is two bytes: register, value.
#define FPGA_ADDR_START  0x50
#define FPGA_ADDR_CS     0x58
#define FPGA_REG         0x79
#define FPGA_START_RUN   0x03   // shared START asserted
#define FPGA_START_STOP  0x01   // shared START deasserted
// CS slave 0x51, register 0x79. Bit 0 is enable, bit 1 is the target.
// 0x00 nothing, 0x01 CS1 (EMG), 0x02 target with enable clear (nothing), 0x03 CS2 (EEG).
#define FPGA_CS_NONE     0x00
#define FPGA_CS_EMG      0x01
#define FPGA_CS_EEG      0x03
// Impedance ramps are extra values on this same register: 0x15, 0x11, 0x1D, 0x19.
// Bit 0 is already CS enable and bit 1 is already clear (EMG). The arming write
// is that exact byte. Reading the EEG chip flips bit 1 only.

// ---- ADS ----
// 1: EMG outputs its clock (CLK_EN). EEG has CLKSEL=0 and takes that clock, so the
//    slower DRDY only asserts together with a faster one (rates are 2^n and share START).
// 0: both on internal oscillators. DRDY edges drift; the level check below still
//    avoids reading a chip that is not ready, but the fast stream can duplicate a sample
//    when only the slow chip caused the interrupt.
#define ADS_SHARED_CLOCK 1

// Channel source for both chips.
//   ADS_INPUT_SHORTED  PGA inputs tied together (noise floor)
//   ADS_INPUT_TEST     internal test signal, ~1 Hz square wave, 1x
// ADS_INPUT_NORMAL
#define ADS_INPUT_MODE ADS_INPUT_NORMAL

// 1: the chip is used. 0: it is not clocked, configured, or read.
#define EMG_ADC_ENABLED 1
#define EEG_ADC_ENABLED 1

// 1: channel on for sampling. 0: powered down. Bit n of the chip mask is channel n+1.
// A channel requires its ADC. Checked below.
#define EMG1_ENABLED 0
#define EMG2_ENABLED 0
#define EMG3_ENABLED 0
#define EMG4_ENABLED 0
#define EMG5_ENABLED 0
#define EMG6_ENABLED 0
#define EMG7_ENABLED 0
#define EMG8_ENABLED 0

#define EEG1_ENABLED 0
#define EEG2_ENABLED 0
#define EEG3_ENABLED 1
#define EEG4_ENABLED 1
#define EEG5_ENABLED 1
#define EEG6_ENABLED 1
#define EEG7_ENABLED 1
#define EEG8_ENABLED 1

// Impedance, separate from sampling. Same ADC rule.
// Reference and bias are wired to both chips. Z_REF_ELECTRODE and
// Z_BIAS_ELECTRODE name the input that measures each one ("EMG1".."EMG8"
// or "EEG1".."EEG8"). The reference input must be enabled for sampling and
// impedance. The bias input only has to be on an enabled ADC: its sampling
// flag stays out of the normal stream, and Z_DISABLE_BIAS stops the bias drive.
#define EMG1_Z_ENABLED 1
#define EMG2_Z_ENABLED 1
#define EMG3_Z_ENABLED 0
#define EMG4_Z_ENABLED 0
#define EMG5_Z_ENABLED 0
#define EMG6_Z_ENABLED 0
#define EMG7_Z_ENABLED 0
#define EMG8_Z_ENABLED 0

#define EEG1_Z_ENABLED 0
#define EEG2_Z_ENABLED 0
#define EEG3_Z_ENABLED 1
#define EEG4_Z_ENABLED 1
#define EEG5_Z_ENABLED 0
#define EEG6_Z_ENABLED 0
#define EEG7_Z_ENABLED 0
#define EEG8_Z_ENABLED 0

// 1: host reports SRB1 and BIAS impedance. 0: it does not.
// The FPGA steers those readings onto channels 13/14 or 15/16. It does not
// follow Z_REF_ELECTRODE / Z_BIAS_ELECTRODE. Those name the reference and
// bias electrodes themselves.
#define Z_REF_BIAS_ENABLED 1
#define Z_REF_ELECTRODE "EEG3"
#define Z_BIAS_ELECTRODE "EEG4"

// Channels averaged into the bias drive during normal sampling. Independent of
// the sampling and impedance flags. The EEG chip does not drive bias.
#define EMG1_BIAS_SENSE 0
#define EMG2_BIAS_SENSE 0
#define EMG3_BIAS_SENSE 0
#define EMG4_BIAS_SENSE 0
#define EMG5_BIAS_SENSE 0
#define EMG6_BIAS_SENSE 0
#define EMG7_BIAS_SENSE 0
#define EMG8_BIAS_SENSE 0

#define EEG1_BIAS_SENSE 0
#define EEG2_BIAS_SENSE 0
#define EEG3_BIAS_SENSE 0
#define EEG4_BIAS_SENSE 0
#define EEG5_BIAS_SENSE 0
#define EEG6_BIAS_SENSE 0
#define EEG7_BIAS_SENSE 0
#define EEG8_BIAS_SENSE 0

// 1: during impedance, power down the bias buffer and clear BIAS_SENS.
// The internal reference stays on. Streaming configures bias again on the way out.
#define Z_DISABLE_BIAS 1

#define ADS_CH_MASK(c1, c2, c3, c4, c5, c6, c7, c8) \
    ((uint8_t)(((c1) << 0) | ((c2) << 1) | ((c3) << 2) | ((c4) << 3) | \
               ((c5) << 4) | ((c6) << 5) | ((c7) << 6) | ((c8) << 7)))

typedef struct {
    ads_mode_t  mode;          // MONOPOLAR: ref electrode on SRB1 (tie SRB1 of both chips). BIPOLAR: pairs on INxP/INxN.
    uint8_t     channel_mask;  // bit n = channel n+1 enabled; disabled channels are powered down
    int         gain;          // 1, 2, 4, 6, 8, 12, 24
    uint8_t     rate;          // ADS_DR_*. Smaller code = faster. Slow rate must divide the fast one.
    ads_input_t input;         // NORMAL / SHORTED / TEST
    bool        bias;          // drive BIAS electrode + feed enabled channels into BIAS_SENS
    uint8_t     cs_mask;       // FPGA_CS_EMG / FPGA_CS_EEG
} ads_chip_cfg_t;

// ADS1299 PGA steps: 1, 2, 4, 6, 8, 12, 24. There is no 10.
#define EMG_GAIN 1
#define EEG_GAIN 8
#define ADS_GAIN_OK(g) ((g) == 1 || (g) == 2 || (g) == 4 || (g) == 6 || (g) == 8 || (g) == 12 || (g) == 24)
_Static_assert(ADS_GAIN_OK(EMG_GAIN) && ADS_GAIN_OK(EEG_GAIN),
               "ADS1299 gain is 1, 2, 4, 6, 8, 12, or 24");

static const ads_chip_cfg_t kChipCfg[2] __attribute__((unused)) = {
    { ADS_MONOPOLAR, ADS_CH_MASK(EMG1_ENABLED, EMG2_ENABLED, EMG3_ENABLED, EMG4_ENABLED,
                                 EMG5_ENABLED, EMG6_ENABLED, EMG7_ENABLED, EMG8_ENABLED),
      EMG_GAIN, ADS_DR_1000, ADS_INPUT_MODE, true,  FPGA_CS_EMG },
    { ADS_MONOPOLAR, ADS_CH_MASK(EEG1_ENABLED, EEG2_ENABLED, EEG3_ENABLED, EEG4_ENABLED,
                                 EEG5_ENABLED, EEG6_ENABLED, EEG7_ENABLED, EEG8_ENABLED),
      EEG_GAIN, ADS_DR_1000, ADS_INPUT_MODE, false, FPGA_CS_EEG },
};

#define EMG_Z_MASK ADS_CH_MASK(EMG1_Z_ENABLED, EMG2_Z_ENABLED, EMG3_Z_ENABLED, EMG4_Z_ENABLED, \
                               EMG5_Z_ENABLED, EMG6_Z_ENABLED, EMG7_Z_ENABLED, EMG8_Z_ENABLED)
#define EEG_Z_MASK ADS_CH_MASK(EEG1_Z_ENABLED, EEG2_Z_ENABLED, EEG3_Z_ENABLED, EEG4_Z_ENABLED, \
                               EEG5_Z_ENABLED, EEG6_Z_ENABLED, EEG7_Z_ENABLED, EEG8_Z_ENABLED)
#define EMG_BIAS_MASK ADS_CH_MASK(EMG1_BIAS_SENSE, EMG2_BIAS_SENSE, EMG3_BIAS_SENSE, EMG4_BIAS_SENSE, \
                                  EMG5_BIAS_SENSE, EMG6_BIAS_SENSE, EMG7_BIAS_SENSE, EMG8_BIAS_SENSE)
#define EEG_BIAS_MASK ADS_CH_MASK(EEG1_BIAS_SENSE, EEG2_BIAS_SENSE, EEG3_BIAS_SENSE, EEG4_BIAS_SENSE, \
                                  EEG5_BIAS_SENSE, EEG6_BIAS_SENSE, EEG7_BIAS_SENSE, EEG8_BIAS_SENSE)
#define Z_NAME_IS(s, lit) (__builtin_strcmp(s, lit) == 0)
#define Z_ID(s) ( \
    Z_NAME_IS(s, "EMG1") ? 0 : Z_NAME_IS(s, "EMG2") ? 1 : Z_NAME_IS(s, "EMG3") ? 2 : \
    Z_NAME_IS(s, "EMG4") ? 3 : Z_NAME_IS(s, "EMG5") ? 4 : Z_NAME_IS(s, "EMG6") ? 5 : \
    Z_NAME_IS(s, "EMG7") ? 6 : Z_NAME_IS(s, "EMG8") ? 7 : \
    Z_NAME_IS(s, "EEG1") ? 8 : Z_NAME_IS(s, "EEG2") ? 9 : Z_NAME_IS(s, "EEG3") ? 10 : \
    Z_NAME_IS(s, "EEG4") ? 11 : Z_NAME_IS(s, "EEG5") ? 12 : Z_NAME_IS(s, "EEG6") ? 13 : \
    Z_NAME_IS(s, "EEG7") ? 14 : Z_NAME_IS(s, "EEG8") ? 15 : -1)
#define Z_SAMPLE_AT(i) ( \
    (i) == 0 ? EMG1_ENABLED : (i) == 1 ? EMG2_ENABLED : (i) == 2 ? EMG3_ENABLED : \
    (i) == 3 ? EMG4_ENABLED : (i) == 4 ? EMG5_ENABLED : (i) == 5 ? EMG6_ENABLED : \
    (i) == 6 ? EMG7_ENABLED : (i) == 7 ? EMG8_ENABLED : \
    (i) == 8 ? EEG1_ENABLED : (i) == 9 ? EEG2_ENABLED : (i) == 10 ? EEG3_ENABLED : \
    (i) == 11 ? EEG4_ENABLED : (i) == 12 ? EEG5_ENABLED : (i) == 13 ? EEG6_ENABLED : \
    (i) == 14 ? EEG7_ENABLED : EEG8_ENABLED)
#define Z_Z_AT(i) ( \
    (i) == 0 ? EMG1_Z_ENABLED : (i) == 1 ? EMG2_Z_ENABLED : (i) == 2 ? EMG3_Z_ENABLED : \
    (i) == 3 ? EMG4_Z_ENABLED : (i) == 4 ? EMG5_Z_ENABLED : (i) == 5 ? EMG6_Z_ENABLED : \
    (i) == 6 ? EMG7_Z_ENABLED : (i) == 7 ? EMG8_Z_ENABLED : \
    (i) == 8 ? EEG1_Z_ENABLED : (i) == 9 ? EEG2_Z_ENABLED : (i) == 10 ? EEG3_Z_ENABLED : \
    (i) == 11 ? EEG4_Z_ENABLED : (i) == 12 ? EEG5_Z_ENABLED : (i) == 13 ? EEG6_Z_ENABLED : \
    (i) == 14 ? EEG7_Z_ENABLED : EEG8_Z_ENABLED)
#define Z_REF_ID Z_ID(Z_REF_ELECTRODE)
#define Z_BIAS_ID Z_ID(Z_BIAS_ELECTRODE)
#define Z_CHIP_OF(i) ((i) >= 8)

#define Z_REQUIRE(flag, adc, name) _Static_assert(!(flag) || (adc), name " on a disabled ADC")
Z_REQUIRE(EMG1_ENABLED, EMG_ADC_ENABLED, "EMG1 sampling");
Z_REQUIRE(EMG2_ENABLED, EMG_ADC_ENABLED, "EMG2 sampling");
Z_REQUIRE(EMG3_ENABLED, EMG_ADC_ENABLED, "EMG3 sampling");
Z_REQUIRE(EMG4_ENABLED, EMG_ADC_ENABLED, "EMG4 sampling");
Z_REQUIRE(EMG5_ENABLED, EMG_ADC_ENABLED, "EMG5 sampling");
Z_REQUIRE(EMG6_ENABLED, EMG_ADC_ENABLED, "EMG6 sampling");
Z_REQUIRE(EMG7_ENABLED, EMG_ADC_ENABLED, "EMG7 sampling");
Z_REQUIRE(EMG8_ENABLED, EMG_ADC_ENABLED, "EMG8 sampling");
Z_REQUIRE(EEG1_ENABLED, EEG_ADC_ENABLED, "EEG1 sampling");
Z_REQUIRE(EEG2_ENABLED, EEG_ADC_ENABLED, "EEG2 sampling");
Z_REQUIRE(EEG3_ENABLED, EEG_ADC_ENABLED, "EEG3 sampling");
Z_REQUIRE(EEG4_ENABLED, EEG_ADC_ENABLED, "EEG4 sampling");
Z_REQUIRE(EEG5_ENABLED, EEG_ADC_ENABLED, "EEG5 sampling");
Z_REQUIRE(EEG6_ENABLED, EEG_ADC_ENABLED, "EEG6 sampling");
Z_REQUIRE(EEG7_ENABLED, EEG_ADC_ENABLED, "EEG7 sampling");
Z_REQUIRE(EEG8_ENABLED, EEG_ADC_ENABLED, "EEG8 sampling");
Z_REQUIRE(EMG1_Z_ENABLED, EMG_ADC_ENABLED, "EMG1 impedance");
Z_REQUIRE(EMG2_Z_ENABLED, EMG_ADC_ENABLED, "EMG2 impedance");
Z_REQUIRE(EMG3_Z_ENABLED, EMG_ADC_ENABLED, "EMG3 impedance");
Z_REQUIRE(EMG4_Z_ENABLED, EMG_ADC_ENABLED, "EMG4 impedance");
Z_REQUIRE(EMG5_Z_ENABLED, EMG_ADC_ENABLED, "EMG5 impedance");
Z_REQUIRE(EMG6_Z_ENABLED, EMG_ADC_ENABLED, "EMG6 impedance");
Z_REQUIRE(EMG7_Z_ENABLED, EMG_ADC_ENABLED, "EMG7 impedance");
Z_REQUIRE(EMG8_Z_ENABLED, EMG_ADC_ENABLED, "EMG8 impedance");
Z_REQUIRE(EEG1_Z_ENABLED, EEG_ADC_ENABLED, "EEG1 impedance");
Z_REQUIRE(EEG2_Z_ENABLED, EEG_ADC_ENABLED, "EEG2 impedance");
Z_REQUIRE(EEG3_Z_ENABLED, EEG_ADC_ENABLED, "EEG3 impedance");
Z_REQUIRE(EEG4_Z_ENABLED, EEG_ADC_ENABLED, "EEG4 impedance");
Z_REQUIRE(EEG5_Z_ENABLED, EEG_ADC_ENABLED, "EEG5 impedance");
Z_REQUIRE(EEG6_Z_ENABLED, EEG_ADC_ENABLED, "EEG6 impedance");
Z_REQUIRE(EEG7_Z_ENABLED, EEG_ADC_ENABLED, "EEG7 impedance");
Z_REQUIRE(EEG8_Z_ENABLED, EEG_ADC_ENABLED, "EEG8 impedance");
Z_REQUIRE(EMG1_BIAS_SENSE, EMG_ADC_ENABLED, "EMG1 bias sense");
Z_REQUIRE(EMG2_BIAS_SENSE, EMG_ADC_ENABLED, "EMG2 bias sense");
Z_REQUIRE(EMG3_BIAS_SENSE, EMG_ADC_ENABLED, "EMG3 bias sense");
Z_REQUIRE(EMG4_BIAS_SENSE, EMG_ADC_ENABLED, "EMG4 bias sense");
Z_REQUIRE(EMG5_BIAS_SENSE, EMG_ADC_ENABLED, "EMG5 bias sense");
Z_REQUIRE(EMG6_BIAS_SENSE, EMG_ADC_ENABLED, "EMG6 bias sense");
Z_REQUIRE(EMG7_BIAS_SENSE, EMG_ADC_ENABLED, "EMG7 bias sense");
Z_REQUIRE(EMG8_BIAS_SENSE, EMG_ADC_ENABLED, "EMG8 bias sense");
Z_REQUIRE(EEG1_BIAS_SENSE, EEG_ADC_ENABLED, "EEG1 bias sense");
Z_REQUIRE(EEG2_BIAS_SENSE, EEG_ADC_ENABLED, "EEG2 bias sense");
Z_REQUIRE(EEG3_BIAS_SENSE, EEG_ADC_ENABLED, "EEG3 bias sense");
Z_REQUIRE(EEG4_BIAS_SENSE, EEG_ADC_ENABLED, "EEG4 bias sense");
Z_REQUIRE(EEG5_BIAS_SENSE, EEG_ADC_ENABLED, "EEG5 bias sense");
Z_REQUIRE(EEG6_BIAS_SENSE, EEG_ADC_ENABLED, "EEG6 bias sense");
Z_REQUIRE(EEG7_BIAS_SENSE, EEG_ADC_ENABLED, "EEG7 bias sense");
Z_REQUIRE(EEG8_BIAS_SENSE, EEG_ADC_ENABLED, "EEG8 bias sense");
_Static_assert(EEG_BIAS_MASK == 0, "EEG chip does not drive bias");
_Static_assert(EMG_ADC_ENABLED || EEG_ADC_ENABLED, "both ADCs disabled");
_Static_assert(!EEG_ADC_ENABLED || !ADS_SHARED_CLOCK || EMG_ADC_ENABLED,
               "EEG shares the EMG clock");
_Static_assert(Z_REF_ID >= 0, "Z_REF_ELECTRODE must be EMG1-8 or EEG1-8");
_Static_assert(Z_BIAS_ID >= 0, "Z_BIAS_ELECTRODE must be EMG1-8 or EEG1-8");
_Static_assert(Z_REF_ID != Z_BIAS_ID, "reference and bias are the same electrode");
_Static_assert(Z_CHIP_OF(Z_REF_ID) ? EEG_ADC_ENABLED : EMG_ADC_ENABLED,
               "reference electrode is on a disabled ADC");
_Static_assert(Z_CHIP_OF(Z_BIAS_ID) ? EEG_ADC_ENABLED : EMG_ADC_ENABLED,
               "bias electrode is on a disabled ADC");
_Static_assert(Z_SAMPLE_AT(Z_REF_ID), "reference electrode is not enabled for sampling");
_Static_assert(Z_Z_AT(Z_REF_ID), "reference electrode is not enabled for impedance");
// Reference and bias electrodes are an adjacent pair on one ADC. The ramp
// command does not use this pair.
#define Z_PAIR_LO ((Z_REF_ID & 7) < (Z_BIAS_ID & 7) ? (Z_REF_ID & 7) : (Z_BIAS_ID & 7))
#define Z_PAIR_HI ((Z_REF_ID & 7) > (Z_BIAS_ID & 7) ? (Z_REF_ID & 7) : (Z_BIAS_ID & 7))
_Static_assert(Z_CHIP_OF(Z_REF_ID) == Z_CHIP_OF(Z_BIAS_ID),
               "reference and bias must be on the same ADC");
_Static_assert((Z_PAIR_LO & 1) == 0 && Z_PAIR_HI == Z_PAIR_LO + 1,
               "reference and bias must be channels 1-2, 3-4, 5-6, or 7-8");

// ---- USB serial packets ----
// little-endian magic, then one type byte, then the payload.
// EMG and EEG are separate types so each sample index belongs to one chip.
// A faster chip simply sends more packets; the other chip is not repeated.
// 0x02 payload is u16 length + text. Sample payload is one frame_t.
#define FRAME_MAGIC 0x574F4F44u  // bytes on the wire: 44 4F 4F 57
#define FRAME_VERSION  3
#define FRAME_CHANNELS 8
#define CHIP_EMG       0
#define CHIP_EEG       1
#define PKT_TYPE_EMG   0x01
#define PKT_TYPE_LOG   0x02
#define PKT_TYPE_EEG   0x03
#define PKT_TYPE_Z     0x04
#define PKT_TYPE_AUDIO 0x05
// Host → device. 0xA5 0x5A, then a command.
// 0x00 / 0x01 are a single byte (impedance off / on).
// 0x10 is one playback frame: u32 seq, u16 count (160), int16 samples[160].
#define HOST_CMD_MAGIC0 0xA5
#define HOST_CMD_MAGIC1 0x5A
#define HOST_CMD_Z_OFF  0x00
#define HOST_CMD_Z_ON   0x01
#define HOST_CMD_AUDIO  0x10

// Mic and speaker. 0 disables both. The client reads this same line.
#define AUDIO_ENABLED 0

// EXG, impedance, audio, and host commands all use this link.
// The client reads this same line. Flip it and rebuild.
#define DATA_LINK_USB 0
#define DATA_LINK_BLE 1
#define DATA_LINK DATA_LINK_USB
