#include "impedance.h"
#include "config.h"
#include "fpga.h"
#include <string.h>
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Continuous impedance capture. One cycle:
//   SRB1 mode, up ramp (vr) then down ramp (vf),
//   INxP/INxN mode, up ramp (ir) then down ramp (if).
// The ADS is already converting on START. Writing slave 0x58, register 0x79,
// starts the current. Parity 0 writes 0x15 then 0x11: channels 13/14 read
// SRB1 and BIAS. Parity 1 writes 0x1D then 0x19: channels 15/16 do.
// Channels 1-12 are electrodes on every cycle. 13/14 and 15/16 alternate
// between that sense pair and their own electrodes.
//
// Those four bytes already select the EMG chip. Reading the EEG chip flips
// bit 1 only. Same-value writes are skipped, so a second read of one chip
// does not write the register again.
//
// CS is an I2C transaction, so four samples can land farther apart than the
// 62.5 us grid. dt_us is the real spacing; the host fit uses it.

_Static_assert(sizeof(z_packet_t) == 1062, "z_packet_t wire size");

static uint8_t z_cs_cur = 0xFF;

static bool z_drdy_low(void)
{
    return gpio_get_level(PIN_DRDY) == 0;
}

static esp_err_t z_cs(uint8_t value)
{
    if (value == z_cs_cur) return ESP_OK;
    esp_err_t err = fpga_cs(value);
    if (err == ESP_OK) z_cs_cur = value;
    return err;
}

static uint8_t z_cs_chip(uint8_t stim, int chip)
{
    if (chip == 0) return (uint8_t)((stim & ~0x02u) | 0x01u);
    return (uint8_t)(stim | 0x03u);
}

static bool z_adc_on(int chip)
{
    return chip == 0 ? EMG_ADC_ENABLED : EEG_ADC_ENABLED;
}

static uint8_t z_mask(int chip)
{
    return chip == 0 ? EMG_Z_MASK : EEG_Z_MASK;
}

static void z_discard(ads_chip_t ads[2])
{
    uint8_t raw[ADS_FRAME_BYTES];
    const uint8_t cs[2] = { FPGA_CS_EMG, FPGA_CS_EEG };
    for (int n = 0; n < 3 && z_drdy_low(); n++) {
        for (int chip = 0; chip < 2; chip++) {
            if (!z_adc_on(chip)) continue;
            if (z_cs(cs[chip]) != ESP_OK) return;
            if (ads_shift_frame(&ads[chip], raw, true) != ESP_OK) return;
        }
    }
    z_cs(FPGA_CS_NONE);
}

static esp_err_t z_stop(esp_err_t err)
{
    fpga_set_start(false);
    z_cs(FPGA_CS_NONE);
    return err;
}

// Gain 1, 16 kSPS, impedance-enabled channels powered, inputs normal.
// Z_DISABLE_BIAS writes CONFIG3 with the reference buffer still on and the bias
// buffer off. Otherwise CONFIG3 is left alone so the reference is not restarted.
esp_err_t z_configure(ads_chip_t ads[2])
{
    z_cs_cur = 0xFF;
    esp_err_t err = fpga_set_start(false);
    if (err != ESP_OK) return err;
    z_discard(ads);

    for (int i = 0; i < 2; i++) {
        if (!z_adc_on(i)) continue;
        err = ads_cmd(&ads[i], ADS_CMD_SDATAC);
        if (err != ESP_OK) return err;
    }

    for (int i = 0; i < 2; i++) {
        if (!z_adc_on(i)) continue;
        const ads_chip_cfg_t *c = &kChipCfg[i];
        const uint8_t mask = z_mask(i);
        uint8_t measure = mask;
        if (Z_CHIP_OF(Z_BIAS_ID) == i)
            measure |= (uint8_t)(1u << (Z_BIAS_ID & 7));
        const uint8_t cfg1 = ADS_CONFIG1_BASE | ADS_CONFIG1_MULTI_READBACK | ADS_DR_16000 |
            ((ADS_SHARED_CLOCK && EEG_ADC_ENABLED && i == 0) ? ADS_CONFIG1_CLK_EN : 0);
        uint8_t chs[10];
        for (int ch = 0; ch < 8; ch++)
            chs[ch] = (measure & (1u << ch)) ? 0x00 : ADS_CHSET_OFF;  // gain 1, or powered down
        chs[8] = (Z_DISABLE_BIAS || !c->bias) ? 0 : mask;
        chs[9] = (Z_DISABLE_BIAS || !c->bias || c->mode != ADS_BIPOLAR) ? 0 : mask;
        const uint8_t misc1 = ADS_MISC1_SRB1;
        const uint8_t cfg2 = ADS_CONFIG2_NORMAL;
        const uint8_t cfg3 = ADS_CONFIG3_REF;

        err = ads_wreg(&ads[i], ADS_REG_CONFIG1, &cfg1, 1);
        if (err != ESP_OK) return err;
        err = ads_wreg(&ads[i], ADS_REG_CONFIG2, &cfg2, 1);
        if (err != ESP_OK) return err;
        if (Z_DISABLE_BIAS) {
            err = ads_wreg(&ads[i], ADS_REG_CONFIG3, &cfg3, 1);
            if (err != ESP_OK) return err;
        }
        err = ads_wreg(&ads[i], ADS_REG_CH1SET, chs, sizeof chs);
        if (err != ESP_OK) return err;
        err = ads_wreg(&ads[i], ADS_REG_MISC1, &misc1, 1);
        if (err != ESP_OK) return err;

        uint8_t rb_cfg1 = 0, rb_cfg3 = 0, rb_misc = 0, rb_ch[8];
        err = ads_rreg(&ads[i], ADS_REG_CONFIG1, &rb_cfg1, 1);
        if (err != ESP_OK) return err;
        err = ads_rreg(&ads[i], ADS_REG_MISC1, &rb_misc, 1);
        if (err != ESP_OK) return err;
        err = ads_rreg(&ads[i], ADS_REG_CH1SET, rb_ch, 8);
        if (err != ESP_OK) return err;
        if (Z_DISABLE_BIAS) {
            err = ads_rreg(&ads[i], ADS_REG_CONFIG3, &rb_cfg3, 1);
            if (err != ESP_OK) return err;
            if (rb_cfg3 != cfg3) return ESP_FAIL;
        }
        if (rb_cfg1 != cfg1 || rb_misc != misc1 || memcmp(rb_ch, chs, 8) != 0) return ESP_FAIL;
    }
    z_cs_cur = 0xFF;  // register writes select CS on their own and leave it released

    for (int i = 0; i < 2; i++) {
        if (!z_adc_on(i)) continue;
        err = ads_cmd(&ads[i], ADS_CMD_RDATAC);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static esp_err_t z_set_srb1(ads_chip_t ads[2], bool srb1, volatile uint8_t *run)
{
    if (!*run) return z_stop(ESP_ERR_INVALID_STATE);
    esp_err_t err = fpga_set_start(false);
    if (err != ESP_OK) return err;
    z_discard(ads);
    for (int i = 0; i < 2; i++) {
        if (!z_adc_on(i)) continue;
        err = ads_cmd(&ads[i], ADS_CMD_SDATAC);
        if (err != ESP_OK) return err;
    }
    const uint8_t misc1 = srb1 ? ADS_MISC1_SRB1 : 0x00;
    for (int i = 0; i < 2; i++) {
        if (!z_adc_on(i)) continue;
        err = ads_wreg(&ads[i], ADS_REG_MISC1, &misc1, 1);
        if (err != ESP_OK) return err;
        err = ads_cmd(&ads[i], ADS_CMD_RDATAC);
        if (err != ESP_OK) return err;
    }
    z_cs_cur = 0xFF;
    return ESP_OK;
}

static bool z_wait_sample(volatile uint8_t *run)
{
    for (int i = 0; i < 8; i++) {
        if (!*run) return false;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
        if (!*run) return false;
        if (z_drdy_low()) return true;
    }
    return false;
}

static esp_err_t z_read(ads_chip_t ads[2], uint8_t stim, int32_t sample[Z_ADC])
{
    uint8_t raw[ADS_FRAME_BYTES];
    memset(sample, 0, sizeof(int32_t) * Z_ADC);
    // Chip 0 first: the arming byte already selects it, so this write is skipped.
    // Chip 1 flips bit 1 only. A plain chip-select would clear the ramp bits.
    for (int chip = 0; chip < 2; chip++) {
        if (!z_adc_on(chip)) continue;
        esp_err_t err = z_cs(z_cs_chip(stim, chip));
        if (err != ESP_OK) return err;
        err = ads_shift_frame(&ads[chip], raw, true);
        if (err != ESP_OK) return err;
        for (int ch = 0; ch < 8; ch++)
            sample[chip * 8 + ch] = ads_s24(&raw[3 + ch * 3]);
    }
    return ESP_OK;
}

static esp_err_t z_phase(ads_chip_t ads[2], uint8_t stim, z_packet_t *out, int phase,
                         volatile uint8_t *run)
{
    if (!*run) return z_stop(ESP_ERR_INVALID_STATE);
    ulTaskNotifyTake(pdTRUE, 0);
    // START is already high before the ramp byte. That byte starts the current.
    // A DRDY already pending finished before the current started.
    esp_err_t err = fpga_set_start(true);
    if (err != ESP_OK) return z_stop(err);
    if (z_drdy_low())
        z_discard(ads);
    err = z_cs(stim);
    if (err != ESP_OK) return z_stop(err);
    // Read it with the ramp byte still in the CS register. z_discard would
    // write a plain chip-select and clear the ramp.
    if (z_drdy_low()) {
        int32_t junk[Z_ADC];
        err = z_read(ads, stim, junk);
        if (err != ESP_OK) return z_stop(err);
    }

    int64_t t0 = 0;
    for (int s = 0; s < Z_SAMPLES; s++) {
        if (!*run) return z_stop(ESP_ERR_INVALID_STATE);
        if (!z_wait_sample(run)) return z_stop(*run ? ESP_ERR_TIMEOUT : ESP_ERR_INVALID_STATE);
        int64_t now = esp_timer_get_time();
        if (s == 0) t0 = now;
        int64_t dt = now - t0;
        if (dt < 0) dt = 0;
        if (dt > 65535) dt = 65535;
        out->dt_us[phase][s] = (uint16_t)dt;

        int32_t sample[Z_ADC];
        err = z_read(ads, stim, sample);
        if (err != ESP_OK) return z_stop(err);
        for (int ch = 0; ch < Z_ADC; ch++)
            out->code[phase][ch][s] = sample[ch];
    }

    fpga_set_start(false);
    z_discard(ads);
    return ESP_OK;
}

esp_err_t z_capture_cycle(ads_chip_t ads[2], uint8_t parity, volatile uint8_t *run, z_packet_t *out)
{
    memset(out, 0, sizeof *out);
    out->parity = parity & 1;
    // Exact FPGA opcodes. 0x15/0x11 steer channels 13/14 onto SRB1 and BIAS.
    // 0x1D/0x19 steer channels 15/16. No other bits.
    const uint8_t up = out->parity ? 0x1D : 0x15;
    const uint8_t down = out->parity ? 0x19 : 0x11;

    esp_err_t err = z_set_srb1(ads, true, run);
    if (err != ESP_OK) return err;
    err = z_phase(ads, up, out, 0, run);
    if (err != ESP_OK) return err;
    err = z_phase(ads, down, out, 1, run);
    if (err != ESP_OK) return err;
    err = z_set_srb1(ads, false, run);
    if (err != ESP_OK) return err;
    err = z_phase(ads, up, out, 2, run);
    if (err != ESP_OK) return err;
    err = z_phase(ads, down, out, 3, run);
    if (err != ESP_OK) return err;
    out->ok = 1;
    return ESP_OK;
}
