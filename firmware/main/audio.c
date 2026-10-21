#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_adc/adc_continuous.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "config.h"
#include "audio.h"

static const char *TAG = "audio";

#define ADC_HZ       64000
#define DECIM        4
#define FIR_TAPS     16
#define PLAY_CAP     1600
#define PLAY_PREFILL 640
#define PLAY_HIGH    1280
#define PLAY_LOW     320
#define MIC_QUEUE    8

// 16-tap Hamming-windowed sinc, cutoff 7 kHz at 64 kHz, unity DC gain.
static const float k_fir[FIR_TAPS] = {
    -0.00309499f, -0.00573700f, -0.00807224f, 0.00139243f,
    0.03621782f, 0.09778945f, 0.16741384f, 0.21409068f,
    0.21409068f, 0.16741384f, 0.09778945f, 0.03621782f,
    0.00139243f, -0.00807224f, -0.00573700f, -0.00309499f,
};
// One-pole high-pass, fc ≈ 40 Hz at 64 kHz: exp(-2π·40/64000).
static const float k_hp = 0.99608071f;

typedef struct {
    uint32_t seq;
    int16_t s[AUDIO_FRAME_SAMPLES];
} mic_frame_t;

static QueueHandle_t mic_q;
static adc_continuous_handle_t adc;
static i2s_chan_handle_t i2s_tx;

static int16_t play_ring[PLAY_CAP];
static volatile uint32_t play_w;
static volatile uint32_t play_r;
static volatile bool play_primed;
static uint32_t play_expect;
static bool play_have_seq;

#define PLAY_BARRIER() __asm__ __volatile__("" ::: "memory")

static uint32_t play_fill(void)
{
    return play_w - play_r;
}

static void play_push(const int16_t *s, int n)
{
    if (play_fill() + (uint32_t)n > PLAY_CAP) return;
    uint32_t w = play_w;
    for (int i = 0; i < n; i++)
        play_ring[(w + (uint32_t)i) % PLAY_CAP] = s[i];
    PLAY_BARRIER();
    play_w = w + (uint32_t)n;
}

void audio_play(const int16_t *samples, int n, uint32_t seq)
{
    if (!i2s_tx || !samples || n != AUDIO_FRAME_SAMPLES) return;
    if (play_have_seq) {
        uint32_t gap = seq - play_expect;
        if (gap > 0 && gap <= 3) {
            int16_t z[AUDIO_FRAME_SAMPLES] = {0};
            for (uint32_t i = 0; i < gap; i++) play_push(z, AUDIO_FRAME_SAMPLES);
        }
    }
    play_have_seq = true;
    play_expect = seq + 1;
    play_push(samples, n);
}

static void i2s_task(void *arg)
{
    int16_t mono[AUDIO_FRAME_SAMPLES];
    int16_t stereo[AUDIO_FRAME_SAMPLES * 2];
    for (;;) {
        uint32_t w = play_w;
        PLAY_BARRIER();
        uint32_t f = w - play_r;
        bool from_ring = false;
        if (!play_primed && f >= PLAY_PREFILL) play_primed = true;
        if (play_primed && f >= AUDIO_FRAME_SAMPLES) {
            if (f > PLAY_HIGH) play_r++;
            f = play_w - play_r;
            int dup = (f < PLAY_LOW && f >= AUDIO_FRAME_SAMPLES) ? 1 : 0;
            uint32_t r = play_r;
            if (dup) mono[0] = play_ring[r % PLAY_CAP];
            int take = AUDIO_FRAME_SAMPLES - dup;
            if ((uint32_t)take <= f) {
                for (int i = 0; i < take; i++)
                    mono[dup + i] = play_ring[(r + (uint32_t)i) % PLAY_CAP];
                play_r = r + (uint32_t)take;
                from_ring = true;
            }
        }
        if (play_primed && !from_ring) {
            play_primed = false;
            play_r = play_w;
        }
        if (!from_ring) memset(mono, 0, sizeof mono);
        for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
            stereo[i * 2] = mono[i];
            stereo[i * 2 + 1] = mono[i];
        }
        size_t wrote = 0;
        i2s_channel_write(i2s_tx, stereo, sizeof stereo, &wrote, 50);
    }
}

static int16_t scale_sample(float y)
{
    float g = y * 16.f;
    int v = (int)(g >= 0.f ? g + 0.5f : g - 0.5f);
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

static void adc_task(void *arg)
{
    uint8_t raw[1024];
    float hp_y = 0.f, hp_x = 0.f;
    bool hp_init = false;
    float delay[FIR_TAPS];
    memset(delay, 0, sizeof delay);
    uint32_t decim = 0;
    int16_t frame[AUDIO_FRAME_SAMPLES];
    int filled = 0;
    uint32_t seq = 0;

    for (;;) {
        uint32_t n = 0;
        esp_err_t err = adc_continuous_read(adc, raw, sizeof raw, &n, 20);
        if (err != ESP_OK || n < sizeof(adc_digi_output_data_t)) continue;
        int count = (int)(n / sizeof(adc_digi_output_data_t));
        const adc_digi_output_data_t *s = (const adc_digi_output_data_t *)raw;
        for (int i = 0; i < count; i++) {
            if (s[i].type2.channel != ADC_CHANNEL_3) continue;
            float x = (float)(int)s[i].type2.data - 2048.f;
            if (!hp_init) {
                hp_x = x;
                hp_y = 0.f;
                hp_init = true;
                continue;
            }
            hp_y = k_hp * (hp_y + x - hp_x);
            hp_x = x;
            memmove(delay, delay + 1, (FIR_TAPS - 1) * sizeof delay[0]);
            delay[FIR_TAPS - 1] = hp_y;
            if ((++decim & (DECIM - 1)) != 0) continue;
            float acc = 0.f;
            for (int k = 0; k < FIR_TAPS; k++) acc += k_fir[k] * delay[k];
            frame[filled++] = scale_sample(acc);
            if (filled < AUDIO_FRAME_SAMPLES) continue;
            filled = 0;
            if (!mic_q) continue;
            mic_frame_t out = {.seq = seq++};
            memcpy(out.s, frame, sizeof out.s);
            (void)xQueueSend(mic_q, &out, 0);
        }
    }
}

static esp_err_t adc_start(void)
{
    adc_continuous_handle_cfg_t hcfg = {
        .max_store_buf_size = 4096,
        .conv_frame_size = 1024,
    };
    hcfg.flags.flush_pool = 1;
    ESP_RETURN_ON_ERROR(adc_continuous_new_handle(&hcfg, &adc), TAG, "adc handle");

    adc_digi_pattern_config_t pat = {
        .atten = ADC_ATTEN_DB_12,
        .channel = ADC_CHANNEL_3,
        .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12,
    };
    adc_continuous_config_t cfg = {
        .pattern_num = 1,
        .adc_pattern = &pat,
        .sample_freq_hz = ADC_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    };
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
#pragma GCC diagnostic pop
    ESP_RETURN_ON_ERROR(adc_continuous_config(adc, &cfg), TAG, "adc config");
    ESP_RETURN_ON_ERROR(adc_continuous_start(adc), TAG, "adc start");
    if (xTaskCreate(adc_task, "adc", 5120, NULL, 9, NULL) != pdPASS) return ESP_ERR_NO_MEM;
    return ESP_OK;
}

static esp_err_t i2s_start(void)
{
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan.dma_desc_num = 4;
    chan.dma_frame_num = AUDIO_FRAME_SAMPLES;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan, &i2s_tx, NULL), TAG, "i2s chan");

    i2s_std_config_t std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCLK,
            .ws = PIN_I2S_LRCK,
            .dout = PIN_I2S_DOUT,
            .din = I2S_GPIO_UNUSED,
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(i2s_tx, &std), TAG, "i2s init");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(i2s_tx), TAG, "i2s enable");
    if (xTaskCreate(i2s_task, "i2s", 4096, NULL, 7, NULL) != pdPASS) return ESP_ERR_NO_MEM;
    return ESP_OK;
}

esp_err_t audio_init(void)
{
    mic_q = xQueueCreate(MIC_QUEUE, sizeof(mic_frame_t));
    if (!mic_q) return ESP_ERR_NO_MEM;
    esp_err_t err = adc_start();
    if (err != ESP_OK) return err;
    err = i2s_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "speaker init failed, mic still streaming");
        i2s_tx = NULL;
    }
    return ESP_OK;
}

bool audio_take_frame(int16_t samples[AUDIO_FRAME_SAMPLES], uint32_t *seq)
{
    if (!mic_q) {
        vTaskDelay(pdMS_TO_TICKS(200));
        return false;
    }
    mic_frame_t f;
    if (xQueueReceive(mic_q, &f, portMAX_DELAY) != pdTRUE) return false;
    memcpy(samples, f.s, sizeof f.s);
    *seq = f.seq;
    return true;
}
