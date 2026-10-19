#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "lwip/inet.h"
#include "ping/ping_sock.h"
#include "csi_capture.h"

static const char *TAG = "csi_capture";

#define CSI_HISTORY_LEN         64
#define ACTIVITY_HISTORY_LEN    8
#define WARMUP_MS               15000
#define MOTION_ENTER_FRAMES     2
#define MOTION_EXIT_FRAMES      5
#define EWMA_ALPHA              0.25f
#define THRESHOLD_SIGMA         4.0f
#define MIN_THRESHOLD           1.0f
#define HYSTERESIS_LOW_RATIO    0.70f
#define FFT_WINDOW              64
#define FFT_LOW_BIN_MIN         2
#define FFT_LOW_BIN_MAX         32
#define FEATURE_WEIGHT_PHASE    1.0f
#define FEATURE_WEIGHT_ENTROPY  10.0f
#define FEATURE_WEIGHT_LOWFREQ  20.0f
#define PI_F                    3.14159265358979323846f

static csi_capture_frame_t s_latest;
static bool s_valid;
static uint32_t s_frame_count;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static esp_ping_handle_t s_ping;

static float s_amp_history[CSI_HISTORY_LEN][CSI_CAPTURE_MAX_SUBCARRIERS];
static uint16_t s_amp_history_counts[CSI_HISTORY_LEN];
static float s_phase_history[CSI_HISTORY_LEN][CSI_CAPTURE_MAX_SUBCARRIERS];
static uint16_t s_phase_history_counts[CSI_HISTORY_LEN];
static uint16_t s_history_idx;
static uint16_t s_history_count;

static float s_activity_ring[ACTIVITY_HISTORY_LEN];
static uint16_t s_activity_idx;
static uint16_t s_activity_count;
static float s_activity_ewma;
static bool s_activity_ewma_valid;

static uint32_t s_warmup_start_ms;
static double s_warmup_sum;
static double s_warmup_sum_sq;
static uint32_t s_warmup_count;
static bool s_warmup_finished;

static bool s_moving;
static uint16_t s_moving_count;
static uint16_t s_still_count;
static float s_threshold;
static float s_confidence;

static float s_amp_delta;
static float s_phase_delta;
static float s_spectral_entropy;
static float s_low_freq_ratio;

static float s_fft_signal[FFT_WINDOW];
static float s_fft_real[FFT_WINDOW];
static float s_fft_imag[FFT_WINDOW];
static uint16_t s_fft_count;

static float median_activity(void)
{
    float values[ACTIVITY_HISTORY_LEN];
    uint16_t n = s_activity_count;
    if (n == 0) {
        return 0.0f;
    }

    for (uint16_t i = 0; i < n; i++) {
        uint16_t idx = (s_activity_idx + ACTIVITY_HISTORY_LEN - n + i) % ACTIVITY_HISTORY_LEN;
        values[i] = s_activity_ring[idx];
    }

    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = i + 1; j < n; j++) {
            if (values[j] < values[i]) {
                float tmp = values[i];
                values[i] = values[j];
                values[j] = tmp;
            }
        }
    }

    uint16_t mid = n / 2;
    if (n % 2 == 0) {
        return (values[mid - 1] + values[mid]) * 0.5f;
    }
    return values[mid];
}

static void fft_radix2(float *real, float *imag, uint16_t n)
{
    uint16_t j = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (j > i) {
            float tmp = real[i];
            real[i] = real[j];
            real[j] = tmp;
            tmp = imag[i];
            imag[i] = imag[j];
            imag[j] = tmp;
        }

        uint16_t m = n >> 1;
        while (m >= 1 && (j & m)) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    for (uint16_t len = 2; len <= n; len <<= 1) {
        float angle = -2.0f * PI_F / (float)len;
        float wlen_real = cosf(angle);
        float wlen_imag = sinf(angle);
        for (uint16_t i = 0; i < n; i += len) {
            float w_real = 1.0f;
            float w_imag = 0.0f;
            for (uint16_t k = 0; k < (len >> 1); k++) {
                uint16_t a = i + k;
                uint16_t b = a + (len >> 1);
                float tr = w_real * real[b] - w_imag * imag[b];
                float ti = w_real * imag[b] + w_imag * real[b];
                real[b] = real[a] - tr;
                imag[b] = imag[a] - ti;
                real[a] += tr;
                imag[a] += ti;

                float next_real = w_real * wlen_real - w_imag * wlen_imag;
                float next_imag = w_real * wlen_imag + w_imag * wlen_real;
                w_real = next_real;
                w_imag = next_imag;
            }
        }
    }
}

static void update_low_freq_ratio(float amp_delta)
{
    if (s_fft_count < FFT_WINDOW) {
        s_fft_signal[s_fft_count] = amp_delta;
        s_fft_count++;
        s_low_freq_ratio = 0.0f;
        return;
    }

    memmove(s_fft_signal, s_fft_signal + 1, (FFT_WINDOW - 1) * sizeof(float));
    s_fft_signal[FFT_WINDOW - 1] = amp_delta;

    for (uint16_t i = 0; i < FFT_WINDOW; i++) {
        s_fft_real[i] = s_fft_signal[i];
        s_fft_imag[i] = 0.0f;
    }

    fft_radix2(s_fft_real, s_fft_imag, FFT_WINDOW);

    float low_energy = 0.0f;
    float high_energy = 0.0f;
    for (uint16_t k = 0; k < FFT_WINDOW; k++) {
        float magnitude_sq = s_fft_real[k] * s_fft_real[k] + s_fft_imag[k] * s_fft_imag[k];
        if (k >= FFT_LOW_BIN_MIN && k <= FFT_LOW_BIN_MAX) {
            low_energy += magnitude_sq;
        } else if (k > FFT_LOW_BIN_MAX) {
            high_energy += magnitude_sq;
        }
    }

    float total = low_energy + high_energy;
    s_low_freq_ratio = (total > 0.0001f) ? (low_energy / total) : 0.0f;
}

static void process_activity(float raw_activity)
{
    s_activity_ring[s_activity_idx] = raw_activity;
    s_activity_idx = (s_activity_idx + 1) % ACTIVITY_HISTORY_LEN;
    if (s_activity_count < ACTIVITY_HISTORY_LEN) {
        s_activity_count++;
    }

    float filtered = median_activity();
    if (!s_activity_ewma_valid) {
        s_activity_ewma = filtered;
        s_activity_ewma_valid = true;
    } else {
        s_activity_ewma = EWMA_ALPHA * filtered + (1.0f - EWMA_ALPHA) * s_activity_ewma;
    }

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (!s_warmup_finished) {
        if (s_warmup_start_ms == 0) {
            s_warmup_start_ms = now_ms;
        }

        s_warmup_sum += s_activity_ewma;
        s_warmup_sum_sq += (double)s_activity_ewma * s_activity_ewma;
        s_warmup_count++;

        if ((now_ms - s_warmup_start_ms) >= WARMUP_MS && s_warmup_count >= 20) {
            double mean = s_warmup_sum / s_warmup_count;
            double variance = s_warmup_sum_sq / s_warmup_count - mean * mean;
            if (variance < 0.0) {
                variance = 0.0;
            }
            float stddev = sqrtf((float)variance);
            s_threshold = (float)mean + THRESHOLD_SIGMA * stddev;
            if (s_threshold < MIN_THRESHOLD) {
                s_threshold = MIN_THRESHOLD;
            }
            s_warmup_finished = true;
            s_moving = false;
            s_confidence = 0.0f;
            ESP_LOGI(TAG, "Warmup finished: mean=%.2f std=%.2f threshold=%.2f",
                     (float)mean, stddev, s_threshold);
        }
        return;
    }

    float low_threshold = s_threshold * HYSTERESIS_LOW_RATIO;
    if (!s_moving) {
        if (s_activity_ewma > s_threshold) {
            s_moving_count++;
        } else {
            s_moving_count = 0;
        }
        if (s_moving_count >= MOTION_ENTER_FRAMES) {
            s_moving = true;
            s_still_count = 0;
            ESP_LOGI(TAG, "Motion detected");
        }
    } else {
        if (s_activity_ewma < low_threshold) {
            s_still_count++;
        } else {
            s_still_count = 0;
        }
        if (s_still_count >= MOTION_EXIT_FRAMES) {
            s_moving = false;
            s_moving_count = 0;
            ESP_LOGI(TAG, "Motion ended");
        }
    }

    float range = s_threshold - low_threshold;
    if (range < 0.0001f) {
        range = 0.0001f;
    }
    s_confidence = (s_activity_ewma - low_threshold) / range;
    if (s_confidence < 0.0f) {
        s_confidence = 0.0f;
    } else if (s_confidence > 1.0f) {
        s_confidence = 1.0f;
    }
}

static void csi_rx_cb(void *ctx, wifi_csi_info_t *data)
{
    (void)ctx;
    if (data == NULL || data->buf == NULL || data->len == 0) {
        return;
    }

    int start = data->first_word_invalid ? 4 : 0;
    int samples = (int)((data->len - start) / 2);
    if (samples <= 0) {
        return;
    }
    if (samples > CSI_CAPTURE_MAX_SUBCARRIERS) {
        samples = CSI_CAPTURE_MAX_SUBCARRIERS;
    }

    float amp_delta = 0.0f;
    float phase_delta = 0.0f;
    float amplitude_sum = 0.0f;
    uint16_t prev_idx = (s_history_idx + CSI_HISTORY_LEN - 1) % CSI_HISTORY_LEN;

    portENTER_CRITICAL(&s_lock);
    s_latest.seq = data->rx_seq;
    s_latest.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    s_latest.rssi = data->rx_ctrl.rssi;
    s_latest.subcarrier_count = (uint16_t)samples;

    for (int i = 0; i < samples; i++) {
        int idx = start + i * 2;
        int8_t imag = data->buf[idx];
        int8_t real = data->buf[idx + 1];
        float amplitude = sqrtf((float)(imag * imag + real * real));
        float phase = atan2f((float)imag, (float)real);
        s_latest.amplitudes[i] = amplitude;
        amplitude_sum += amplitude;

        if (s_history_count > 0 && i < s_amp_history_counts[prev_idx]) {
            amp_delta += fabsf(amplitude - s_amp_history[prev_idx][i]);

            float dphase = phase - s_phase_history[prev_idx][i];
            while (dphase > PI_F) {
                dphase -= 2.0f * PI_F;
            }
            while (dphase < -PI_F) {
                dphase += 2.0f * PI_F;
            }
            phase_delta += fabsf(dphase);
        }
        s_amp_history[s_history_idx][i] = amplitude;
        s_phase_history[s_history_idx][i] = phase;
    }

    s_amp_history_counts[s_history_idx] = (uint16_t)samples;
    s_phase_history_counts[s_history_idx] = (uint16_t)samples;
    s_history_idx = (s_history_idx + 1) % CSI_HISTORY_LEN;
    if (s_history_count < CSI_HISTORY_LEN) {
        s_history_count++;
    }

    s_frame_count++;
    s_valid = true;
    portEXIT_CRITICAL(&s_lock);

    float spectral_entropy = 0.0f;
    if (amplitude_sum > 0.0001f) {
        for (int i = 0; i < samples; i++) {
            float p = s_latest.amplitudes[i] / amplitude_sum;
            if (p > 0.0f) {
                spectral_entropy -= p * logf(p);
            }
        }
    }

    update_low_freq_ratio(amp_delta);
    s_amp_delta = amp_delta;
    s_phase_delta = phase_delta;
    s_spectral_entropy = spectral_entropy;

    float fused_raw = amp_delta
                    + FEATURE_WEIGHT_PHASE * phase_delta
                    + FEATURE_WEIGHT_ENTROPY * spectral_entropy
                    + FEATURE_WEIGHT_LOWFREQ * s_low_freq_ratio;
    process_activity(fused_raw);
}

esp_err_t csi_capture_init(void)
{
    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = false,
        .stbc_htltf2_en = false,
        .ltf_merge_en = false,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = 0,
        .dump_ack_en = false,
    };

    memset(&s_latest, 0, sizeof(s_latest));
    memset(s_amp_history, 0, sizeof(s_amp_history));
    memset(s_amp_history_counts, 0, sizeof(s_amp_history_counts));
    memset(s_phase_history, 0, sizeof(s_phase_history));
    memset(s_phase_history_counts, 0, sizeof(s_phase_history_counts));
    memset(s_activity_ring, 0, sizeof(s_activity_ring));
    memset(s_fft_signal, 0, sizeof(s_fft_signal));
    memset(s_fft_real, 0, sizeof(s_fft_real));
    memset(s_fft_imag, 0, sizeof(s_fft_imag));

    s_valid = false;
    s_frame_count = 0;
    s_history_idx = 0;
    s_history_count = 0;
    s_activity_idx = 0;
    s_activity_count = 0;
    s_activity_ewma = 0.0f;
    s_activity_ewma_valid = false;
    s_amp_delta = 0.0f;
    s_phase_delta = 0.0f;
    s_spectral_entropy = 0.0f;
    s_low_freq_ratio = 0.0f;
    s_fft_count = 0;
    s_warmup_start_ms = 0;
    s_warmup_sum = 0.0;
    s_warmup_sum_sq = 0.0;
    s_warmup_count = 0;
    s_warmup_finished = false;
    s_moving = false;
    s_moving_count = 0;
    s_still_count = 0;
    s_threshold = 0.0f;
    s_confidence = 0.0f;

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_rx_cb, NULL));
    ESP_LOGI(TAG, "CSI callback configured");
    return ESP_OK;
}

esp_err_t csi_capture_start(void)
{
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    ESP_LOGI(TAG, "CSI enabled");
    return ESP_OK;
}

esp_err_t csi_capture_stop(void)
{
    if (s_ping != NULL) {
        esp_ping_stop(s_ping);
        esp_ping_delete_session(s_ping);
        s_ping = NULL;
    }

    ESP_ERROR_CHECK(esp_wifi_set_csi(false));
    return ESP_OK;
}

esp_err_t csi_capture_start_traffic(const char *target_ip, uint32_t interval_ms)
{
    if (target_ip == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ping != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    ping_config.count = ESP_PING_COUNT_INFINITE;
    ping_config.interval_ms = interval_ms;
    ping_config.timeout_ms = 1000;
    ping_config.data_size = 32;
    ping_config.target_addr.type = ESP_IPADDR_TYPE_V4;
    ip4addr_aton(target_ip, &ping_config.target_addr.u_addr.ip4);

    esp_ping_callbacks_t callbacks = {0};
    ESP_ERROR_CHECK(esp_ping_new_session(&ping_config, &callbacks, &s_ping));
    ESP_ERROR_CHECK(esp_ping_start(s_ping));
    ESP_LOGI(TAG, "CSI ping traffic started: %s every %lu ms", target_ip,
             (unsigned long)interval_ms);
    return ESP_OK;
}

esp_err_t csi_capture_get_latest(csi_capture_frame_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    if (!s_valid) {
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_NOT_FOUND;
    }
    *out = s_latest;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t csi_capture_get_motion_info(csi_capture_motion_info_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    out->activity = s_activity_ewma;
    out->threshold = s_threshold;
    out->moving = s_moving;
    out->warmup = !s_warmup_finished;
    out->confidence = s_confidence;
    out->amp_delta = s_amp_delta;
    out->phase_delta = s_phase_delta;
    out->spectral_entropy = s_spectral_entropy;
    out->low_freq_ratio = s_low_freq_ratio;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

uint32_t csi_capture_get_frame_count(void)
{
    portENTER_CRITICAL(&s_lock);
    uint32_t count = s_frame_count;
    portEXIT_CRITICAL(&s_lock);
    return count;
}
