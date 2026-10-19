#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define CSI_CAPTURE_MAX_SUBCARRIERS 128

typedef struct {
    uint32_t seq;
    uint32_t timestamp_ms;
    int8_t rssi;
    uint16_t subcarrier_count;
    float amplitudes[CSI_CAPTURE_MAX_SUBCARRIERS];
} csi_capture_frame_t;

typedef struct {
    float activity;
    float threshold;
    bool moving;
    bool warmup;
    float confidence;
    float amp_delta;
    float phase_delta;
    float spectral_entropy;
    float low_freq_ratio;
} csi_capture_motion_info_t;

esp_err_t csi_capture_init(void);
esp_err_t csi_capture_start(void);
esp_err_t csi_capture_stop(void);
esp_err_t csi_capture_start_traffic(const char *target_ip, uint32_t interval_ms);
esp_err_t csi_capture_get_latest(csi_capture_frame_t *out);
esp_err_t csi_capture_get_motion_info(csi_capture_motion_info_t *out);
uint32_t csi_capture_get_frame_count(void);
