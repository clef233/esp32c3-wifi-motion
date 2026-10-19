#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "driver/temperature_sensor.h"
#include "driver/i2c.h"
#include "esp_netif.h"
#include "wifi_config.h"
#include "wifi_web.h"
#include "csi_capture.h"

static const char *TAG = "main";
static temperature_sensor_handle_t temp_sensor = NULL;
static bool aht21_found = false;
static bool ens160_found = false;

#define AHT21_ADDR           0x38
#define ENS160_ADDR          0x53
#define ENS160_REG_OPMODE    0x10
#define ENS160_REG_STATUS    0x20
#define ENS160_REG_AQI       0x21
#define ENS160_REG_TVOC      0x22
#define ENS160_REG_ECO2      0x24

static const char CSI_PAGE[] =
"<!doctype html><html><head><meta charset='utf-8'><title>WiFi CSI</title>"
"<style>html,body{margin:0;height:100%;background:#0d0d0d;color:#ddd;font-family:sans-serif;display:flex;flex-direction:column}"
"#status{height:28px;line-height:28px;padding:0 10px;font-weight:bold}"
"#activity{flex:0 0 30%;width:100%;background:#111}"
"#heat{flex:1 1 auto;width:100%;background:#111}</style></head><body>"
"<div id='status'>正在等待 CSI 数据...</div>"
"<canvas id='activity'></canvas><canvas id='heat'></canvas><script>"
"const ac=document.getElementById('activity'),hc=document.getElementById('heat'),actx=ac.getContext('2d'),hctx=hc.getContext('2d'),status=document.getElementById('status');"
"let hist=[],diffs=[],acts=[];let lastThreshold=0,lastMoving=false,lastWarmup=true,lastAmp=0,lastPhase=0,lastEntropy=0,lastLow=0;const MAX_DIFF=15;"
"function size(){ac.width=ac.clientWidth;ac.height=ac.clientHeight;hc.width=hc.clientWidth;hc.height=hc.clientHeight;}addEventListener('resize',size);size();setTimeout(size,0);"
"function median(arr){const s=[...arr].sort((a,b)=>a-b);const m=s.length>>1;return s.length%2?s[m]:(s[m-1]+s[m])/2;}"
"function drawActivity(){const w=ac.width,h=ac.height,n=acts.length;actx.fillStyle='#111';actx.fillRect(0,0,w,h);if(!n)return;const max=Math.max(...acts,lastThreshold,1);"
"actx.strokeStyle='#3d7bff';actx.lineWidth=2;actx.beginPath();for(let t=0;t<n;t++){const x=t*(w-1)/Math.max(1,n-1);const y=h-(acts[t]/max)*(h-10)-5;if(t===0)actx.moveTo(x,y);else actx.lineTo(x,y);}actx.stroke();"
"actx.strokeStyle='#f0a';actx.setLineDash([4,4]);actx.beginPath();actx.moveTo(0,h-(lastThreshold/max)*(h-10)-5);actx.lineTo(w,h-(lastThreshold/max)*(h-10)-5);actx.stroke();actx.setLineDash([]);"
"if(acts.length){const stateText=lastWarmup?'预热中...':(lastMoving?'检测到移动':'静止');status.textContent=stateText+' | 融合 '+acts[acts.length-1].toFixed(1)+' | 阈值 '+lastThreshold.toFixed(1)+' | 幅 '+lastAmp.toFixed(1)+' | 相 '+lastPhase.toFixed(1)+' | 熵 '+lastEntropy.toFixed(2)+' | 低频 '+lastLow.toFixed(2);}}"
"function drawHeat(){const w=hc.width,h=hc.height,n=diffs.length;hctx.fillStyle='#0a0a0a';hctx.fillRect(0,0,w,h);if(!n)return;const bw=w/Math.max(1,n-1);for(let t=0;t<n;t++){const d=diffs[t];if(!d)continue;const sh=h/d.length;for(let s=0;s<d.length;s++){const v=Math.min(1,d[s]/MAX_DIFF);const hue=(1-v)*220;const lig=8+v*42;hctx.fillStyle='hsl('+hue+',100%,'+lig+'%)';hctx.fillRect(t*bw,s*sh,bw+0.5,sh+0.5);}}}"
"async function poll(){try{const r=await fetch('/api/csi');const d=await r.json();if(d.subcarriers){const a=d.subcarriers;let diff=a.map(()=>0);if(hist.length){const p=hist[hist.length-1];diff=a.map((v,i)=>Math.abs(v-p[i]));}lastThreshold=Number(d.threshold)||0;lastMoving=!!d.moving;lastWarmup=!!d.warmup;lastAmp=Number(d.amp_delta)||0;lastPhase=Number(d.phase_delta)||0;lastEntropy=Number(d.spectral_entropy)||0;lastLow=Number(d.low_freq_ratio)||0;hist.push(a);diffs.push(diff);acts.push(Number(d.activity)||0);if(hist.length>240){hist.shift();diffs.shift();acts.shift();}drawActivity();drawHeat();}}catch(e){}setTimeout(poll,500);}poll();"
"</script></body></html>";

static esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_NUM_0, addr, &reg, 1, data, len,
                                        pdMS_TO_TICKS(100));
}

static esp_err_t aht21_read(float *temperature, float *humidity)
{
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    uint8_t data[7] = {0};
    esp_err_t err = i2c_master_write_read_device(I2C_NUM_0, AHT21_ADDR,
                                                 cmd, sizeof(cmd),
                                                 data, sizeof(data),
                                                 pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        return err;
    }

    uint32_t raw_humidity = ((uint32_t)data[1] << 12) |
                            ((uint32_t)data[2] << 4) |
                            ((uint32_t)(data[3] >> 4) & 0x0F);
    uint32_t raw_temperature = ((uint32_t)(data[3] & 0x0F) << 16) |
                               ((uint32_t)data[4] << 8) |
                               (uint32_t)data[5];

    *humidity = (float)raw_humidity * 100.0f / 1048576.0f;
    *temperature = (float)raw_temperature * 200.0f / 1048576.0f - 50.0f;
    return ESP_OK;
}

static esp_err_t ens160_read_air(int *tvoc, int *eco2, int *aqi)
{
    uint8_t tvoc_buf[2] = {0};
    uint8_t eco2_buf[2] = {0};
    uint8_t aqi_buf = 0;

    esp_err_t err = i2c_read_reg(ENS160_ADDR, ENS160_REG_TVOC, tvoc_buf, sizeof(tvoc_buf));
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_read_reg(ENS160_ADDR, ENS160_REG_ECO2, eco2_buf, sizeof(eco2_buf));
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_read_reg(ENS160_ADDR, ENS160_REG_AQI, &aqi_buf, 1);
    if (err != ESP_OK) {
        return err;
    }

    *tvoc = (int)((uint16_t)tvoc_buf[0] | ((uint16_t)tvoc_buf[1] << 8));
    *eco2 = (int)((uint16_t)eco2_buf[0] | ((uint16_t)eco2_buf[1] << 8));
    *aqi = (int)aqi_buf;
    return ESP_OK;
}

static void sensors_init(void)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_0,
        .scl_io_num = GPIO_NUM_1,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    uint8_t probe = 0x00;
    esp_err_t aht_err = i2c_master_write_to_device(I2C_NUM_0, AHT21_ADDR,
                                                   &probe, 1,
                                                   pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "AHT21 I2C: %s", aht_err == ESP_OK ? "found" : "not found");
    aht21_found = (aht_err == ESP_OK);

    uint8_t opmode_write[2] = {ENS160_REG_OPMODE, 0x02};
    esp_err_t err = i2c_master_write_to_device(I2C_NUM_0, ENS160_ADDR,
                                               opmode_write, sizeof(opmode_write),
                                               pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "ENS160 I2C: not found (%s)", esp_err_to_name(err));
        ens160_found = false;
    } else {
        ESP_LOGI(TAG, "ENS160 I2C: found");
        ens160_found = true;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}

static const char OTA_PAGE[] =
"<!doctype html><html><head><meta charset='utf-8'><title>OTA</title></head><body>"
"<h2>WiFi OTA 升级</h2>"
"<p>选择编译生成的 esp32c3-wifi-motion.bin</p>"
"<input type='file' id='f' accept='.bin'>"
"<button onclick='upload()'>开始升级</button>"
"<p id='s'></p>"
"<script>"
"async function upload(){const f=document.getElementById('f').files[0];if(!f){document.getElementById('s').textContent='请先选择文件';return;}"
"const r=await fetch('/ota',{method:'POST',body:f});document.getElementById('s').textContent=await r.text();}"
"</script></body></html>";

static esp_err_t ota_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_sendstr(req, OTA_PAGE);
    return ESP_OK;
}

static esp_err_t ota_upload_handler(httpd_req_t *req)
{
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (req->content_len == 0 || req->content_len > update_partition->size) {
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "bad size");
        return ESP_OK;
    }

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char *buf = malloc(4096);
    if (buf == NULL) {
        esp_ota_abort(ota_handle);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    while (remaining > 0) {
        int received = httpd_req_recv(req, buf, remaining < 4096 ? remaining : 4096);
        if (received <= 0) {
            ESP_LOGE(TAG, "httpd_req_recv: %d", received);
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        err = esp_ota_write(ota_handle, buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        remaining -= received;
    }
    free(buf);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OTA ok, rebooting...");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    float celsius = -100.0f;
    temperature_sensor_get_celsius(temp_sensor, &celsius);

    float aht_temp = 0.0f;
    float aht_hum = 0.0f;
    int tvoc = 0;
    int eco2 = 0;
    int aqi = 0;
    bool aht_ok = aht21_read(&aht_temp, &aht_hum) == ESP_OK;
    bool ens_ok = ens160_read_air(&tvoc, &eco2, &aqi) == ESP_OK;

    char aht_temp_str[32] = "读取失败";
    char aht_hum_str[32] = "读取失败";
    char tvoc_str[32] = "读取失败";
    char eco2_str[32] = "读取失败";
    char aqi_str[32] = "读取失败";
    if (aht_ok) {
        snprintf(aht_temp_str, sizeof(aht_temp_str), "%.2f °C", aht_temp);
        snprintf(aht_hum_str, sizeof(aht_hum_str), "%.2f %%", aht_hum);
    }
    if (ens_ok) {
        snprintf(tvoc_str, sizeof(tvoc_str), "%d ppb", tvoc);
        snprintf(eco2_str, sizeof(eco2_str), "%d ppm", eco2);
        snprintf(aqi_str, sizeof(aqi_str), "%d", aqi);
    }

    char ip[16] = "unknown";
    wifi_web_get_ip_str(ip, sizeof(ip));
    uint32_t frames = csi_capture_get_frame_count();

    char html[1024];
    snprintf(html, sizeof(html),
             "<!doctype html><meta charset='utf-8'>"
             "<h1>ESP32-C3 Sensor Monitor</h1>"
             "<p>WiFi IP: %s</p>"
             "<p>芯片温度: %.2f °C</p>"
             "<p>AHT21 检测: %s</p>"
             "<p>ENS160 检测: %s</p>"
             "<p>AHT21 温度: %s</p>"
             "<p>AHT21 湿度: %s</p>"
             "<p>ENS160 TVOC: %s</p>"
             "<p>ENS160 eCO2: %s</p>"
             "<p>ENS160 AQI: %s</p>"
             "<p>CSI frames: %lu</p>"
             "<p><a href='/csi'>打开 CSI 实时热力图</a></p>",
             ip, celsius,
             aht21_found ? "已找到" : "未找到",
             ens160_found ? "已找到" : "未找到",
             aht_temp_str,
             aht_hum_str,
             tvoc_str,
             eco2_str,
             aqi_str,
             (unsigned long)frames);

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t csi_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_sendstr(req, CSI_PAGE);
    return ESP_OK;
}

static esp_err_t api_csi_handler(httpd_req_t *req)
{
    csi_capture_frame_t frame;
    if (csi_capture_get_latest(&frame) != ESP_OK) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"error\":\"no data\"}");
        return ESP_OK;
    }

    csi_capture_motion_info_t motion;
    csi_capture_get_motion_info(&motion);

    char *buf = malloc(4096);
    if (buf == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int off = snprintf(buf, 4096,
                       "{\"seq\":%lu,\"timestamp_ms\":%lu,\"rssi\":%d,"
                       "\"activity\":%.2f,\"threshold\":%.2f,"
                       "\"moving\":%s,\"warmup\":%s,\"confidence\":%.2f,"
                       "\"amp_delta\":%.2f,\"phase_delta\":%.2f,"
                       "\"spectral_entropy\":%.2f,\"low_freq_ratio\":%.2f,"
                       "\"subcarriers\":[",
                       (unsigned long)frame.seq, (unsigned long)frame.timestamp_ms,
                       frame.rssi, motion.activity, motion.threshold,
                       motion.moving ? "true" : "false",
                       motion.warmup ? "true" : "false",
                       motion.confidence, motion.amp_delta, motion.phase_delta,
                       motion.spectral_entropy, motion.low_freq_ratio);
    if (off < 0 || off >= 4096) {
        free(buf);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    for (uint16_t i = 0; i < frame.subcarrier_count; i++) {
        off += snprintf(buf + off, 4096 - off, "%.2f%s",
                        frame.amplitudes[i],
                        (i == frame.subcarrier_count - 1) ? "" : ",");
        if (off < 0 || off >= 4096) {
            free(buf);
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
    }
    off += snprintf(buf + off, 4096 - off, "]}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    free(buf);
    return ESP_OK;
}

static esp_err_t api_sensors_handler(httpd_req_t *req)
{
    float chip_temp = -100.0f;
    temperature_sensor_get_celsius(temp_sensor, &chip_temp);

    float aht_temp = 0.0f;
    float aht_hum = 0.0f;
    int tvoc = 0;
    int eco2 = 0;
    int aqi = 0;
    bool aht_ok = aht21_read(&aht_temp, &aht_hum) == ESP_OK;
    bool ens_ok = ens160_read_air(&tvoc, &eco2, &aqi) == ESP_OK;

    char *buf = malloc(512);
    if (buf == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    snprintf(buf, 512,
             "{\"chip_temp\":%.2f,"
             "\"aht_found\":%s,\"ens_found\":%s,"
             "\"aht_ok\":%s,\"aht_temp\":%.2f,\"aht_hum\":%.2f,"
             "\"ens_ok\":%s,\"tvoc\":%d,\"eco2\":%d,\"aqi\":%d}",
             chip_temp,
             aht21_found ? "true" : "false",
             ens160_found ? "true" : "false",
             aht_ok ? "true" : "false",
             aht_temp,
             aht_hum,
             ens_ok ? "true" : "false",
             tvoc,
             eco2,
             aqi);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    free(buf);
    return ESP_OK;
}

static void init_temperature_sensor(void)
{
    temperature_sensor_config_t config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    ESP_ERROR_CHECK(temperature_sensor_install(&config, &temp_sensor));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_temperature_sensor();
    sensors_init();

    wifi_web_config_t net_config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASSWORD,
        .use_static_ip = false,
    };
    ESP_ERROR_CHECK(wifi_web_init(&net_config));
    ESP_ERROR_CHECK(csi_capture_init());

    ESP_ERROR_CHECK(wifi_web_add_uri("/", HTTP_GET, root_get_handler));
    ESP_ERROR_CHECK(wifi_web_add_uri("/csi", HTTP_GET, csi_page_handler));
    ESP_ERROR_CHECK(wifi_web_add_uri("/api/csi", HTTP_GET, api_csi_handler));
    ESP_ERROR_CHECK(wifi_web_add_uri("/api/sensors", HTTP_GET, api_sensors_handler));
    ESP_ERROR_CHECK(wifi_web_add_uri("/ota", HTTP_GET, ota_page_handler));
    ESP_ERROR_CHECK(wifi_web_add_uri("/ota", HTTP_POST, ota_upload_handler));

    ESP_ERROR_CHECK(wifi_web_start());
    ESP_ERROR_CHECK(csi_capture_start());

    char ip[16] = {0};
    for (int i = 0; i < 100; i++) {
        if (wifi_web_get_ip_str(ip, sizeof(ip)) == ESP_OK && strcmp(ip, "0.0.0.0") != 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (ip[0] == '\0' || strcmp(ip, "0.0.0.0") == 0) {
        ESP_LOGW(TAG, "Still waiting for Wi-Fi; ping will start after connection");
    }
    while (wifi_web_get_ip_str(ip, sizeof(ip)) != ESP_OK ||
           ip[0] == '\0' || strcmp(ip, "0.0.0.0") == 0) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    char target[16] = {0};
    if (CSI_PING_TARGET[0] != '\0') {
        snprintf(target, sizeof(target), "%s", CSI_PING_TARGET);
    } else {
        esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        esp_netif_ip_info_t info;
        ESP_ERROR_CHECK(esp_netif_get_ip_info(sta, &info));
        esp_ip4addr_ntoa(&info.gw, target, sizeof(target));
    }
    ESP_ERROR_CHECK(csi_capture_start_traffic(target, CSI_PING_INTERVAL_MS));
    ESP_LOGI(TAG, "System ready, IP=%s", ip);
}
