#include "calib.h"
#include "config.h"
#include "mpu6050.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "cal";
#define NS "wheelo"

esp_err_t calib_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

void calib_load(tilt_t *t)
{
    nvs_handle_t h;
    t->gyro_bias_dps = 0.0f;
    t->zero_offset   = 0.0f;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGW(TAG, "no stored calibration -- run both calibrations");
        return;
    }
    size_t n = sizeof(float);
    nvs_get_blob(h, "gbias", &t->gyro_bias_dps, &n);
    n = sizeof(float);
    nvs_get_blob(h, "zoff",  &t->zero_offset,   &n);
    nvs_close(h);
    ESP_LOGI(TAG, "loaded: gyro bias %.3f dps, zero %.2f deg",
             t->gyro_bias_dps, t->zero_offset * CFG_RAD2DEG);
}

void calib_save(const tilt_t *t)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "gbias", &t->gyro_bias_dps, sizeof(float));
    nvs_set_blob(h, "zoff",  &t->zero_offset,   sizeof(float));
    nvs_commit(h);
    nvs_close(h);
}

void calib_gyro_bias(tilt_t *t)
{
    ESP_LOGI(TAG, "gyro bias: hold PERFECTLY still...");
    vTaskDelay(pdMS_TO_TICKS(500));

    const int N = 2000;                 /* 2 s at 1 kHz */
    double sum = 0.0;
    int got = 0;
    mpu_sample_t s;
    for (int i = 0; i < N; i++) {
        if (mpu6050_read(&s) == ESP_OK) { sum += s.gyro_dps[CFG_AX_ROT]; got++; }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (got < N / 2) { ESP_LOGE(TAG, "read failures, aborted"); return; }

    t->gyro_bias_dps = (float)(sum / got);
    calib_save(t);
    tilt_reset(t);
    ESP_LOGI(TAG, "gyro bias = %.4f dps  (drift if left: %.1f deg/min)",
             t->gyro_bias_dps, t->gyro_bias_dps * 60.0f);
}

void calib_zero(tilt_t *t)
{
    ESP_LOGI(TAG, "zero: hold the robot UPRIGHT and still...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Average as a UNIT VECTOR, then atan2 the mean.  Averaging angles
     * numerically is wrong near the wrap point: mean(+179, -179) = 0. */
    const int N = 1000;
    double cx = 0.0, cy = 0.0;
    int got = 0;
    mpu_sample_t s;
    for (int i = 0; i < N; i++) {
        if (mpu6050_read(&s) == ESP_OK) {
            float th = tilt_accel_angle(s.acc_g, 0.0f, 0.0f);  /* static */
            cx += cosf(th); cy += sinf(th); got++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (got < N / 2) { ESP_LOGE(TAG, "read failures, aborted"); return; }

    t->zero_offset = atan2f((float)cy, (float)cx);
    calib_save(t);
    tilt_reset(t);
    ESP_LOGI(TAG, "zero offset = %.2f deg  (expect ~15 for this mount)",
             t->zero_offset * CFG_RAD2DEG);
}
