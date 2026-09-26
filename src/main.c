/* ------------------------------------------------------------------------
 * wheelo -- tilt angle for a single-axis balancing robot, ESP32-C3.
 *
 * BOOT button (GPIO9):
 *   short press        -> set current attitude as 0 deg (hold it upright)
 *   hold >= 2 s        -> gyro bias calibration (keep it still)
 *   hold at power-up   -> axis identification mode
 * ---------------------------------------------------------------------- */
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "config.h"
#include "mpu6050.h"
#include "tilt.h"
#include "calib.h"

static const char *TAG = "wheelo";
static tilt_t g_tilt;

/* ---------------------------------------------------------------- button */
static void button_init(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << CFG_BTN_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);
}
static inline bool button_down(void) { return gpio_get_level(CFG_BTN_GPIO) == 0; }

/* Returns 0 = nothing, 1 = short press, 2 = long press. Non-blocking-ish:
 * only spins while the button is actually held. */
static int button_poll(void)
{
    if (!button_down()) return 0;
    int held = 0;
    while (button_down() && held < 5000) { vTaskDelay(pdMS_TO_TICKS(10)); held += 10; }
    if (held < 40) return 0;                        /* debounce */
    return (held >= CFG_BTN_LONG_MS) ? 2 : 1;
}

/* ------------------------------------------------------------- axis ident */
static void axis_identify(void)
{
    printf("\n=== AXIS ID ===\n"
           "Tip the robot slowly from upright to flat and back.\n"
           "  accel axis that barely changes        -> CFG_AX_ROT\n"
           "  accel axis near +-1g when UPRIGHT     -> CFG_AX_UP   (want +1)\n"
           "  accel axis near +-1g when FLAT        -> CFG_AX_FWD  (want +1)\n"
           "  gyro axis that spikes while tipping   -> CFG_AX_ROT (same index)\n"
           "A -1g reading means flip the matching *_SIGN macro.\n"
           "Press BOOT to exit.\n\n");
    vTaskDelay(pdMS_TO_TICKS(600));

    mpu_sample_t s;
    while (!button_down()) {
        if (mpu6050_read(&s) == ESP_OK) {
            printf("acc X%+6.2f Y%+6.2f Z%+6.2f g  |  gyr X%+7.1f Y%+7.1f Z%+7.1f dps\n",
                   s.acc_g[0], s.acc_g[1], s.acc_g[2],
                   s.gyro_dps[0], s.gyro_dps[1], s.gyro_dps[2]);
        }
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    while (button_down()) vTaskDelay(pdMS_TO_TICKS(20));
    printf("=== exit axis id ===\n\n");
}

/* ------------------------------------------------------------------- main */
void app_main(void)
{
    ESP_ERROR_CHECK(calib_init());
    button_init();

    if (mpu6050_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 not responding -- halting");
        return;
    }

    tilt_reset(&g_tilt);
    calib_load(&g_tilt);

    if (button_down()) axis_identify();   /* held at boot */

    ESP_LOGI(TAG, "running at %d Hz, r = %.3f m", CFG_LOOP_HZ, CFG_R_ARM);
    ESP_LOGI(TAG, "BOOT: tap = set zero, hold 2s = gyro cal");

    TickType_t next = xTaskGetTickCount();
    int64_t t_prev = esp_timer_get_time();
    int64_t t_print = t_prev;

    for (;;) {
        vTaskDelayUntil(&next, pdMS_TO_TICKS(1000 / CFG_LOOP_HZ));

        int btn = button_poll();
        if (btn == 1) { calib_zero(&g_tilt);       t_prev = esp_timer_get_time(); continue; }
        if (btn == 2) { calib_gyro_bias(&g_tilt);  t_prev = esp_timer_get_time(); continue; }

        mpu_sample_t s;
        if (mpu6050_read(&s) != ESP_OK) continue;

        /* Real elapsed time, not the nominal 5 ms -- a missed deadline or a
         * retried I2C transaction would otherwise corrupt the integration. */
        int64_t now = esp_timer_get_time();
        float dt = (float)(now - t_prev) * 1e-6f;
        t_prev = now;
        if (dt <= 0.0f || dt > 0.1f) continue;      /* stall: skip, don't lie */

        tilt_update(&g_tilt, s.acc_g, s.gyro_dps, dt);

        /* ---- this is what a balance controller consumes ----------------
         * error = 0 - angle          (setpoint is upright)
         * d-term = -omega            (a real measurement, already clean;
         *                             never differentiate `angle` for it)
         * if (fabsf(angle) > 45 deg) cut the motors -- it has fallen.
         */

        if (now - t_print > 50000) {                /* 20 Hz telemetry */
            t_print = now;
            printf("angle %+7.2f deg | rate %+7.1f dps | accel-only %+7.2f deg\n",
                   g_tilt.angle     * CFG_RAD2DEG,
                   g_tilt.omega     * CFG_RAD2DEG,
                   g_tilt.acc_angle * CFG_RAD2DEG);
        }
    }
}
