#include "mpu6050.h"
#include "config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mpu";

/* register map (only what we touch) */
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(CFG_I2C_PORT, CFG_MPU_ADDR,
                                      buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t rd(uint8_t reg, uint8_t *dst, size_t n)
{
    return i2c_master_write_read_device(CFG_I2C_PORT, CFG_MPU_ADDR,
                                        &reg, 1, dst, n, pdMS_TO_TICKS(100));
}

esp_err_t mpu6050_init(void)
{
    const i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = CFG_I2C_SDA_GPIO,
        .scl_io_num       = CFG_I2C_SCL_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,   /* module usually has its own */
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CFG_I2C_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(CFG_I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(CFG_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    /* Device reset, then wake.  The datasheet's reset takes ~100 ms. */
    if (wr(REG_PWR_MGMT_1, 0x80) != ESP_OK) {
        ESP_LOGE(TAG, "no ACK at 0x%02X -- check wiring / AD0", CFG_MPU_ADDR);
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t who = 0;
    ESP_ERROR_CHECK(rd(REG_WHO_AM_I, &who, 1));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who);   /* 0x68 genuine, 0x70/0x71 clone */

    /* Clock from the gyro X PLL: more stable than the internal 8 MHz RC,
     * which drifts with temperature and would smear the gyro scale factor. */
    ESP_ERROR_CHECK(wr(REG_PWR_MGMT_1, 0x01));
    vTaskDelay(pdMS_TO_TICKS(10));

    /* DLPF = 3 -> 44 Hz accel / 42 Hz gyro, 4.9 ms group delay.
     * Well above our 200 Hz loop's band of interest, well below the
     * structural ringing of a printed/aluminium chassis. */
    ESP_ERROR_CHECK(wr(REG_CONFIG,       0x03));
    ESP_ERROR_CHECK(wr(REG_GYRO_CONFIG,  0x08));  /* +-500 dps  */
    ESP_ERROR_CHECK(wr(REG_ACCEL_CONFIG, 0x00));  /* +-2 g      */
    ESP_ERROR_CHECK(wr(REG_SMPLRT_DIV,   0x00));  /* 1 kHz      */
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

esp_err_t mpu6050_read(mpu_sample_t *out)
{
    uint8_t b[14];
    esp_err_t err = rd(REG_ACCEL_XOUT_H, b, sizeof(b));
    if (err != ESP_OK) return err;

    /* Burst is ACCEL[6] TEMP[2] GYRO[6], big-endian signed 16-bit. */
    for (int i = 0; i < 3; i++) {
        int16_t a = (int16_t)((b[i * 2] << 8) | b[i * 2 + 1]);
        out->acc_g[i] = (float)a / CFG_ACC_LSB_PER_G;
    }
    int16_t t = (int16_t)((b[6] << 8) | b[7]);
    out->temp_c = (float)t / 340.0f + 36.53f;

    for (int i = 0; i < 3; i++) {
        int16_t g = (int16_t)((b[8 + i * 2] << 8) | b[9 + i * 2]);
        out->gyro_dps[i] = (float)g / CFG_GYRO_LSB_PER_DPS;
    }
    return ESP_OK;
}
