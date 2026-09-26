#pragma once
#include <stdbool.h>
#include "esp_err.h"

/* Raw-to-engineering-units sample from the MPU6050. */
typedef struct {
    float acc_g[3];      /* X, Y, Z in g            */
    float gyro_dps[3];   /* X, Y, Z in deg/s, RAW (bias not removed) */
    float temp_c;
} mpu_sample_t;

esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read(mpu_sample_t *out);
