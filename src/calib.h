#pragma once
#include "tilt.h"
#include "esp_err.h"

esp_err_t calib_init(void);              /* mounts NVS                     */
void      calib_load(tilt_t *t);         /* fills gyro_bias_dps, zero_offset */
void      calib_save(const tilt_t *t);

/* Robot must be motionless.  Averages the rotation-axis gyro. */
void calib_gyro_bias(tilt_t *t);

/* Robot must be held UPRIGHT and still.  Records the raw accel angle as
 * the new definition of 0 deg -- this is where the 15 deg mount goes. */
void calib_zero(tilt_t *t);
