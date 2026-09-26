#pragma once
/* ------------------------------------------------------------------------
 * Single-axis tilt estimator.  No hardware here on purpose -- this file is
 * pure math over floats and can be compiled and unit-tested on a PC.
 *
 * Output convention:
 *      0 deg   = robot standing upright
 *    +90 deg   = lying flat one way
 *    -90 deg   = lying flat the other way
 *   continuous through +-180 deg, no gimbal/quadrant discontinuity.
 * ---------------------------------------------------------------------- */
#include <stdbool.h>

typedef struct {
    /* --- calibration (persisted) --- */
    float gyro_bias_dps;   /* zero-rate offset of the rotation-axis gyro   */
    float zero_offset;     /* rad: raw accel angle seen when upright.
                              This is where the ~15 deg mount tilt lives.  */
    /* --- live state --- */
    float angle;           /* rad, fused estimate. THE output.             */
    float omega;           /* rad/s, bias-corrected rate about the axle    */
    float alpha;           /* rad/s^2, low-passed d(omega)/dt              */
    float acc_angle;       /* rad, accel-only angle (diagnostics)          */
    bool  seeded;
} tilt_t;

/* Wrap any angle into (-pi, +pi]. */
float tilt_wrap_pi(float rad);

/* Accelerometer-only tilt, with the sensor's own rotation about the pivot
 * removed.  Pass omega/alpha = 0 for a static measurement.
 * Returns the RAW angle -- zero_offset is NOT subtracted. */
float tilt_accel_angle(const float acc_g[3], float omega, float alpha);

/* Reset live state; keeps calibration. */
void tilt_reset(tilt_t *t);

/* One control-loop step.  dt in seconds. */
void tilt_update(tilt_t *t, const float acc_g[3], const float gyro_dps[3], float dt);
