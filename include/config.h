#pragma once
/* ------------------------------------------------------------------------
 * wheelo -- physical + tuning constants
 *
 * Geometry: the MPU6050 sits on the circumference of a circle of radius
 * R_ARM about the robot's rotation axis (the wheel axle).  Its board is
 * tilted ~15 deg out of the plane that contains the axle and the sensor,
 * i.e. the long edge slopes down.  That 15 deg is NOT hard-coded anywhere:
 * it is measured once at calibration and stored in NVS as ZERO_OFFSET.
 * ---------------------------------------------------------------------- */

/* ---- I2C ------------------------------------------------------------- */
/* GPIO2 / GPIO8 / GPIO9 are strapping pins on the ESP32-C3.  Keep I2C off
 * them; GPIO9 is left free below so the BOOT button can drive calibration. */
#define CFG_I2C_PORT        0
#define CFG_I2C_SDA_GPIO    4
#define CFG_I2C_SCL_GPIO    5
#define CFG_I2C_HZ          400000
#define CFG_MPU_ADDR        0x68      /* 0x69 if AD0 is tied high          */

/* ---- calibration button (BOOT on most C3 devkits) --------------------- */
#define CFG_BTN_GPIO        9         /* active low, internal pull-up      */
#define CFG_BTN_LONG_MS     2000      /* >= this = gyro-bias cal           */

/* ---- geometry -------------------------------------------------------- */
#define CFG_R_ARM           0.245f    /* pivot -> sensor, metres           */

/* ---- axis map -- set these with the axis-ID mode, then rebuild --------
 * Index 0 = X, 1 = Y, 2 = Z (same order for accel and gyro).
 *   AX_UP  : body axis pointing radially OUT from the pivot to the sensor.
 *            Reads ~+1 g when the robot stands upright.
 *   AX_FWD : body axis tangential to the circle -- the direction the robot
 *            falls in.  Reads ~+1 g when the robot lies flat "forward".
 *   AX_ROT : body axis parallel to the wheel axle.  Barely changes when you
 *            tip the robot; this is the gyro channel we integrate.
 * The SIGN macros flip an axis that is physically correct but points the
 * wrong way because of how the board is glued on.                        */
#define CFG_AX_UP           2
#define CFG_AX_FWD          0
#define CFG_AX_ROT          1
#define CFG_UP_SIGN         (+1.0f)
#define CFG_FWD_SIGN        (+1.0f)
#define CFG_GYRO_SIGN       (+1.0f)

/* ---- filter ---------------------------------------------------------- */
#define CFG_LOOP_HZ         200
#define CFG_LOOP_DT         (1.0f / (float)CFG_LOOP_HZ)
#define CFG_TAU             0.75f     /* complementary crossover, seconds  */
#define CFG_ALPHA_LP        0.15f     /* LPF coeff on angular acceleration */

/* ---- sensor full-scale ----------------------------------------------- */
#define CFG_ACC_LSB_PER_G     16384.0f  /* +-2 g   */
#define CFG_GYRO_LSB_PER_DPS  65.5f     /* +-500 dps */

/* ---- constants ------------------------------------------------------- */
#define CFG_G_MS2           9.80665f
#define CFG_DEG2RAD         0.01745329252f
#define CFG_RAD2DEG         57.2957795131f
