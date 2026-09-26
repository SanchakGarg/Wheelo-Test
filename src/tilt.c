#include "tilt.h"
#include "config.h"
#include <math.h>

float tilt_wrap_pi(float rad)
{
    /* remainderf() already returns a value in [-pi, pi] for a 2*pi divisor
     * and is branch-free, unlike a while-loop that stalls on a NaN. */
    return remainderf(rad, 2.0f * (float)M_PI);
}

/* ------------------------------------------------------------------------
 * The accelerometer does not measure gravity; it measures SPECIFIC FORCE,
 * which is (its own acceleration - gravity) resolved into the body frame.
 * Standing still, that is exactly gravity, and the tilt falls straight out
 * of the ratio of two axes.
 *
 * But our sensor is bolted 245 mm out from the rotation axis, so whenever
 * the robot rotates the sensor is being swung on the end of a stick and
 * feels two extra accelerations:
 *
 *        p_ddot = alpha x r  +  omega x (omega x r)
 *               = (r*alpha) * t_hat  -  (r*omega^2) * u_hat
 *
 *   u_hat : unit vector pivot -> sensor          -> our AX_UP  axis
 *   t_hat : perpendicular to it, in-plane        -> our AX_FWD axis
 *
 * so the raw axes read
 *        up  = g*cos(theta) - r*omega^2
 *        fwd = g*sin(theta) + r*alpha
 *
 * and we add/subtract those back to recover the gravity-only components.
 * At r = 0.245 m and omega = 2 rad/s the centripetal term alone is
 * 0.98 m/s^2, about 0.1 g -- roughly 5.7 deg of error at upright, exactly
 * when the robot is swinging hardest to catch itself.
 * ---------------------------------------------------------------------- */
float tilt_accel_angle(const float acc_g[3], float omega, float alpha)
{
    float up  = CFG_UP_SIGN  * acc_g[CFG_AX_UP]  * CFG_G_MS2;
    float fwd = CFG_FWD_SIGN * acc_g[CFG_AX_FWD] * CFG_G_MS2;

    up  += CFG_R_ARM * omega * omega;   /* undo centripetal (points inward) */
    fwd -= CFG_R_ARM * alpha;           /* undo tangential                  */

    /* atan2, not asin: it uses BOTH axes, so it stays valid and linear all
     * the way round -- upright, flat at +-90, and past it.  asin(fwd/g)
     * would fold over at 90 deg and blow up when |fwd| > g. */
    return atan2f(fwd, up);
}

void tilt_reset(tilt_t *t)
{
    t->angle = t->omega = t->alpha = t->acc_angle = 0.0f;
    t->seeded = false;
}

void tilt_update(tilt_t *t, const float acc_g[3], const float gyro_dps[3], float dt)
{
    /* ---- 1. rate about the rotation axis, bias removed, in rad/s ------- */
    float w = CFG_GYRO_SIGN *
              (gyro_dps[CFG_AX_ROT] - t->gyro_bias_dps) * CFG_DEG2RAD;

    /* ---- 2. angular acceleration for the tangential term ---------------
     * Differentiating a noisy signal amplifies noise, so low-pass it.
     * This feeds only the small r*alpha correction, never the angle path. */
    float alpha_raw = (w - t->omega) / dt;
    t->alpha += CFG_ALPHA_LP * (alpha_raw - t->alpha);
    t->omega  = w;

    /* ---- 3. accelerometer angle, referenced to "upright = 0" -----------
     * Subtracting zero_offset here is what cancels the 15 deg mount tilt
     * (and every other mounting error) in one shot. */
    float acc_ang = tilt_wrap_pi(
        tilt_accel_angle(acc_g, t->omega, t->alpha) - t->zero_offset);
    t->acc_angle = acc_ang;

    /* ---- 4. first sample: trust the accelerometer, don't start at a lie */
    if (!t->seeded) { t->angle = acc_ang; t->seeded = true; return; }

    /* ---- 5. complementary filter ---------------------------------------
     * Gyro: correct short-term, drifts long-term (bias + integration).
     * Accel: correct long-term, garbage short-term (the robot's own
     *        translation shows up as fake tilt).
     * So integrate the gyro and nudge it toward the accelerometer with a
     * time constant TAU.  k = TAU/(TAU+dt) ~= 0.9934 at 200 Hz, i.e. a
     * ~0.75 s first-order crossover.
     *
     * The correction is applied to the ERROR, wrapped -- if we blended the
     * two angles directly, a robot sitting near +-180 deg with one estimate
     * at +179 and the other at -179 would average to 0 and snap upright. */
    float pred = t->angle + t->omega * dt;
    float k    = CFG_TAU / (CFG_TAU + dt);
    t->angle   = tilt_wrap_pi(pred + (1.0f - k) * tilt_wrap_pi(acc_ang - pred));
}
