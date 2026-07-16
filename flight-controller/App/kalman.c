/* ============================================================
 * kalman.c: 1-D altitude/velocity Kalman filter (baro-driven)
 *
 * State x = [alt (m), vel_up (m/s)].  F = [[1 dt],[0 1]].
 * Accel (already gravity-compensated, up-positive) is a control input
 * used ONLY when accel_valid (ISM330DHCX saturates at ±16 g in boost).
 * Q = white-accel model scaled by KAL_Q_ACCEL; baro update is a scalar
 * measurement of alt with R = KAL_R_BARO, Joseph-form covariance update.
 * ============================================================ */
#include "app.h"
#include <math.h>

#define P_INIT_ALT   1.0e4f    /* m^2, large: state unknown at reset */
#define P_INIT_VEL   1.0e2f    /* (m/s)^2 */
#define DT_MAX       0.5f      /* clamp after loop stalls */

void kal_reset(kalman_t *k)
{
    k->alt_m  = 0.0f;
    k->vel_ms = 0.0f;
    k->P[0][0] = P_INIT_ALT; k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;       k->P[1][1] = P_INIT_VEL;
    k->init = false;          /* first baro update seeds alt */
    k->gate_rejects = 0;
}

void kal_predict(kalman_t *k, float accel_up_ms2, float dt, bool accel_valid)
{
    if (!k->init || dt <= 0.0f)
        return;
    if (dt > DT_MAX)
        dt = DT_MAX;

    /* state propagation */
    if (accel_valid) {
        k->alt_m  += k->vel_ms * dt + 0.5f * accel_up_ms2 * dt * dt;
        k->vel_ms += accel_up_ms2 * dt;
    } else {
        k->alt_m  += k->vel_ms * dt;   /* coast on velocity only */
    }

    /* P = F P F' + Q,  Q = q * [[dt^4/4, dt^3/2],[dt^3/2, dt^2]] */
    float p00 = k->P[0][0], p01 = k->P[0][1];
    float p10 = k->P[1][0], p11 = k->P[1][1];

    float n00 = p00 + dt * (p10 + p01) + dt * dt * p11;
    float n01 = p01 + dt * p11;
    float n10 = p10 + dt * p11;
    float n11 = p11;

    const float q   = KAL_Q_ACCEL;
    const float dt2 = dt * dt;
    n00 += q * 0.25f * dt2 * dt2;
    n01 += q * 0.5f  * dt2 * dt;
    n10 += q * 0.5f  * dt2 * dt;
    n11 += q * dt2;

    /* enforce symmetry */
    float off = 0.5f * (n01 + n10);
    k->P[0][0] = n00; k->P[0][1] = off;
    k->P[1][0] = off; k->P[1][1] = n11;
}

void kal_update_baro(kalman_t *k, float alt_m)
{
    if (!isfinite(alt_m))      /* never let NaN/inf into the state */
        return;

    if (!k->init) {            /* seed from first measurement */
        k->alt_m  = alt_m;
        k->vel_ms = 0.0f;
        k->P[0][0] = KAL_R_BARO; k->P[0][1] = 0.0f;
        k->P[1][0] = 0.0f;       k->P[1][1] = P_INIT_VEL;
        k->init = true;
        k->gate_rejects = 0;
        return;
    }

    float p00 = k->P[0][0], p01 = k->P[0][1];
    float p10 = k->P[1][0], p11 = k->P[1][1];

    /* H = [1 0]; S = p00 + R; K = P H' / S */
    const float r = KAL_R_BARO;
    float s  = p00 + r;
    if (s <= 0.0f)             /* degenerate: reseed covariance */
        s = r;
    float k0 = p00 / s;
    float k1 = p10 / s;

    float y = alt_m - k->alt_m;

    /* Innovation gate: a wild outlier (glitched conversion, vent transient)
     * must not slam alt/vel, since deploy logic keys off them. P keeps growing
     * via predict while we reject, so the gate widens on its own; after
     * KAL_GATE_MAX_REJECT consecutive rejections accept unconditionally so
     * a genuine altitude step can never be locked out. */
    if ((y * y) > (KAL_GATE_SIGMA * KAL_GATE_SIGMA) * s &&
        k->gate_rejects < (uint16_t)KAL_GATE_MAX_REJECT) {
        k->gate_rejects++;
        return;
    }
    k->gate_rejects = 0;
    k->alt_m  += k0 * y;
    k->vel_ms += k1 * y;

    /* Joseph form: P = (I-KH) P (I-KH)' + K R K'  (keeps P PSD) */
    float a = 1.0f - k0;       /* (I-KH) = [[a,0],[-k1,1]] */
    float n00 = a * a * p00;
    float n01 = a * (p01 - k1 * p00);
    float n10 = a * (p10 - k1 * p00);
    float n11 = p11 - k1 * (p01 + p10) + k1 * k1 * p00;

    n00 += r * k0 * k0;
    n01 += r * k0 * k1;
    n10 += r * k0 * k1;
    n11 += r * k1 * k1;

    float off = 0.5f * (n01 + n10);
    k->P[0][0] = n00; k->P[0][1] = off;
    k->P[1][0] = off; k->P[1][1] = n11;
}
