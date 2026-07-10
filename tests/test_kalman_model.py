"""Model tests for flight-controller/App/kalman.c via a faithful Python port.

The port below mirrors kalman.c as committed, line for line:
  - kal_reset / kal_predict / kal_update_baro
  - isfinite guard on the baro measurement
  - innovation gate: reject when y^2 > KAL_GATE_SIGMA^2 * (p00 + R) while
    init'ed, with a KAL_GATE_MAX_REJECT consecutive-reject cap after which
    the update is accepted unconditionally (a real altitude step can never
    be locked out)
  - Joseph-form covariance update, symmetry enforcement

Constants from App/app_config.h. The C uses 32-bit floats and this port uses
Python doubles; the assertions here are far looser than that difference.

If kalman.c changes, update this port in lockstep.
"""

import math
import random
import unittest

# ---- App/app_config.h ----
KAL_Q_ACCEL = 4.0          # process noise (m/s^2)^2
KAL_R_BARO = 1.5           # baro meas noise m^2
KAL_GATE_SIGMA = 5.0       # innovation gate: reject y^2 > N^2*S
KAL_GATE_MAX_REJECT = 25   # ~0.5 s at 50 Hz, then accept (real step)

# ---- kalman.c file-local constants ----
P_INIT_ALT = 1.0e4         # m^2 — large: state unknown at reset
P_INIT_VEL = 1.0e2         # (m/s)^2
DT_MAX = 0.5               # clamp after loop stalls


class Kalman(object):
    """Python port of kalman_t + kalman.c."""

    def __init__(self):
        self.reset()

    def reset(self):                       # kal_reset
        self.alt_m = 0.0
        self.vel_ms = 0.0
        self.P = [[P_INIT_ALT, 0.0], [0.0, P_INIT_VEL]]
        self.init = False                  # first baro update seeds alt
        self.gate_rejects = 0

    def predict(self, accel_up_ms2, dt, accel_valid):   # kal_predict
        if not self.init or dt <= 0.0:
            return
        if dt > DT_MAX:
            dt = DT_MAX

        # state propagation
        if accel_valid:
            self.alt_m += self.vel_ms * dt + 0.5 * accel_up_ms2 * dt * dt
            self.vel_ms += accel_up_ms2 * dt
        else:
            self.alt_m += self.vel_ms * dt     # coast on velocity only

        # P = F P F' + Q,  Q = q * [[dt^4/4, dt^3/2],[dt^3/2, dt^2]]
        p00, p01 = self.P[0]
        p10, p11 = self.P[1]

        n00 = p00 + dt * (p10 + p01) + dt * dt * p11
        n01 = p01 + dt * p11
        n10 = p10 + dt * p11
        n11 = p11

        q = KAL_Q_ACCEL
        dt2 = dt * dt
        n00 += q * 0.25 * dt2 * dt2
        n01 += q * 0.5 * dt2 * dt
        n10 += q * 0.5 * dt2 * dt
        n11 += q * dt2

        off = 0.5 * (n01 + n10)            # enforce symmetry
        self.P = [[n00, off], [off, n11]]

    def update_baro(self, alt_m):          # kal_update_baro
        if not math.isfinite(alt_m):       # never let NaN/inf into the state
            return

        if not self.init:                  # seed from first measurement
            self.alt_m = alt_m
            self.vel_ms = 0.0
            self.P = [[KAL_R_BARO, 0.0], [0.0, P_INIT_VEL]]
            self.init = True
            self.gate_rejects = 0
            return

        p00, p01 = self.P[0]
        p10, p11 = self.P[1]

        # H = [1 0]; S = p00 + R; K = P H' / S
        r = KAL_R_BARO
        s = p00 + r
        if s <= 0.0:                       # degenerate — reseed covariance
            s = r
        k0 = p00 / s
        k1 = p10 / s

        y = alt_m - self.alt_m

        # Innovation gate (see kalman.c comment): P keeps growing via
        # predict while rejecting; after KAL_GATE_MAX_REJECT consecutive
        # rejections accept unconditionally.
        if (y * y) > (KAL_GATE_SIGMA * KAL_GATE_SIGMA) * s \
                and self.gate_rejects < KAL_GATE_MAX_REJECT:
            self.gate_rejects += 1
            return
        self.gate_rejects = 0
        self.alt_m += k0 * y
        self.vel_ms += k1 * y

        # Joseph form: P = (I-KH) P (I-KH)' + K R K'
        a = 1.0 - k0                       # (I-KH) = [[a,0],[-k1,1]]
        n00 = a * a * p00
        n01 = a * (p01 - k1 * p00)
        n10 = a * (p10 - k1 * p00)
        n11 = p11 - k1 * (p01 + p10) + k1 * k1 * p00

        n00 += r * k0 * k0
        n01 += r * k0 * k1
        n10 += r * k0 * k1
        n11 += r * k1 * k1

        off = 0.5 * (n01 + n10)
        self.P = [[n00, off], [off, n11]]


DT = 1.0 / 50.0  # 50 Hz baro update rate


def run_converged(alt_true=100.0, n=200, seed=99, noise_sigma=1.0):
    """Return a filter converged on noisy baro at alt_true."""
    k = Kalman()
    rng = random.Random(seed)
    for _ in range(n):
        k.predict(0.0, DT, False)
        k.update_baro(alt_true + rng.gauss(0.0, noise_sigma))
    return k


class TestConvergence(unittest.TestCase):
    def test_constant_altitude_noisy_baro(self):
        # (a) 2 s of 50 Hz noisy baro at 100 m -> tight alt/vel estimate.
        k = Kalman()
        rng = random.Random(1234)
        for _ in range(100):               # 2 s at 50 Hz
            k.predict(0.0, DT, False)
            k.update_baro(100.0 + rng.gauss(0.0, 1.0))
        self.assertTrue(k.init)
        self.assertLess(abs(k.alt_m - 100.0), 0.5)
        self.assertLess(abs(k.vel_ms), 0.3)
        self.assertEqual(k.gate_rejects, 0)

    def test_first_update_seeds(self):
        k = Kalman()
        k.update_baro(250.0)
        self.assertTrue(k.init)
        self.assertEqual(k.alt_m, 250.0)
        self.assertEqual(k.vel_ms, 0.0)
        self.assertEqual(k.P[0][0], KAL_R_BARO)

    def test_predict_before_init_is_noop(self):
        k = Kalman()
        k.predict(9.81, DT, True)
        self.assertEqual(k.alt_m, 0.0)
        self.assertEqual(k.vel_ms, 0.0)


class TestSpikeRejection(unittest.TestCase):
    def test_single_200m_spike_bounded(self):
        # (b) converged filter; one +200 m outlier must barely move state.
        k = run_converged()
        alt0, vel0 = k.alt_m, k.vel_ms
        k.predict(0.0, DT, False)
        k.update_baro(k.alt_m + 200.0)
        self.assertLess(abs(k.alt_m - alt0), 1.0)
        self.assertLess(abs(k.vel_ms - vel0), 1.0)
        self.assertEqual(k.gate_rejects, 1)

    def test_good_sample_resets_reject_count(self):
        k = run_converged()
        k.predict(0.0, DT, False)
        k.update_baro(k.alt_m + 200.0)     # rejected
        self.assertEqual(k.gate_rejects, 1)
        k.predict(0.0, DT, False)
        k.update_baro(100.0)               # plausible -> accepted
        self.assertEqual(k.gate_rejects, 0)


class TestSustainedStep(unittest.TestCase):
    def test_step_accepted_within_gate_cap_and_converges(self):
        # (c) +50 m step held: gate must give up within KAL_GATE_MAX_REJECT+1
        # updates, then the filter converges to the new altitude.
        k = run_converged(alt_true=100.0)
        first_accept = None
        for n in range(1, 501):            # 10 s at 50 Hz
            k.predict(0.0, DT, False)
            k.update_baro(150.0)
            if first_accept is None and k.gate_rejects == 0:
                first_accept = n
        self.assertIsNotNone(first_accept)
        self.assertLessEqual(first_accept, KAL_GATE_MAX_REJECT + 1)  # <= 26
        self.assertLess(abs(k.alt_m - 150.0), 0.5)
        self.assertLess(abs(k.vel_ms), 0.5)

    def test_never_locked_out(self):
        # Even from a tiny covariance, the reject cap guarantees acceptance.
        k = run_converged()
        rejects_seen = 0
        for _ in range(KAL_GATE_MAX_REJECT):
            k.predict(0.0, DT, False)
            k.update_baro(500.0)
            rejects_seen = max(rejects_seen, k.gate_rejects)
        self.assertEqual(rejects_seen, KAL_GATE_MAX_REJECT)
        alt_before = k.alt_m
        k.predict(0.0, DT, False)
        k.update_baro(500.0)               # 26th: accepted unconditionally
        self.assertEqual(k.gate_rejects, 0)
        self.assertGreater(k.alt_m, alt_before)


class TestNanGuard(unittest.TestCase):
    def _assert_noop(self, k, value):
        alt0, vel0 = k.alt_m, k.vel_ms
        p_before = [row[:] for row in k.P]
        rej0 = k.gate_rejects
        k.update_baro(value)
        self.assertEqual(k.alt_m, alt0)
        self.assertEqual(k.vel_ms, vel0)
        self.assertEqual(k.P, p_before)
        self.assertEqual(k.gate_rejects, rej0)

    def test_nan_is_noop(self):
        # (d) NaN input changes nothing — state, P, reject counter.
        self._assert_noop(run_converged(), float("nan"))

    def test_inf_is_noop(self):
        self._assert_noop(run_converged(), float("inf"))
        self._assert_noop(run_converged(), float("-inf"))

    def test_nan_before_init_does_not_seed(self):
        k = Kalman()
        k.update_baro(float("nan"))
        self.assertFalse(k.init)


if __name__ == "__main__":
    unittest.main()
