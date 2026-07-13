#!/usr/bin/env python3
"""Mirror tests for firmware helper logic that the SIL harness doesn't cover.

No host C compiler on this box, so (as with test_sil_flight) these are faithful
Python transcriptions of small, safety/security-relevant C helpers. Keep in
lockstep with the referenced C when it changes.

  telemetry.c    nonce_fresh          (1a anti-replay)
  ism330dhcx.c   gyro cal stationarity (1e)
  ism330dhcx.c   imu_read freeze detect (1f)

Run: py -m unittest tests.test_fw_logic
"""
import unittest

NONCE_HIST = 32           # telemetry.c
GYRO_CAL_SAMPLES = 400
GYRO_CAL_MAX_PP_DPS = 5.0
GYRO_CAL_ACC_BAND_G = 0.15
IMU_FREEZE_N = 10


# ============================================================
# telemetry.c nonce_fresh — dedup ring, valid-slot tracking, NO floor
# ============================================================
class NonceRing:
    def __init__(self):
        self.buf = [0] * NONCE_HIST
        self.idx = 0
        self.cnt = 0        # valid entries (saturates at HIST)

    def fresh(self, n):
        for i in range(self.cnt):          # only filled slots compared
            if self.buf[i] == n:
                return False
        self.buf[self.idx] = n
        self.idx = (self.idx + 1) % NONCE_HIST
        if self.cnt < NONCE_HIST:
            self.cnt += 1
        return True


class Nonce(unittest.TestCase):
    def test_zero_nonce_accepted_first(self):
        # the old memset-to-0 ring falsely matched an incoming 0 forever
        r = NonceRing()
        self.assertTrue(r.fresh(0))
        self.assertFalse(r.fresh(0))       # exact replay now rejected

    def test_exact_replay_rejected(self):
        r = NonceRing()
        self.assertTrue(r.fresh(12345))
        self.assertFalse(r.fresh(12345))

    def test_replay_window_is_hist_deep(self):
        r = NonceRing()
        r.fresh(1000)                       # capture this "ARM"
        for k in range(NONCE_HIST - 1):     # HIST-1 newer distinct commands
            r.fresh(2000 + k)
        self.assertFalse(r.fresh(1000), "still within the %d-deep window" % NONCE_HIST)
        r.fresh(9999)                       # one more evicts nonce 1000
        self.assertTrue(r.fresh(1000), "beyond the window it is accepted again")

    def test_gs_restart_lower_nonce_accepted(self):
        # GS reseeds to a RANDOM value on restart (may be lower). A monotonic
        # floor would lock it out; the dedup ring accepts any unseen value.
        r = NonceRing()
        for n in range(5_000_000, 5_000_050):
            r.fresh(n)
        self.assertTrue(r.fresh(42), "restarted-GS lower nonce must be accepted")


# ============================================================
# ism330dhcx.c gyro cal stationarity (1e)
# ============================================================
def gyro_cal_window(samples):
    """samples: list of (gx,gy,gz,amag). Returns 'done' if steady over the
    full window, else 'restart'. Mirrors the imu_read cal block."""
    csum = [0.0, 0.0, 0.0]
    cmin = [1e9, 1e9, 1e9]
    cmax = [-1e9, -1e9, -1e9]
    moved = False
    for (gx, gy, gz, amag) in samples:
        g = (gx, gy, gz)
        if abs(amag - 1.0) > GYRO_CAL_ACC_BAND_G:
            moved = True
        for i in range(3):
            csum[i] += g[i]
            cmin[i] = min(cmin[i], g[i])
            cmax[i] = max(cmax[i], g[i])
    steady = not moved
    for i in range(3):
        if (cmax[i] - cmin[i]) > GYRO_CAL_MAX_PP_DPS:
            steady = False
    return "done" if steady else "restart"


class GyroCal(unittest.TestCase):
    def test_still_completes(self):
        s = [(0.1, -0.1, 0.05, 1.0)] * GYRO_CAL_SAMPLES   # tiny steady noise
        self.assertEqual(gyro_cal_window(s), "done")

    def test_jitter_restarts(self):
        s = [(0.0, 0.0, (10.0 if k % 2 else -10.0), 1.0)   # 20 dps p-p
             for k in range(GYRO_CAL_SAMPLES)]
        self.assertEqual(gyro_cal_window(s), "restart")

    def test_handling_tilt_restarts(self):
        s = [(0.1, 0.1, 0.1, 1.4)] * GYRO_CAL_SAMPLES      # accel off 1 g
        self.assertEqual(gyro_cal_window(s), "restart")


# ============================================================
# ism330dhcx.c imu_read freeze detect (1f)
# ============================================================
def freeze_index(bursts):
    """Return the (1-based) read count at which a freeze is declared, or -1.
    Mirrors the memcmp-vs-previous-burst logic."""
    prev = None
    same = 0
    for i, b in enumerate(bursts):
        if prev is not None and b == prev:
            same += 1
            if same >= IMU_FREEZE_N:
                return i + 1
        else:
            same = 0
        prev = b
    return -1


class ImuFreeze(unittest.TestCase):
    def test_identical_burst_flags_freeze(self):
        frozen = [(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)] * (IMU_FREEZE_N + 5)
        # first read seeds prev; the Nth *repeat* trips it
        self.assertEqual(freeze_index(frozen), IMU_FREEZE_N + 1)

    def test_live_noise_never_freezes(self):
        live = [tuple((v + k) % 256 for v in range(12)) for k in range(200)]
        self.assertEqual(freeze_index(live), -1)

    def test_brief_repeat_ok(self):
        b = (0,) * 12
        bursts = [b] * (IMU_FREEZE_N - 1) + [(9,) * 12] * 5   # repeat then move
        self.assertEqual(freeze_index(bursts), -1)


# ============================================================
# state_machine.c fsm_params_apply — apogee_timeout clamp
# ============================================================
MAX_BURN_MS = 6000        # app_config.h
APOGEE_TIMEOUT_MAX_MS = 120000


def apogee_timeout_apply(ms):
    """Mirror fsm_params_apply's clamp: accept ms and return it, else None
    (keep the compile-time default). Floor is MAX_BURN_MS — the value also
    drives the fault-independent backup deploy, which fires regardless of
    state, so anything <= burnout could deploy under thrust."""
    if MAX_BURN_MS <= ms <= APOGEE_TIMEOUT_MAX_MS:
        return ms
    return None


class ApogeeTimeoutClamp(unittest.TestCase):
    def test_rejects_during_boost(self):
        # below MAX_BURN_MS: the backup deploy could fire during powered ascent
        self.assertIsNone(apogee_timeout_apply(3000))   # old MIN_DROGUE_DELAY floor
        self.assertIsNone(apogee_timeout_apply(5999))

    def test_accepts_past_burnout(self):
        self.assertEqual(apogee_timeout_apply(6000), 6000)
        self.assertEqual(apogee_timeout_apply(30000), 30000)

    def test_rejects_absurd(self):
        self.assertIsNone(apogee_timeout_apply(0))
        self.assertIsNone(apogee_timeout_apply(200000))


if __name__ == "__main__":
    unittest.main(verbosity=2)
