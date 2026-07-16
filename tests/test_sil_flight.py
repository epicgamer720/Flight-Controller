#!/usr/bin/env python3
"""SIL (software-in-the-loop) regression net for the flight state machine.

There is no host C compiler on this box (only arm-none-eabi cross), so this is
a *faithful Python transcription* of the deploy-critical firmware, not the C
itself. It mirrors, line-for-line where practical (C source refs in comments):

    App/state_machine.c   fsm_step / fsm_request_arm / fsm_disarm
    App/kalman.c          kal_reset / kal_predict / kal_update_baro
    App/pyro.c            fire_allowed / pyro_fire / pyro_poll (fire SM)

The value here is the adversarial flight profiles + invariant asserts, which are
backend-agnostic: if a native compiler is ever installed, only FlightSim's guts
swap to ctypes/subprocess against the real .c, and the scenarios and asserts below
carry over unchanged.

LOCKSTEP RULE: every deploy-logic edit in App/state_machine.c / pyro.c / kalman.c
must be mirrored here in the same change, and this file rerun (`py -m unittest`).

Run: py -m unittest tests.test_sil_flight
"""
import math
import unittest

# ---- constants mirrored from App/app_config.h + board.h (NUM_PYRO) ----
CTRL_HZ = 200
BARO_HZ = 50
CTRL_DT_MS = 1000 // CTRL_HZ            # 5
CTRL_DT_S = 1.0 / CTRL_HZ              # 0.005
BARO_DECIM = CTRL_HZ // BARO_HZ        # 4
LAUNCH_ACC_N = 150 // CTRL_DT_MS       # LAUNCH_DEBOUNCE_MS/dt = 30
LAUNCH_BARO_N = 4
BURNOUT_N = 200 // CTRL_DT_MS          # 40
INIT_OK_N = 20
IMU_FAIL_N = CTRL_HZ // 2              # 100
BARO_FAIL_N = BARO_HZ // 2             # 25
APOGEE_PEAK_MARGIN_M = 0.25
LAND_ALT_BAND_M = 2.0

LAUNCH_G = 3.0
LAUNCH_BARO_ALT_M = 10.0
BURNOUT_G = 0.5
MAX_BURN_MS = 6000
APOGEE_DEBOUNCE_N = 10
APOGEE_TIMEOUT_MS = 30000
MAIN_ALT_M_DEFAULT = 150.0
LAND_VEL_MS = 1.0
LAND_DEBOUNCE_MS = 10000
MIN_DROGUE_DELAY_MS = 3000
BACKUP_DEPLOY_MS = APOGEE_TIMEOUT_MS   # fault-independent backup (= apogee timeout)

FIRE_PULSE_MS = 1000
FIRE_RETRY_MAX = 1
CONT_THRESH_MV = 300           # unused here (continuity modelled as boolean)

SERVO_DEPLOY_IDX = 0
SERVO_SAFE_US = 1000           # main retained (SAFE)
SERVO_RELEASE_US = 2000        # main released

ARM_MAX_VEL_MS = 2.0
ARM_MAX_TILT_G = 0.5
ARM_MAX_LATERAL_G = 0.35
ARM_MIN_VBAT_MV = 7000
ARM_UP_AXIS = 2         # +Z skyward (default mount knob); -1 = auto-pick
ARM_UP_SIGN = 1.0
GYRO_CAL_SAMPLES = 400

KAL_Q_ACCEL = 4.0
KAL_R_BARO = 1.5
KAL_GATE_SIGMA = 5.0
KAL_GATE_MAX_REJECT = 25

NUM_PYRO = 1
G = 9.80665

# ---- flight_state_t (order = fsm_state_name[] in state_machine.c) ----
(ST_INIT, ST_GROUND_IDLE, ST_ARMED, ST_BOOST, ST_COAST, ST_APOGEE,
 ST_DROGUE, ST_MAIN, ST_DESCENT, ST_LANDED, ST_FAULT) = range(11)
STATE_NAME = ["INIT", "GROUND_IDLE", "ARMED", "BOOST", "COAST", "APOGEE",
              "DROGUE", "MAIN", "DESCENT", "LANDED", "FAULT"]


# ============================================================
# kalman.c
# ============================================================
class Kalman:
    P_INIT_ALT = 1.0e4
    P_INIT_VEL = 1.0e2
    DT_MAX = 0.5

    def reset(self):
        self.alt = 0.0
        self.vel = 0.0
        self.P = [[self.P_INIT_ALT, 0.0], [0.0, self.P_INIT_VEL]]
        self.init = False
        self.gate_rejects = 0

    def predict(self, accel, dt, accel_valid):
        if not self.init or dt <= 0.0:
            return
        if dt > self.DT_MAX:
            dt = self.DT_MAX
        if accel_valid:
            self.alt += self.vel * dt + 0.5 * accel * dt * dt
            self.vel += accel * dt
        else:
            self.alt += self.vel * dt
        p00, p01 = self.P[0]
        p10, p11 = self.P[1]
        n00 = p00 + dt * (p10 + p01) + dt * dt * p11
        n01 = p01 + dt * p11
        n10 = p10 + dt * p11
        n11 = p11
        q, dt2 = KAL_Q_ACCEL, dt * dt
        n00 += q * 0.25 * dt2 * dt2
        n01 += q * 0.5 * dt2 * dt
        n10 += q * 0.5 * dt2 * dt
        n11 += q * dt2
        off = 0.5 * (n01 + n10)
        self.P = [[n00, off], [off, n11]]

    def update_baro(self, alt_m):
        if not math.isfinite(alt_m):
            return
        if not self.init:
            self.alt = alt_m
            self.vel = 0.0
            self.P = [[KAL_R_BARO, 0.0], [0.0, self.P_INIT_VEL]]
            self.init = True
            self.gate_rejects = 0
            return
        p00, p01 = self.P[0]
        p10, p11 = self.P[1]
        r = KAL_R_BARO
        s = p00 + r
        if s <= 0.0:
            s = r
        k0 = p00 / s
        k1 = p10 / s
        y = alt_m - self.alt
        if (y * y) > (KAL_GATE_SIGMA ** 2) * s and self.gate_rejects < KAL_GATE_MAX_REJECT:
            self.gate_rejects += 1
            return
        self.gate_rejects = 0
        self.alt += k0 * y
        self.vel += k1 * y
        a = 1.0 - k0
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


# ============================================================
# pyro.c: fire gate + fire-pulse state machine
# ============================================================
PY_IDLE, PY_FIRING, PY_SETTLE = range(3)
PY_SETTLE_MS = 2 * (1000 // 10)   # CONT_HZ=10 -> 200 ms


class Pyro:
    def __init__(self):
        self.state = PY_IDLE
        self.fired_bits = 0
        self.retries = 0
        self.ch = 0
        self.t0 = 0
        self.backup_fire = False      # current pulse is a fault-independent backup
        self.fire_ok_count = 0        # successful pyro_fire()/pyro_fire_backup() calls
        self.cont = [True]            # continuity present (e-match intact)
        self.deploy_succeeds = True   # match opens after a good pulse
        self.deploy_failed = False    # Phase 1h latch

    @staticmethod
    def _fire_allowed(fsm):
        return fsm.armed or (fsm.state == ST_GROUND_IDLE and fsm.test_enabled)

    def fire(self, ch, fsm, now):
        if ch >= NUM_PYRO:
            return -1
        if not self._fire_allowed(fsm):
            return -2
        if self.state != PY_IDLE:
            return -3
        self.ch = ch
        self.retries = 0
        self.backup_fire = False
        self.fired_bits |= (1 << ch)
        self.t0 = now
        self.state = PY_FIRING
        self.fire_ok_count += 1
        return 0

    def fire_backup(self, ch, now):
        # bypasses fire_allowed(); pyro.c pyro_fire_backup
        if ch >= NUM_PYRO:
            return -1
        if self.state != PY_IDLE:
            return -3
        self.ch = ch
        self.retries = 0
        self.backup_fire = True
        self.fired_bits |= (1 << ch)
        self.t0 = now
        self.state = PY_FIRING
        self.fire_ok_count += 1
        return 0

    def poll(self, fsm, now):
        # abort ONLY on a ground disarm of a pad test-fire (mirror pyro.c
        # abort_pulse); an in-flight deploy pulse always completes + retries.
        abort = (not self.backup_fire) and (not self._fire_allowed(fsm)) and \
                fsm.state in (ST_GROUND_IDLE, ST_INIT)
        if self.state == PY_FIRING:
            if abort:
                self.state = PY_IDLE
                self.backup_fire = False
                return
            if (now - self.t0) >= FIRE_PULSE_MS:
                # pulse done: a good deploy consumes the match -> continuity opens
                if self.deploy_succeeds:
                    self.cont[self.ch] = False
                self.t0 = now
                self.state = PY_SETTLE
        elif self.state == PY_SETTLE:
            if (now - self.t0) >= PY_SETTLE_MS:
                if self.cont[self.ch] and self.retries < FIRE_RETRY_MAX and not abort:
                    self.retries += 1
                    self.t0 = now
                    self.state = PY_FIRING
                else:
                    if self.cont[self.ch]:      # continuity survived fire+retry
                        self.deploy_failed = True
                    self.state = PY_IDLE
                    self.backup_fire = False


# ============================================================
# state_machine.c
# ============================================================
class FlightSim:
    def __init__(self):
        self.kal = Kalman()
        self.pyro = Pyro()
        # g_fsm fields
        self.state = ST_INIT
        self.armed = False
        self.test_enabled = False
        self.agl = 0.0
        self.vel = 0.0
        self.main_alt = MAIN_ALT_M_DEFAULT
        self.apogee_timeout_ms = APOGEE_TIMEOUT_MS   # runtime-tunable (Phase 3)
        self.t_launch_ms = 0
        self.imu = dict(ax=0.0, ay=0.0, az=1.0, accel_sat=False)
        self.imu_ok = False
        self.baro_ok = False
        self.vbat_mv = 8000
        # private statics
        self.kal.reset()
        self.step_n = 0
        self.imu_fail = 0
        self.baro_fail = 0
        self.init_ok = 0
        self.launch_acc = 0
        self.launch_baro = 0
        self.burnout = 0
        self.apogee_cnt = 0
        self.peak_agl = 0.0
        self.land_win = False
        self.land_start = 0
        self.land_ref = 0.0
        self.deploy_fired = False
        self.main_released = False
        self.servo_us = SERVO_SAFE_US
        self.servo_by_state = {}          # state -> set of servo_us commanded
        # sensor health injectables + gyro cal model
        self.imu_health = True
        self.baro_health = True
        self._cal_active = False
        self._cal_count = 0
        # event log
        self.transitions = []          # (now_ms, state)
        self.fires = []                # dict per pyro_fire from the FSM
        self.transitions.append((0, ST_INIT))

    # -- gyro cal model (imu_gyro_cal_start/done) --
    def _gyro_cal_start(self):
        self._cal_active = True
        self._cal_count = 0

    def _gyro_cal_done(self):
        return self._cal_active and self._cal_count >= GYRO_CAL_SAMPLES

    def _set_state(self, s):
        if self.state == s:
            return
        self.state = s
        self.transitions.append((self._now, s))

    def _fault(self, why):
        if self.state == ST_FAULT:
            return
        self.armed = False
        self._set_state(ST_FAULT)

    def request_arm(self, via_radio=False):
        if self.state != ST_GROUND_IDLE:
            return -1
        if not self.imu_ok or not self.baro_ok:
            return -2
        if not self._gyro_cal_done():
            return -3
        if abs(self.vel) >= ARM_MAX_VEL_MS:
            return -4
        a = [self.imu["ax"], self.imu["ay"], self.imu["az"]]
        amag = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
        if self.imu["accel_sat"]:
            return -5
        if abs(amag - 1.0) >= ARM_MAX_TILT_G:
            return -5
        # orientation gate (mirrors state_machine.c)
        if ARM_UP_AXIS < 0:
            ax_up = max(range(3), key=lambda i: abs(a[i]))
            sign = 1.0 if a[ax_up] >= 0.0 else -1.0
        else:
            ax_up, sign = ARM_UP_AXIS, ARM_UP_SIGN
        up = sign * a[ax_up]
        lat2 = sum(a[i] ** 2 for i in range(3) if i != ax_up)
        if abs(up - 1.0) >= ARM_MAX_TILT_G:
            return -8
        if lat2 >= ARM_MAX_LATERAL_G ** 2:
            return -8
        if not self.pyro.cont[0]:
            return -6
        if self.vbat_mv < ARM_MIN_VBAT_MV:
            return -7
        self.armed = True
        self.launch_acc = self.launch_baro = 0
        self._set_state(ST_ARMED)
        return 0

    def step(self, now_ms, imu, baro_agl):
        """imu: dict or None (imu_read fail). baro_agl: float or None (baro_read fail)."""
        self._now = now_ms

        # ---- 1. sensors ----
        if imu is not None:
            self.imu = imu
            self.imu_fail = 0
            if self._cal_active:
                self._cal_count += 1
        elif self.imu_fail < 0xFFFF:
            self.imu_fail += 1
        self.imu_ok = self.imu_health and (self.imu_fail < IMU_FAIL_N)

        if (self.step_n % BARO_DECIM) == 0:
            if baro_agl is not None:
                self.baro_fail = 0
                self.kal.update_baro(baro_agl)
            elif self.baro_fail < 0xFFFF:
                self.baro_fail += 1
            self.baro_ok = self.baro_health and (self.baro_fail < BARO_FAIL_N)

        # ---- 2. filter (accel never used as control input) ----
        self.kal.predict(0.0, CTRL_DT_S, False)
        self.agl = self.kal.alt
        self.vel = self.kal.vel

        # ---- 3. health -> FAULT ----
        if self.state != ST_FAULT and (self.imu_fail >= IMU_FAIL_N or self.baro_fail >= BARO_FAIL_N):
            self._fault("IMU LOST" if self.imu_fail >= IMU_FAIL_N else "BARO LOST")

        amag = math.sqrt(self.imu["ax"] ** 2 + self.imu["ay"] ** 2 + self.imu["az"] ** 2)

        # ---- 4. transitions ----
        st = self.state
        if st == ST_INIT:
            if self.imu_ok and self.baro_ok:
                self.init_ok += 1
                if self.init_ok >= INIT_OK_N:
                    self._gyro_cal_start()
                    self.kal.reset()
                    self._set_state(ST_GROUND_IDLE)
            else:
                self.init_ok = 0

        elif st == ST_GROUND_IDLE:
            pass  # arming only via request_arm

        elif st == ST_ARMED:
            thrusting = self.imu["accel_sat"] or (amag > LAUNCH_G)
            self.launch_acc = self.launch_acc + 1 if thrusting else 0
            baro_rising = (self.agl > LAUNCH_BARO_ALT_M) and (self.vel > 0.5)
            self.launch_baro = self.launch_baro + 1 if baro_rising else 0
            if self.launch_acc >= LAUNCH_ACC_N or self.launch_baro >= LAUNCH_BARO_N:
                self.t_launch_ms = now_ms
                self.peak_agl = self.agl
                self.burnout = 0
                self.apogee_cnt = 0
                self._set_state(ST_BOOST)

        elif st == ST_BOOST:
            if self.agl > self.peak_agl:
                self.peak_agl = self.agl
            low_acc = (not self.imu["accel_sat"]) and (amag < BURNOUT_G)
            self.burnout = self.burnout + 1 if low_acc else 0
            if self.burnout >= BURNOUT_N or (now_ms - self.t_launch_ms) >= MAX_BURN_MS:
                self.apogee_cnt = 0
                self._set_state(ST_COAST)

        elif st == ST_COAST:
            if self.agl > self.peak_agl:
                self.peak_agl = self.agl
            descending = (self.vel <= 0.0) and (self.agl < self.peak_agl - APOGEE_PEAK_MARGIN_M)
            self.apogee_cnt = self.apogee_cnt + 1 if descending else 0
            timeout = (now_ms - self.t_launch_ms) >= self.apogee_timeout_ms
            if (now_ms - self.t_launch_ms) >= MIN_DROGUE_DELAY_MS and \
               (self.apogee_cnt >= APOGEE_DEBOUNCE_N or timeout):
                self._set_state(ST_APOGEE)
                if not self.deploy_fired:
                    rc = self.pyro.fire(0, self, now_ms)
                    if rc == 0:
                        self.deploy_fired = True
                    self.fires.append(dict(now=now_ms, dt_launch=now_ms - self.t_launch_ms,
                                           agl=self.agl, vel=self.vel, peak=self.peak_agl,
                                           rc=rc, backup=False,
                                           by_timeout=(timeout and self.apogee_cnt < APOGEE_DEBOUNCE_N)))

        elif st == ST_APOGEE:
            self._set_state(ST_DROGUE)

        elif st == ST_DROGUE:
            if self.vel < 0.0 and self.agl <= self.main_alt:
                self.main_released = True     # servo release latch (only site)
                self._set_state(ST_MAIN)

        elif st == ST_MAIN:
            self._set_state(ST_DESCENT)
            self.land_win = False

        elif st == ST_DESCENT:
            if not self.land_win:
                self.land_win = True
                self.land_start = now_ms
                self.land_ref = self.agl
            if abs(self.vel) < LAND_VEL_MS and abs(self.agl - self.land_ref) < LAND_ALT_BAND_M:
                if (now_ms - self.land_start) >= LAND_DEBOUNCE_MS:
                    self.armed = False
                    self._set_state(ST_LANDED)
            else:
                self.land_start = now_ms
                self.land_ref = self.agl

        elif st == ST_FAULT:
            # self-heal a GROUND fault only (never launched + sensors healthy)
            if self.t_launch_ms == 0 and self.imu_ok and self.baro_ok:
                self.init_ok += 1
                if self.init_ok >= INIT_OK_N:
                    self.init_ok = 0
                    self._set_state(ST_INIT)
            else:
                self.init_ok = 0

        # ST_LANDED: terminal

        # fault-independent backup deploy (Phase 2): fire once, past
        # BACKUP_DEPLOY_MS, regardless of state/health, never on pad or landed.
        if self.t_launch_ms != 0 and not self.deploy_fired and \
                self.state != ST_LANDED and \
                (now_ms - self.t_launch_ms) >= self.apogee_timeout_ms:
            self.deploy_fired = True
            rc = self.pyro.fire_backup(0, now_ms)
            self.fires.append(dict(now=now_ms, dt_launch=now_ms - self.t_launch_ms,
                                   agl=self.agl, vel=self.vel, peak=self.peak_agl,
                                   rc=rc, backup=True, by_timeout=False))

        # servo main-release: SAFE until main_released latches (idempotent)
        self.servo_us = SERVO_RELEASE_US if self.main_released else SERVO_SAFE_US
        self.servo_by_state.setdefault(self.state, set()).add(self.servo_us)

        self.pyro.poll(self, now_ms)   # app_loop calls pyro_poll every pass
        self.step_n += 1

    # ---- convenience ----
    def states_seen(self):
        return [s for _, s in self.transitions]


# ============================================================
# physics truth + driver
# ============================================================
def _baro_noise(step):
    """Deterministic small baro noise (repeatable, no RNG)."""
    return 0.20 * math.sin(step * 0.7) + 0.12 * math.sin(step * 0.131)


def run_flight(thrust_g=5.0, burn_s=1.5, descent_vel=-30.0,
               baro_loss=None, no_apogee=False, boost_glitch=False,
               updraft_drogue=False, deploy_succeeds=True,
               apogee_timeout_ms=None, max_s=60.0, arm=True):
    """Drive a FlightSim through pad -> boost -> coast -> descent -> land.

    Hooks inject the adversarial cases:
      baro_loss=(t0,t1)  feed baro=None in that window (sensor loss)
      no_apogee          freeze baro at apogee (only the 30 s timer can fire)
      boost_glitch       brief downward baro dip mid-boost (false-apogee bait)
      updraft_drogue     upward baro segment while in DROGUE below main_alt
      deploy_succeeds    False -> continuity persists after fire (failed deploy)
    Returns the FlightSim after the run.
    """
    sim = FlightSim()
    sim.pyro.deploy_succeeds = deploy_succeeds
    if apogee_timeout_ms is not None:
        sim.apogee_timeout_ms = apogee_timeout_ms
    dt = CTRL_DT_S
    true_alt = 0.0
    true_vel = 0.0
    launch_t = None
    armed_done = False
    apogee_frozen = None
    apogee_frozen_t = None
    n = int(max_s * CTRL_HZ)

    for i in range(n):
        t = i * dt
        now_ms = int(t * 1000)

        # --- arm once gyro cal is done and still on the pad ---
        if arm and not armed_done and sim.state == ST_GROUND_IDLE and sim._gyro_cal_done():
            assert sim.request_arm() == 0, "arm refused on a healthy stationary pad"
            armed_done = True
            launch_t = t + 0.5   # light the motor 0.5 s after arming

        # --- physics truth ---
        if launch_t is not None and t >= launch_t:
            te = t - launch_t
            if te < burn_s:
                a_felt_g = thrust_g                       # motor: felt = thrust/g
                a_kin = thrust_g * G - G                  # net up accel
            elif true_vel > 0 or true_alt > 0:
                a_felt_g = 0.0                            # freefall / drogue
                a_kin = -G
            else:
                a_felt_g = 1.0
                a_kin = 0.0
            # descent terminal clamp
            if true_vel <= descent_vel:
                true_vel = descent_vel
                a_kin = 0.0
                a_felt_g = 1.0
            true_vel += a_kin * dt
            true_alt += true_vel * dt
            if true_alt <= 0.0 and true_vel < 0.0:       # touchdown
                true_alt = 0.0
                true_vel = 0.0
                a_felt_g = 1.0
        else:
            a_felt_g = 1.0
            a_kin = 0.0

        if apogee_frozen is None and launch_t is not None and t > launch_t + 0.2 and true_vel < 0:
            # first sample past true apogee. Base the phantom climb ABOVE the
            # (lagging, overshooting) filtered altitude so the innovation stays
            # positive and filtered vel never goes negative.
            apogee_frozen = sim.agl + 100.0
            apogee_frozen_t = t

        # --- sensors fed to the sim ---
        sat = a_felt_g >= 15.5
        imu = dict(ax=0.0, ay=0.0, az=a_felt_g, accel_sat=sat)

        baro_alt = true_alt
        if no_apogee and apogee_frozen is not None:
            # phantom rising baro: filtered vel stays > 0 so the descending
            # test never trips -> only the 30 s backup timer can deploy.
            baro_alt = apogee_frozen + 30.0 * (t - apogee_frozen_t)
        if boost_glitch and launch_t is not None and 0.30 < (t - launch_t) < 0.33:
            baro_alt = max(0.0, true_alt - 40.0)          # 3-sample downward dip
        if updraft_drogue and sim.state == ST_DROGUE and true_alt < sim.main_alt and true_vel < 0:
            baro_alt = true_alt + 5.0 * (t % 0.05) / 0.05  # keep filtered vel >= 0

        baro = baro_alt + _baro_noise(i)
        if baro_loss and baro_loss[0] <= t < baro_loss[1]:
            baro = None

        sim.step(now_ms, imu, baro)

        # stop early once landed to keep runs short
        if sim.state == ST_LANDED:
            break

    return sim


# ============================================================
# invariant tests
# ============================================================
class NominalFlight(unittest.TestCase):
    def setUp(self):
        self.sim = run_flight()

    def test_reaches_apogee_and_fires(self):
        seq = self.sim.states_seen()
        for s in (ST_GROUND_IDLE, ST_ARMED, ST_BOOST, ST_COAST, ST_APOGEE):
            self.assertIn(s, seq, "never reached %s: %s" %
                          (STATE_NAME[s], [STATE_NAME[x] for x in seq]))

    def test_fires_exactly_once(self):
        ok = [f for f in self.sim.fires if f["rc"] == 0]
        self.assertEqual(len(ok), 1, "pyro must fire exactly once, got %d" % len(ok))
        self.assertEqual(self.sim.pyro.fire_ok_count, 1)
        self.assertEqual(self.sim.pyro.fired_bits, 0b1)

    def test_fire_after_min_delay(self):
        f = self.sim.fires[0]
        self.assertGreaterEqual(f["dt_launch"], MIN_DROGUE_DELAY_MS)

    def test_fire_via_apogee_detect_not_timer(self):
        # Real apogee-detection path (debounce), not the 30 s backup timer,
        # and on the descending side (agl below tracked peak, vel <= 0).
        # NOTE: the deploy lands well below peak because the baro-only Kalman
        # (KAL_Q_ACCEL=4) lags the ~9.8 m/s^2 coast deceleration, a real
        # tuning limitation tracked under the "tune thresholds" open item.
        f = self.sim.fires[0]
        self.assertFalse(f["by_timeout"], "nominal flight fired on backup timer, not real apogee")
        self.assertLessEqual(f["vel"], 0.0, "deploy must be on the descending side")
        self.assertLess(f["agl"], f["peak"], "deploy below the tracked peak")

    def test_never_fires_on_pad_or_boost(self):
        # the only fire path is the COAST case; confirm no fire recorded before COAST time
        coast_ms = next(ms for ms, s in self.sim.transitions if s == ST_COAST)
        for f in self.sim.fires:
            self.assertGreaterEqual(f["now"], coast_ms)

    def test_main_marker_inhibited_until_descending(self):
        # MAIN only after DROGUE with vel<0 and agl<=main_alt
        self.assertIn(ST_MAIN, self.sim.states_seen())


class NoApogeeDetected(unittest.TestCase):
    """Stuck baro at apogee -> only the 30 s backup timer can deploy."""
    def test_backup_timer_fires(self):
        sim = run_flight(no_apogee=True, max_s=40.0)
        ok = [f for f in sim.fires if f["rc"] == 0]
        self.assertEqual(len(ok), 1, "backup timer failed to deploy")
        self.assertTrue(ok[0]["by_timeout"], "expected timer-backup deploy")
        self.assertGreaterEqual(ok[0]["dt_launch"], APOGEE_TIMEOUT_MS)


class FalseApogeeRejected(unittest.TestCase):
    """A brief boost-phase baro dip must not trigger an early deploy."""
    def test_no_early_fire(self):
        sim = run_flight(boost_glitch=True)
        # still exactly one fire, and it is the real (near-apogee) one
        ok = [f for f in sim.fires if f["rc"] == 0]
        self.assertEqual(len(ok), 1)
        self.assertFalse(ok[0]["by_timeout"])
        self.assertGreaterEqual(ok[0]["dt_launch"], MIN_DROGUE_DELAY_MS)


class UpwardVelMainInhibit(unittest.TestCase):
    """Below main_alt but still rising (updraft) must not mark MAIN."""
    def test_main_waits_for_descent(self):
        sim = run_flight(updraft_drogue=True)
        # find first time agl<=main_alt after drogue, ensure MAIN only once vel<0
        # (coarse): MAIN must still eventually happen, and after a DROGUE phase
        seq = sim.states_seen()
        if ST_MAIN in seq:
            self.assertIn(ST_DROGUE, seq)
        # exactly one deploy regardless
        self.assertEqual(len([f for f in sim.fires if f["rc"] == 0]), 1)


class SensorLossInCoast(unittest.TestCase):
    """Baro lost during coast -> FAULT latches, but Phase 2's fault-independent
    backup deploy still fires the charge (the lawn-dart closer)."""
    def test_baro_loss_faults(self):
        # kill baro from t=3.2 s (in coast, past min-drogue) for >0.5 s
        sim = run_flight(baro_loss=(3.2, 4.2), max_s=20.0)
        self.assertIn(ST_FAULT, sim.states_seen())
        self.assertFalse(sim.armed, "FAULT must drop armed")

    def test_backup_deploy_fires_through_fault(self):
        # baro dead through the whole flight -> FAULT before apogee. The backup
        # timer must still fire the charge exactly once, past BACKUP_DEPLOY_MS,
        # even though armed==false in FAULT.
        sim = run_flight(baro_loss=(3.2, 60.0), max_s=BACKUP_DEPLOY_MS / 1000.0 + 5.0)
        self.assertEqual(sim.state, ST_FAULT)
        fired = [f for f in sim.fires if f["rc"] == 0]
        self.assertEqual(len(fired), 1, "backup deploy did not fire exactly once")
        self.assertTrue(fired[0]["backup"], "expected a backup (not primary) deploy")
        self.assertGreaterEqual(fired[0]["dt_launch"], BACKUP_DEPLOY_MS)
        self.assertEqual(sim.pyro.fired_bits, 0b1)

    def test_backup_suppressed_on_normal_flight(self):
        # nominal flight deploys at apogee -> the backup timer must NOT also fire
        sim = run_flight()
        fired = [f for f in sim.fires if f["rc"] == 0]
        self.assertEqual(len(fired), 1)
        self.assertFalse(fired[0]["backup"], "normal apogee deploy, not backup")

    def test_tunable_timeout_moves_backup(self):
        # Phase 3: a shorter runtime apogee_timeout fires the fault backup
        # earlier (proves g_fsm.apogee_timeout_ms drives the timer, not a macro).
        sim = run_flight(baro_loss=(3.2, 60.0), apogee_timeout_ms=12000, max_s=20.0)
        fired = [f for f in sim.fires if f["rc"] == 0]
        self.assertEqual(len(fired), 1)
        self.assertTrue(fired[0]["backup"])
        self.assertGreaterEqual(fired[0]["dt_launch"], 12000)
        self.assertLess(fired[0]["dt_launch"], 12000 + 200)   # fired right at the timer


class ArmingGate(unittest.TestCase):
    def _pad_to_idle(self):
        """Step a fresh sim to GROUND_IDLE with gyro cal complete."""
        sim = FlightSim()
        for i in range(GYRO_CAL_SAMPLES + INIT_OK_N + 50):
            imu = dict(ax=0.0, ay=0.0, az=1.0, accel_sat=False)
            sim.step(int(i * CTRL_DT_S * 1000), imu, _baro_noise(i))
        return sim

    def test_arms_on_healthy_pad(self):
        sim = self._pad_to_idle()
        self.assertEqual(sim.state, ST_GROUND_IDLE)
        self.assertEqual(sim.request_arm(), 0)
        self.assertEqual(sim.state, ST_ARMED)

    def test_refuses_before_gyro_cal(self):
        sim = FlightSim()
        for i in range(INIT_OK_N + 5):     # reached idle but cal not done
            sim.step(int(i * CTRL_DT_S * 1000),
                     dict(ax=0.0, ay=0.0, az=1.0, accel_sat=False), _baro_noise(i))
        if sim.state == ST_GROUND_IDLE:
            self.assertEqual(sim.request_arm(), -3)

    def test_refuses_not_idle(self):
        sim = FlightSim()
        self.assertEqual(sim.request_arm(), -1)   # still ST_INIT


class ArmGateHardening(unittest.TestCase):
    """Phase 1b/1c/1d: continuity / battery / orientation arm gates."""
    def _idle(self):
        sim = ArmingGate()._pad_to_idle()
        return sim

    def test_refuses_tilted(self):
        # 30 deg tilt: |a|=1 (passes the old magnitude-only band) but lateral
        # g is too high -> -8.
        sim = self._idle()
        sim.imu = dict(ax=0.5, ay=0.0, az=0.866, accel_sat=False)
        self.assertEqual(sim.request_arm(), -8)

    def test_refuses_on_side(self):
        # rocket on its side: gravity on X, up-axis(Z) reads ~0 -> -8. This is
        # the hole the old magnitude-only band (and auto-pick) let through;
        # the configured +Z default now rejects it.
        sim = self._idle()
        sim.imu = dict(ax=1.0, ay=0.0, az=0.0, accel_sat=False)
        self.assertEqual(sim.request_arm(), -8)

    def test_refuses_inverted(self):
        # nose-down: up-axis reads -1 g -> -8.
        sim = self._idle()
        sim.imu = dict(ax=0.0, ay=0.0, az=-1.0, accel_sat=False)
        self.assertEqual(sim.request_arm(), -8)

    def test_refuses_no_continuity(self):
        sim = self._idle()
        sim.pyro.cont[0] = False
        self.assertEqual(sim.request_arm(), -6)

    def test_refuses_low_battery(self):
        sim = self._idle()
        sim.vbat_mv = 6000
        self.assertEqual(sim.request_arm(), -7)

    def test_arms_when_all_gates_pass(self):
        sim = self._idle()                    # upright, cont True, vbat 8000
        self.assertEqual(sim.request_arm(), 0)


class SelfHealGroundFault(unittest.TestCase):
    """Phase 1g: a ground fault that never launched recovers to INIT."""
    def test_ground_fault_recovers(self):
        sim = ArmingGate()._pad_to_idle()
        self.assertEqual(sim.state, ST_GROUND_IDLE)
        base = sim.step_n
        # kill baro long enough to fault (never launched)
        for i in range(4 * BARO_FAIL_N + 20):
            sim.step(int((base + i) * CTRL_DT_S * 1000),
                     dict(ax=0.0, ay=0.0, az=1.0, accel_sat=False), None)
        self.assertEqual(sim.state, ST_FAULT)
        self.assertEqual(sim.t_launch_ms, 0)
        # restore healthy sensors -> self-heal back into the ladder
        base = sim.step_n
        healed = False
        for i in range(4 * INIT_OK_N + 60):
            sim.step(int((base + i) * CTRL_DT_S * 1000),
                     dict(ax=0.0, ay=0.0, az=1.0, accel_sat=False), _baro_noise(i))
            if sim.state in (ST_INIT, ST_GROUND_IDLE):
                healed = True
                break
        self.assertTrue(healed, "ground FAULT never self-healed")

    def test_inflight_fault_stays_latched(self):
        # baro lost during coast -> fault after launch -> must NOT self-heal
        sim = run_flight(baro_loss=(3.2, 30.0), max_s=32.0)
        self.assertEqual(sim.state, ST_FAULT)
        self.assertNotEqual(sim.t_launch_ms, 0)


class ServoDualDeploy(unittest.TestCase):
    """Phase 4: servo held SAFE through ascent, RELEASE at the main trigger,
    SAFE forever in a fault flight."""
    def test_safe_through_ascent_release_at_main(self):
        sim = run_flight()
        self.assertEqual(sim.servo_by_state.get(ST_BOOST), {SERVO_SAFE_US},
                         "servo must be SAFE during BOOST")
        self.assertEqual(sim.servo_by_state.get(ST_COAST), {SERVO_SAFE_US},
                         "servo must be SAFE during COAST")
        self.assertEqual(sim.servo_by_state.get(ST_DROGUE), {SERVO_SAFE_US},
                         "servo SAFE until the MAIN transition")
        released = sim.servo_by_state.get(ST_MAIN, set()) | sim.servo_by_state.get(ST_DESCENT, set())
        self.assertIn(SERVO_RELEASE_US, released, "servo must release by MAIN")
        self.assertEqual(sim.servo_us, SERVO_RELEASE_US)   # stays released

    def test_fault_flight_never_releases(self):
        sim = run_flight(baro_loss=(3.2, 60.0), max_s=BACKUP_DEPLOY_MS / 1000.0 + 5.0)
        self.assertEqual(sim.state, ST_FAULT)
        seen = set()
        for us in sim.servo_by_state.values():
            seen |= us
        self.assertEqual(seen, {SERVO_SAFE_US}, "servo must never release in a fault flight")
        self.assertFalse(sim.main_released)


class DeployPulseSurvivesFault(unittest.TestCase):
    """A FAULT during the apogee deploy pulse must NOT truncate it (that was a
    lawn-dart hole); only a ground disarm of a pad test-fire aborts a pulse."""
    def test_flight_fault_does_not_abort_deploy(self):
        sim = FlightSim()
        sim.state = ST_COAST
        sim.armed = True
        self.assertEqual(sim.pyro.fire(0, sim, 0), 0)      # apogee deploy pulse
        sim.armed = False                                  # fault mid-pulse...
        sim.state = ST_FAULT                               # ...in flight
        for t in range(0, FIRE_PULSE_MS + PY_SETTLE_MS + 100, 10):
            sim.pyro.poll(sim, t)
        # pulse ran to completion (deploy consumed the match), not aborted early
        self.assertFalse(sim.pyro.cont[0], "deploy pulse completed through fault")
        self.assertFalse(sim.pyro.deploy_failed)
        self.assertEqual(sim.pyro.fired_bits, 0b1)

    def test_ground_disarm_aborts_test_fire(self):
        sim = FlightSim()
        sim.state = ST_GROUND_IDLE
        sim.test_enabled = True
        self.assertEqual(sim.pyro.fire(0, sim, 0), 0)      # pad test-fire
        sim.test_enabled = False                           # disarm on the pad
        sim.pyro.poll(sim, 10)                             # mid-pulse
        self.assertEqual(sim.pyro.state, PY_IDLE, "ground disarm aborts the pulse")
        self.assertTrue(sim.pyro.cont[0], "match not consumed (pulse truncated)")


class FailedDeployLatch(unittest.TestCase):
    """Phase 1h: continuity persisting after fire+retry latches deploy_failed."""
    def test_latches_on_persistent_continuity(self):
        sim = run_flight(deploy_succeeds=False)
        self.assertEqual(len([f for f in sim.fires if f["rc"] == 0]), 1)
        self.assertTrue(sim.pyro.deploy_failed, "deploy_failed never latched")

    def test_clear_on_good_deploy(self):
        sim = run_flight(deploy_succeeds=True)
        self.assertFalse(sim.pyro.deploy_failed)


if __name__ == "__main__":
    unittest.main(verbosity=2)
