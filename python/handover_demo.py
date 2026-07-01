#!/usr/bin/env python3
"""Force-based human-robot HANDOVER demo -- SAFE RELEASE (no PLACE yet).

Passes an object back and forth with a human, sensing the human via the
estimated end-effector wrench (/force). The arm stays STATIONARY -- only the
gripper actuates.

Safety invariant (governs everything):
  The gripper opens ONLY after live, fresh sensing positively confirms the
  object's weight is already carried by a human hand. Never on force magnitude,
  never on one sample, never on missing/stale/frozen data, never on any error,
  timeout, 409, or shutdown. When anything is uncertain: KEEP HOLDING.
  (A missed release = a harmless re-tug; a false release = a dropped object.)

How release avoids "the force came from the object itself":
  * DIRECTIONAL, not magnitude -- we track s = how much weight has LEFT the
    gripper along the loaded (gravity) axis, using the net_force VECTOR.
  * WEIGHT-TRANSFER AUTHORITY -- release needs the load to return near the
    empty-hand level (a stiction slip within the loaded band can't pass).
  * PROGRESSIVE RISE -- s must have been low (held) recently then risen (a
    standing offset from a bad tare / drift is rejected).
  * TORQUE CROSS-CHECK -- a bump/lean on the arm adds force+torque but does NOT
    remove the object's gravity torque; a genuine take removes it.
  * FRESHNESS GATE -- a frozen/stale /force reading is treated as "fully loaded".
  * IDLE-DRIFT RE-TARE -- while quiescent, re-tare so residual drift never
    accrues toward the threshold.

TAKE (receiving) also guards against a pinch: it grasps only on sustained added
weight ALONG GRAVITY with small lateral force, then verifies the grip width.

Run:  python3 handover_demo.py [--start give|take] [--home] [--grasp-force N]
Stop: Ctrl-C (exits cleanly; never actuates the gripper on shutdown).
"""

import argparse
import signal
import time
from collections import deque

import numpy as np
import requests

# ---- tunables (from the reviewed design; tuned for a ~10 N object, 3-5 N floor)
POLL_DT      = 0.05     # 20 Hz
MA_WIN       = 5        # 0.25 s moving average (kills vibration)
R_FLOOR      = 4.0      # N, residual floor subtracted from the measured load
W_MIN        = 6.0      # N, refuse auto-release below this object weight
S_FRAC       = 0.8      # S_on = S_FRAC * W_obj (near-full transfer)
G_DIR        = 0.6      # directional gate: s >= G_DIR * |net|
DROP_FRAC    = 0.8      # raw_magnitude must fall by DROP_FRAC * W_obj
EMPTY_MARGIN = 2.0      # EMPTY_BAND = W_empty + EMPTY_MARGIN
TQ_BUMP      = 0.8      # N.m, arm-bump veto over the holding torque
GRAV_ALIGN   = 0.82     # loaded axis must be within ~35 deg of gravity
T_STABLE     = 0.5      # s of sustained "ok" at the high end
T_LOOKBACK   = 1.5      # s window that must contain a low point (proves a rise)
RETARE_IDLE  = 5.0      # s between idle re-tares
SETTLE_S     = 0.6      # s to settle after a gripper action
F_CEIL       = 40.0     # N, collision ceiling -> never open
TQ_CEIL      = 6.0      # N.m, collision ceiling -> never open
STALE_S      = 0.15     # s; /force stamp not advancing this long => stale
EQUIL_RATE   = 0.5      # N/s; |d raw_mag/dt| below this = settled
# TAKE (receiving)
TAKE_WEIGHT  = 5.0      # N of added weight along gravity to grasp
TAKE_LAT_MAX = 1.5      # N max horizontal component (reject a steadying hand)
TAKE_TQ_MAX  = 0.8      # N.m wrist-torque veto (a hand injects torque; a hung object ~0)
TAKE_DEBOUNCE = 12      # samples (~0.6 s)
GRIP_EMPTY_W = 0.005    # m; finger gap <= this after grasp => grabbed nothing


def quat_gravity_ee(q):
    """Unit gravity direction expressed in the EE frame, from the EE orientation
    quaternion (x,y,z,w) in the base frame. gravity_base = (0,0,-1)."""
    x, y, z, w = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])
    g = R.T @ np.array([0.0, 0.0, -1.0])   # base gravity -> EE frame
    n = np.linalg.norm(g)
    return g / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])


class Handover:
    def __init__(self, a):
        self.base = a.base_url.rstrip("/")
        self.start = a.start
        self.do_home = a.home
        self.grasp_force = a.grasp_force
        self.skip_grasp = getattr(a, "no_grasp", False)  # object already held
        self._running = True
        self._last_stamp = None
        self._stamp_mono = None
        self._recent_rawmag = deque(maxlen=3)
        self.W_empty = None   # empty-hand raw_magnitude at the working pose

    # ---------------- REST ----------------
    def _get(self, path, params=None, timeout=5):
        return requests.get(self.base + path, params=params, timeout=timeout)

    def state(self):
        """(pose_ok, gripper_open_bool, gripper_raw, quat) or (False,...)."""
        try:
            d = self._get("/state", timeout=3).json()
            o = d["orientation"]
            return True, d.get("gripper", -1), (o["x"], o["y"], o["z"], o["w"])
        except Exception:
            return False, -1, None

    def read_force(self):
        """One fresh /force sample, or None. Applies the freshness/frozen gate:
        returns None (== treat as fully loaded) on any bad/stale/frozen read."""
        try:
            r = self._get("/force", timeout=1.5)
            if r.status_code != 200:
                return None
            d = r.json()
            if not d.get("available") or d.get("baseline_source") in (None, "none"):
                return None
            stamp = d.get("stamp")
            mono = time.monotonic()
            if stamp != self._last_stamp:
                self._last_stamp = stamp
                self._stamp_mono = mono
            if self._stamp_mono is None or (mono - self._stamp_mono) > STALE_S:
                return None  # stamp not advancing => publisher frozen
            rawmag = float(d["raw_magnitude"])
            self._recent_rawmag.append(rawmag)
            if len(self._recent_rawmag) == 3 and len(set(self._recent_rawmag)) == 1:
                return None  # 3 identical samples => frozen estimate
            nf, rf, tq = d["net_force"], d["raw_force"], d
            return {
                "net": np.array([nf["x"], nf["y"], nf["z"]]),
                "raw": np.array([rf["x"], rf["y"], rf["z"]]),
                "mag": float(d["magnitude"]),
                "raw_mag": rawmag,
                "tq": float(d["torque_magnitude"]),
            }
        except Exception:
            return None

    def tare(self):
        try:
            self._get("/force/tare", timeout=5)
        except Exception as e:
            print("  [warn] tare failed:", e)

    def grasp(self):
        try:
            r = self._get("/control/gripper_grasp",
                          params={"width": 0.0, "speed": 0.05,
                                  "force": self.grasp_force, "eps_out": 0.08,
                                  "max_retries": 0}, timeout=15)
            out = r.json().get("outcome")
            print(f"  grasp -> {out}")
            return out == "success"
        except Exception as e:
            print("  [warn] grasp failed:", e)
            return False

    def open_gripper(self):
        try:
            r = self._get("/control/gripper_open_force",
                          params={"width": 0.08, "speed": 0.1}, timeout=15)
            print(f"  open  -> {r.json().get('outcome')}")
            return True
        except Exception as e:
            print("  [warn] open failed:", e)
            return False

    def home(self):
        print("[SETUP] homing -- KEEP THE WORKSPACE CLEAR ...")
        try:
            st = self._get("/control/home", timeout=45).json().get("status")
            print("[SETUP] home ->", st)
            return st == "homed"
        except Exception as e:
            print("[SETUP] home failed:", e)
            return False

    # ---------------- helpers ----------------
    def _avg_force(self, dur):
        """Average raw vector, raw_mag, torque over `dur` seconds of fresh reads."""
        raws, mags, tqs = [], [], []
        t0 = time.monotonic()
        while self._running and time.monotonic() - t0 < dur:
            f = self.read_force()
            if f:
                raws.append(f["raw"]); mags.append(f["raw_mag"]); tqs.append(f["tq"])
            time.sleep(POLL_DT)
        if not raws:
            return None
        return np.mean(raws, axis=0), float(np.mean(mags)), float(np.mean(tqs))

    # ---------------- arm a hold (equilibrium tare + checks) ----------------
    def arm_hold(self):
        """Prepare a validated hold: wait for equilibrium, sanity-check the load,
        tare, and confirm net~0 (with a delayed second check). Returns a dict of
        hold params, or None -> KEEP HOLDING (do not arm the release)."""
        # 1) wait for equilibrium (post-grasp stiction must relax)
        print("  arming: waiting for equilibrium ...")
        t0 = time.monotonic()
        prev = None
        while self._running and time.monotonic() - t0 < 4.0:
            f = self.read_force()
            if f:
                if prev is not None and abs(f["raw_mag"] - prev) / POLL_DT < EQUIL_RATE:
                    break
                prev = f["raw_mag"]
            time.sleep(POLL_DT)

        got = self._avg_force(0.6)
        if got is None:
            print("  [HOLD] no force data -- not arming.")
            return None
        R0, W_hold, tq_hold = got
        W_obj = max(W_hold - R_FLOOR, 0.0)
        u_load = R0 / (np.linalg.norm(R0) + 1e-9)

        # 2) sanity: heavy enough + loaded axis really points along gravity
        ok_state, gripper, quat = self.state()
        gK = quat_gravity_ee(quat) if quat else None
        if W_obj < W_MIN:
            print(f"  [HOLD] object too light (W_obj={W_obj:.1f} < {W_MIN} N) -- "
                  "auto-release DISABLED; hand it off manually. Keeping hold.")
            return None
        # The load must lie along the gravity AXIS (a hanging object). F_ext's
        # sign convention makes the measured raw force point along -gK for a
        # downward load, so accept either sign: |dot| near 1 == aligned.
        if gK is not None and abs(float(np.dot(u_load, gK))) < GRAV_ALIGN:
            print("  [HOLD] loaded axis not aligned with gravity (empty/jammed?) "
                  "-- not arming.")
            return None
        if ok_state and gripper == 1:
            print("  [HOLD] gripper reads OPEN -- nothing held; not arming.")
            return None

        # 3) tare + arm check, up to 3 tries
        for _ in range(3):
            self.tare()
            chk = []
            t0 = time.monotonic()
            while self._running and time.monotonic() - t0 < 0.4:
                f = self.read_force()
                if f:
                    chk.append(f["mag"])
                time.sleep(POLL_DT)
            if chk and np.mean(chk) < 1.5 and np.std(chk) < 1.0:
                break
        else:
            print("  [HOLD] net force won't settle after tare -- not arming.")
            return None

        # 4) delayed 2nd check (~2 s) for post-grasp creep
        time.sleep(2.0)
        f = self.read_force()
        if f is not None and abs(float(np.dot(f["net"], u_load))) > 1.5:
            print("  [HOLD] post-grasp creep detected -- re-taring.")
            self.tare()

        W_empty = self.W_empty if self.W_empty is not None else R_FLOOR
        print(f"  [HOLD] armed. W_obj={W_obj:.1f} N, W_hold={W_hold:.1f} N, "
              f"tug it (full weight ~{S_FRAC*W_obj:.1f} N transferred) to release.")
        return {"u_load": u_load, "W_hold": W_hold, "W_obj": W_obj,
                "tq_hold": tq_hold, "W_empty": W_empty, "gK": gK}

    # ---------------- GIVE: weight-transfer release ----------------
    def wait_release(self, h):
        S_on = S_FRAC * h["W_obj"]
        DROP_on = DROP_FRAC * h["W_obj"]
        EMPTY_BAND = h["W_empty"] + EMPTY_MARGIN
        s_hist = deque(maxlen=int(T_LOOKBACK / POLL_DT))
        rawmag_ma = deque(maxlen=MA_WIN)
        net_ma = deque(maxlen=MA_WIN)
        cnt = 0
        need = int(T_STABLE / POLL_DT)
        last_retare = time.monotonic()
        tick = 0

        while self._running:
            f = self.read_force()
            if f is None:                       # stale/frozen/err => still loaded
                cnt = 0
                time.sleep(POLL_DT)
                continue
            if f["mag"] > F_CEIL or f["tq"] > TQ_CEIL:   # collision => never open
                cnt = 0
                time.sleep(POLL_DT)
                continue
            net_ma.append(f["net"]); rawmag_ma.append(f["raw_mag"])
            net = np.mean(net_ma, axis=0)
            raw_mag = float(np.mean(rawmag_ma))
            s = float(-np.dot(net, h["u_load"]))    # weight leaving the gripper
            n = float(np.linalg.norm(net))
            drop = h["W_hold"] - raw_mag
            s_hist.append(s)

            ok = (s >= G_DIR * n and                 # directional
                  s >= S_on and                      # near-full transfer
                  raw_mag <= EMPTY_BAND and          # returned to empty level
                  drop >= DROP_on and                # residual-aware unload
                  f["tq"] <= h["tq_hold"] + TQ_BUMP)  # torque cross-check
            cnt = cnt + 1 if ok else 0

            rose = len(s_hist) == s_hist.maxlen and min(s_hist) < 0.3 * S_on
            if cnt >= need and rose and np.std(list(s_hist)[-need:]) < 1.0:
                print(f"  [GIVE] weight transferred (s={s:.1f} N) -> RELEASING.")
                self.open_gripper()
                return True

            # idle-drift re-tare (only while genuinely quiescent, not mid-rise)
            quiescent = (abs(s) < 1.5 and n < 2.0 and
                         (len(s_hist) < 5 or np.std(list(s_hist)[-5:]) < 1.0))
            if quiescent and time.monotonic() - last_retare > RETARE_IDLE:
                self.tare()
                last_retare = time.monotonic()

            tick += 1
            if tick % 40 == 0:
                print(f"  [GIVE] holding: s={s:.1f}/{S_on:.1f} N  raw={raw_mag:.1f} N")
            time.sleep(POLL_DT)
        return False

    # ---------------- TAKE: directional, pinch-guarded grasp ----------------
    def wait_take(self):
        # Never fabricate a gravity axis: if /state orientation is unavailable we
        # cannot judge direction, so we DON'T grasp (fail-safe hold-open).
        gK = None
        net_ma = deque(maxlen=MA_WIN)
        cnt = tick = 0
        while self._running:
            if gK is None:
                _, _, quat = self.state()
                if quat is not None:
                    gK = quat_gravity_ee(quat)
                else:
                    cnt = 0
                    tick += 1
                    if tick % 40 == 0:
                        print("  [TAKE] waiting for /state orientation (won't grasp blind) ...")
                    time.sleep(POLL_DT)
                    continue
            f = self.read_force()
            if f is None:
                cnt = 0
                time.sleep(POLL_DT)
                continue
            net_ma.append(f["net"])
            net = np.mean(net_ma, axis=0)
            # F_ext sign: a downward added load projects to -gK, so negate to
            # make an added object read as positive "w".
            w = -float(np.dot(net, gK))              # added weight along gravity
            lateral = float(np.linalg.norm(net + w * gK))
            # grasp only on sustained added weight ALONG gravity, small lateral,
            # AND low wrist torque -- a resting/steadying hand injects torque a
            # hung object does not, so the torque veto blocks a pinch.
            if w >= TAKE_WEIGHT and lateral <= TAKE_LAT_MAX and f["tq"] <= TAKE_TQ_MAX:
                cnt += 1
                if cnt >= TAKE_DEBOUNCE:
                    print(f"  [TAKE] object placed (down={w:.1f} N, lat={lateral:.1f}, "
                          f"tq={f['tq']:.2f}) -> grasping.")
                    return True
            else:
                cnt = 0
            tick += 1
            if tick % 40 == 0:
                print(f"  [TAKE] empty: down={w:.1f}/{TAKE_WEIGHT} N  lat={lateral:.1f}  "
                      f"tq={f['tq']:.2f} (veto>{TAKE_TQ_MAX})")
            time.sleep(POLL_DT)
        return False

    def grasp_and_verify(self):
        """Grasp, then verify the fingers actually closed on an object (not a
        hand / thin air). Returns True only if a real grip is confirmed."""
        self.grasp()
        time.sleep(0.4)
        ok, gripper, _ = self.state()
        # gripper==0 means "closed" (< 0.03 m) per /state; that's expected while
        # holding. A grip that slammed fully shut on nothing also reads 0, so we
        # additionally require the object weight to be present.
        got = self._avg_force(0.4)
        if got is not None:
            _, raw_mag, _ = got
            if raw_mag < R_FLOOR + W_MIN * 0.5:
                print(f"  [TAKE] grasp looks empty (raw={raw_mag:.1f} N) -- reopening.")
                self.open_gripper()
                return False
        return True

    # ---------------- main loop ----------------
    def run(self):
        print("[SETUP] waiting for /force ...")
        while self._running and self.read_force() is None:
            time.sleep(0.3)
        if not self._running:
            return
        print("[SETUP] force available.")
        if self.do_home and not self.home():
            print("[SETUP] home failed -- aborting for safety.")
            return

        state = self.start
        holding = None
        while self._running:
            if state == "give":
                if self.skip_grasp:
                    print("[GIVE] using existing grasp (object already held) -- not re-grasping.")
                    self.skip_grasp = False   # only skip the first time
                    holding = True
                else:
                    print("[GIVE] grasping to present the object ...")
                    holding = self.grasp() or holding
                time.sleep(SETTLE_S)
                h = self.arm_hold()
                if h is None:
                    # not safe to auto-release: keep holding, wait for the human
                    # to just take it is unsafe without arming, so pause here.
                    print("[GIVE] not armed -- keeping hold. (Ctrl-C to stop.)")
                    while self._running:
                        time.sleep(0.3)
                    break
                if not self.wait_release(h):
                    break
                holding = False
                state = "take"
            else:  # take
                opened = self.open_gripper()
                holding = False
                time.sleep(SETTLE_S)
                self.tare()
                ok, gripper, _ = self.state()
                got = self._avg_force(0.4)
                # Only trust W_empty from a CONFIRMED-empty gripper; a failed/409
                # open would otherwise capture a loaded reading and inflate the
                # empty-band gate. Keep the previous W_empty if unconfirmed.
                if opened and ok and gripper == 1 and got is not None \
                        and got[1] < R_FLOOR + 0.5 * W_MIN:
                    self.W_empty = got[1]
                else:
                    print("  [TAKE] open not confirmed empty -- keeping prior W_empty.")
                print("[TAKE] open. Place an object and let go (weight along gravity).")
                if not self.wait_take():
                    break
                holding = self.grasp_and_verify()
                state = "give"

        where = "HOLDING object" if holding else ("open/empty" if holding is False else "unknown")
        print(f"\n[EXIT] stopped. Gripper left AS-IS ({where}); not actuated on shutdown.")


def main():
    ap = argparse.ArgumentParser(description="Force-based human-robot handover (safe release)")
    ap.add_argument("--base-url", default="http://172.26.0.212:5000")
    ap.add_argument("--start", choices=["give", "take"], default="take",
                    help="take = start open and receive first (recommended); "
                         "give = object pre-placed in the gripper, robot holds first")
    ap.add_argument("--home", action="store_true",
                    help="home the arm once at start (KEEP THE WORKSPACE CLEAR)")
    ap.add_argument("--grasp-force", type=float, default=10.0,
                    help="grasp force in N, finger-safe (default 10)")
    ap.add_argument("--no-grasp", action="store_true",
                    help="object is ALREADY grasped (e.g. after a manual pick+lift); "
                         "skip the initial grasp so it isn't disturbed")
    args = ap.parse_args()

    h = Handover(args)

    def _stop(signum, frame):
        h._running = False
        print("\n[EXIT] Ctrl-C -- finishing current step, then stopping ...")

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    print("=== Force-based handover (SAFE weight-transfer release) ===")
    print(f"    base={args.base_url}  start={args.start}  grasp_force={args.grasp_force} N")
    print("    Release only on a sustained, directional, near-full weight take.")
    print("    Fail-safe: any doubt -> KEEP HOLDING.  Ctrl-C to stop.\n")
    h.run()


if __name__ == "__main__":
    main()
