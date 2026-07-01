"""Multi-pose payload identification for a grasped object.

Estimates the mass (and a rough center-of-mass offset) of a grasped object from
the Franka external-wrench estimate ``F_ext``, robust to its pose-dependent
residual.

Why two passes
--------------
``F_ext`` at a static pose ≈ residual(joint config) + object's gravity wrench.
The residual drifts with arm configuration (gravity-model error + joint
friction), so a single empty-gripper tare does not cancel it at other poses.

So we measure raw ``F_ext`` at N static joint configurations EMPTY, then at the
SAME configurations while HOLDING the object. Per pose:

    ΔF_i = F_loaded_i − F_empty_i  ≈  object gravity wrench in the EE frame

The residual cancels (same joint config in both passes; replaying the identical
config sequence also keeps the friction/stiction state similar). Gravity is a
constant ``m·g`` in the world frame and rotation preserves magnitude, so

    |ΔF_i| = m·g   at every pose      →   m = mean_i |ΔF_i| / g

The CoM offset ``r`` in the EE frame is fit from ``Δτ_i = r × ΔF_i`` by least
squares (needs ≥2 poses with non-parallel ΔF, which the wrist-reorienting
perturbations provide). The CoM estimate is rough — torque residuals are large
relative to the small lever-arm signal — so treat it as indicative.
"""

import math
import time

import numpy as np

G = 9.80665

# Joint perturbations (rad) added to the captured base config to form the
# measurement poses. WRIST-ONLY (joints 5 and 6, 0-based indices 4 and 5), kept
# small to limit how far a grasped object swings -- IMPORTANT: MoveIt does NOT
# model the held object's geometry, so its collision check covers only the arm
# links. These reorient the EE for gravity-projection diversity but do NOT keep
# it perfectly in place (joint5 also translates the TCP a few cm); the operator
# must ensure clearance around the object before the loaded pass.
PERTURBATIONS = [
    [0, 0, 0, 0,  0.0,  0.0, 0],   # base (as captured)
    [0, 0, 0, 0,  0.0,  0.25, 0],  # joint6 (wrist flex) +
    [0, 0, 0, 0,  0.0, -0.25, 0],  # joint6 (wrist flex) -
    [0, 0, 0, 0,  0.2,  0.0, 0],   # joint5 +
    [0, 0, 0, 0, -0.2,  0.0, 0],   # joint5 -
]


class PayloadIdentifier:
    """Runs the empty/loaded measurement passes and computes mass + CoM.

    Motion is delegated to ``robot.move_to_joint_config``; the caller is
    responsible for holding the motion lock around record_empty/record_loaded.
    ``force_sampler`` is a callable returning the dict from
    ForceVisualizer.get_force() (raw_force / raw_torque are used)."""

    def __init__(self, robot, force_sampler, settle_s=1.0, n_samples=20, sample_dt=0.05):
        self.robot = robot
        self._sample_force = force_sampler
        self.settle_s = settle_s
        self.n_samples = n_samples
        self.sample_dt = sample_dt
        self._configs = None   # 7-vectors, captured during the empty pass
        self._empty = None
        self._loaded = None

    # ------------------------------------------------------------------ #
    def _sample(self):
        """Average raw force/torque over a short static window. Non-finite
        (NaN/inf) fault frames are skipped so one bad frame can't poison the
        averaged wrench (mirrors the isfinite guard in ForceVisualizer)."""
        fs, ts = [], []
        for _ in range(self.n_samples):
            d = self._sample_force()
            if d.get("available"):
                f, t = d["raw_force"], d["raw_torque"]
                fv = [f["x"], f["y"], f["z"]]
                tv = [t["x"], t["y"], t["z"]]
                if all(math.isfinite(v) for v in fv + tv):
                    fs.append(fv)
                    ts.append(tv)
            time.sleep(self.sample_dt)
        if not fs:
            return None
        return {"force": np.mean(fs, axis=0).tolist(),
                "torque": np.mean(ts, axis=0).tolist(),
                "force_std": float(np.linalg.norm(np.std(fs, axis=0))),
                "samples": len(fs)}

    def _run_pass(self, configs):
        """Visit each config, settle, sample. Honors the robot user-stop flag:
        if /control/stop fires mid-pass, bail out immediately (no further poses,
        no return-to-base). Returns (out, returned_to_base)."""
        out = []
        robot = self.robot
        robot.clear_stop()  # clear any stale stop so this pass can start
        stopped = False
        for idx, cfg in enumerate(configs):
            if robot.stop_requested():
                out.append({"pose": idx, "moved": False, "stopped": True})
                stopped = True
                break
            res = robot.move_to_joint_config(cfg)
            if not res.get("executed"):
                out.append({"pose": idx, "moved": False,
                            "stopped": bool(res.get("stopped")),
                            "plan_error": res.get("plan_error_codes")})
                if res.get("stopped") or robot.stop_requested():
                    stopped = True
                    break
                continue
            time.sleep(self.settle_s)  # settle to a static measurement
            entry = {"pose": idx, "moved": True}
            s = self._sample()
            if s:
                entry.update(s)
            out.append(entry)
        # Return to base so the next pass / the user starts from a known pose --
        # but NOT if the pass was stopped (respect the user-stop).
        returned = False
        if configs and not stopped and not robot.stop_requested():
            returned = bool(robot.move_to_joint_config(configs[0]).get("executed"))
        return out, returned

    # ------------------------------------------------------------------ #
    def record_empty(self):
        """Capture the current config as base, build the measurement configs,
        and run the empty-gripper pass."""
        base = list(self.robot.move_group.get_current_joint_values())
        # Invalidate stale data BEFORE the (possibly-failing) pass so a mid-pass
        # exception can't leave old _empty paired with new _configs.
        self._empty = None
        self._loaded = None
        self._configs = [[base[i] + p[i] for i in range(len(base))]
                         for p in PERTURBATIONS]
        out, returned = self._run_pass(self._configs)
        self._empty = out
        return {"base_config": [round(v, 5) for v in base],
                "n_poses": len(self._configs),
                "moved": sum(1 for e in out if e.get("moved")),
                "returned_to_base": returned,
                "stopped": any(e.get("stopped") for e in out),
                "empty": out}

    def record_loaded(self):
        """Run the loaded pass over the SAME configs captured in record_empty."""
        configs = self._configs
        if configs is None:
            raise ValueError("run the empty pass first (/force/identify/empty)")
        out, returned = self._run_pass(configs)
        self._loaded = out
        return {"n_poses": len(configs),
                "moved": sum(1 for l in out if l.get("moved")),
                "returned_to_base": returned,
                "stopped": any(l.get("stopped") for l in out),
                "loaded": out}

    def compute(self):
        """ΔF per pose → mass; Δτ = r×ΔF least-squares → CoM offset."""
        empty, loaded = self._empty, self._loaded  # snapshot (reset() is lock-free)
        if empty is None or loaded is None:
            raise ValueError("need both empty and loaded passes")
        weights, A, b, per = [], [], [], []
        for e, l in zip(empty, loaded):
            if not (e.get("moved") and l.get("moved")
                    and "force" in e and "force" in l):
                continue
            df = np.array(l["force"]) - np.array(e["force"])
            dt = np.array(l["torque"]) - np.array(e["torque"])
            mag = float(np.linalg.norm(df))
            weights.append(mag)
            per.append({"pose": e["pose"], "dF": df.tolist(),
                        "dF_mag_N": mag, "mass_kg": mag / G})
            fx, fy, fz = df  # skew(df) @ r = -dt  (since dt = r×df = -skew(df) r)
            A.append([[0.0, -fz, fy], [fz, 0.0, -fx], [-fy, fx, 0.0]])
            b.append((-dt).tolist())
        if not weights:
            raise ValueError("no valid pose pairs (did the arm move and sample?)")
        masses = np.array(weights) / G
        result = {
            "mass_kg": float(np.median(masses)),
            "mass_mean_kg": float(np.mean(masses)),
            "mass_std_kg": float(np.std(masses)),
            "weight_N": float(np.median(weights)),
            "n_pose_pairs": len(weights),
            "per_pose": per,
        }
        if len(A) >= 2:
            r, _res, _rank, _sv = np.linalg.lstsq(
                np.vstack(A), np.concatenate(b), rcond=None)
            result["com_offset_K_m"] = r.tolist()
            result["com_distance_m"] = float(np.linalg.norm(r))
        return result

    def calibration(self):
        """Return the empty-pass residuals as [{config, force, torque}] for use
        as a pose-dependent baseline (residual compensation). Empty list if no
        empty pass has run or no pose moved."""
        if self._empty is None or self._configs is None:
            return []
        out = []
        for e in self._empty:
            if e.get("moved") and "force" in e and "torque" in e:
                out.append({"config": list(self._configs[e["pose"]]),
                            "force": e["force"], "torque": e["torque"]})
        return out

    def reset(self):
        self._configs = self._empty = self._loaded = None
