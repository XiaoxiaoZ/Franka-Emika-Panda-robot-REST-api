"""End-effector force/torque visualization for RViz.

Subscribes to the Franka external-wrench estimate and republishes it as an
RViz MarkerArray so you can see, e.g., the weight of a grasped object and --
for a long/off-center object -- the moment it exerts on the wrist:

  * a FORCE arrow (green->red by magnitude) + "x.x N" label, and
  * a TORQUE arrow (blue->magenta) + "x.xx N.m" label.

Data source
-----------
``/franka_state_controller/F_ext`` is a ``geometry_msgs/WrenchStamped`` carrying
``K_F_ext_hat_K`` -- the estimated *external* wrench acting on the robot,
expressed in the ``panda_K`` (stiffness / end-effector) frame. Because the
force/torque components are already in ``panda_K`` and that frame is in the TF
tree, the arrows are published directly in ``panda_K``: each starts at the frame
origin (the EE) and its tip is the (force|torque) vector times a scale. RViz
transforms them to the fixed frame via TF, so no manual frame math is needed.
The torque arrow points along the moment axis (right-hand rule).

Baseline / tare
---------------
The estimate carries a non-finite residual (a few N, a few tenths of N.m) that
also drifts with arm configuration. ``tare()`` snapshots the current raw force
*and* torque as baselines that are subtracted from the reported/displayed *net*
wrench; ``untare()`` clears them. Typical use: tare with an empty gripper, then
grasp an object to see its weight (and, for a long object, its moment) alone.
Because the residual is pose-dependent, re-tare after large pose changes.
"""

import math
import threading

import rospy
from geometry_msgs.msg import WrenchStamped, Point, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

ZERO_BASELINE = (0.0, 0.0, 0.0)

# Arm joints, in MoveIt group order, used to match the current configuration
# against the calibration table for pose-dependent residual compensation.
_ARM_JOINTS = ["panda_joint%d" % i for i in range(1, 8)]

# Label colors (arrow colors are computed per-magnitude below).
_FORCE_LABEL = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)   # white
_TORQUE_LABEL = ColorRGBA(r=0.5, g=0.8, b=1.0, a=0.95)  # light blue

# Marker ids within the namespace.
_ID_FORCE_ARROW = 0
_ID_FORCE_TEXT = 1
_ID_TORQUE_ARROW = 2
_ID_TORQUE_TEXT = 3


def _require(value, name, positive):
    """Coerce ``value`` to a finite float, raising ValueError on bad input:
    NaN/inf is always rejected; ``positive`` requires > 0, otherwise >= 0."""
    f = float(value)
    if not math.isfinite(f):
        raise ValueError("%s must be a finite number" % name)
    if positive and f <= 0.0:
        raise ValueError("%s must be > 0" % name)
    if not positive and f < 0.0:
        raise ValueError("%s must be >= 0" % name)
    return f


class ForceVisualizer:
    """Subscribe to F_ext and publish force + torque arrows for RViz."""

    def __init__(self,
                 wrench_topic="/franka_state_controller/F_ext",
                 marker_topic="/franka_ee_force",
                 scale=0.02,            # force arrow length: meters per Newton
                 threshold=0.5,         # N; below this the force arrow is hidden
                 max_force=20.0,        # N at which the force color saturates
                 torque_scale=0.15,     # torque arrow length: meters per N.m
                 torque_threshold=0.05,  # N.m; below this the torque arrow hides
                 max_torque=2.0,        # N.m at which the torque color saturates
                 publish_rate=20.0,     # Hz at which markers are republished
                 joint_topic="/franka_state_controller/joint_states",
                 ns="ee_force"):
        self.wrench_topic = wrench_topic
        self.scale = float(scale)
        self.threshold = float(threshold)
        self.max_force = float(max_force)
        self.torque_scale = float(torque_scale)
        self.torque_threshold = float(torque_threshold)
        self.max_torque = float(max_torque)
        self.ns = ns

        self._lock = threading.Lock()
        self._latest = None                  # tuple set by _on_wrench (below)
        self._baseline_force = ZERO_BASELINE  # tared force offset (panda_K)
        self._baseline_torque = ZERO_BASELINE  # tared torque offset (panda_K)
        # Pose-dependent residual compensation: when enabled, the baseline is the
        # residual of the nearest calibrated joint config instead of the tare.
        self._joints = None                  # latest arm joint values (group order)
        self._calibration = None             # [{config:[7], force:[3], torque:[3]}]
        self._compensate = False

        self._marker_pub = rospy.Publisher(marker_topic, MarkerArray, queue_size=1)
        self._sub = rospy.Subscriber(
            wrench_topic, WrenchStamped, self._on_wrench, queue_size=1)
        self._joint_sub = rospy.Subscriber(
            joint_topic, JointState, self._on_joints, queue_size=1)
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / publish_rate), self._on_timer)
        rospy.loginfo("ForceVisualizer: %s -> %s (force %.3f m/N, torque %.3f m/N.m)",
                      wrench_topic, marker_topic, self.scale, self.torque_scale)

    # ------------------------------------------------------------------ #
    # subscriber
    # ------------------------------------------------------------------ #
    def _on_wrench(self, msg):
        with self._lock:
            self._latest = (
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
                msg.header.frame_id or "panda_K",
                msg.header.stamp,
            )

    def _on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        try:
            j = [pos[n] for n in _ARM_JOINTS]
        except KeyError:
            return  # not the arm joint_states (e.g. gripper-only message)
        with self._lock:
            self._joints = j

    # ------------------------------------------------------------------ #
    # residual compensation (pose-dependent baseline)
    # ------------------------------------------------------------------ #
    def set_calibration(self, entries):
        """Install a pose-dependent residual baseline. ``entries`` is a list of
        {config:[7], force:[3], torque:[3]} from an empty-gripper calibration
        pass. Pass None/[] to clear (which also disables compensation)."""
        entries = list(entries) if entries else None
        with self._lock:
            self._calibration = entries
            if not entries:
                self._compensate = False
        return len(entries) if entries else 0

    def set_compensation(self, on):
        """Enable/disable pose-dependent residual compensation. Returns False if
        enabling without a calibration installed."""
        with self._lock:
            if on and not self._calibration:
                return False
            self._compensate = bool(on)
            return True

    def _read(self):
        """Snapshot the latest wrench and the EFFECTIVE baseline to subtract.
        When compensation is on and a calibration + joint state exist, the
        baseline is the residual of the nearest calibrated config; otherwise the
        tare baselines. Returns (latest, fbase, tbase, source)."""
        with self._lock:
            latest = self._latest
            joints = self._joints
            calib = self._calibration
            comp = self._compensate
            bf = self._baseline_force
            bt = self._baseline_torque
        if latest is None:
            return None, ZERO_BASELINE, ZERO_BASELINE, "none"
        if comp and calib and joints is not None:
            best, best_d = None, None
            for e in calib:
                cfg = e["config"]
                d = sum((joints[i] - cfg[i]) ** 2
                        for i in range(min(len(joints), len(cfg))))
                if best_d is None or d < best_d:
                    best_d, best = d, e
            if best is not None:
                return latest, tuple(best["force"]), tuple(best["torque"]), "calibration"
        return latest, bf, bt, "tare"

    # ------------------------------------------------------------------ #
    # public API (used by the REST endpoints)
    # ------------------------------------------------------------------ #
    def get_force(self):
        """Latest wrench as a dict: raw + net force and torque, magnitudes,
        lever arm (|net torque| / |net force|), frame, and which baseline was
        subtracted (``baseline_source``: 'calibration' | 'tare' | 'none').

        With residual compensation on, ``net_*`` is the true EXTERNAL wrench
        (object weight + any contact) at the current pose, not just at a tare
        pose."""
        latest, bf, bt, source = self._read()
        if latest is None:
            return {"available": False,
                    "msg": "no wrench received yet on %s" % self.wrench_topic}
        fx, fy, fz, tx, ty, tz, frame_id, stamp = latest
        nfx, nfy, nfz = fx - bf[0], fy - bf[1], fz - bf[2]
        ntx, nty, ntz = tx - bt[0], ty - bt[1], tz - bt[2]
        fmag = math.sqrt(nfx * nfx + nfy * nfy + nfz * nfz)
        tmag = math.sqrt(ntx * ntx + nty * nty + ntz * ntz)
        lever = (tmag / fmag) if fmag > 1e-6 else None
        return {
            "available": True,
            "frame_id": frame_id,
            "raw_force": {"x": fx, "y": fy, "z": fz},
            "net_force": {"x": nfx, "y": nfy, "z": nfz},
            "baseline_force": {"x": bf[0], "y": bf[1], "z": bf[2]},
            "magnitude": fmag,
            "raw_magnitude": math.sqrt(fx * fx + fy * fy + fz * fz),
            "raw_torque": {"x": tx, "y": ty, "z": tz},
            "net_torque": {"x": ntx, "y": nty, "z": ntz},
            "baseline_torque": {"x": bt[0], "y": bt[1], "z": bt[2]},
            "torque_magnitude": tmag,
            "raw_torque_magnitude": math.sqrt(tx * tx + ty * ty + tz * tz),
            "lever_arm": lever,  # meters; ~distance of CoM/contact from the EE
            "stamp": stamp.to_sec() if stamp else None,
            "baseline_source": source,
            "compensated": source == "calibration",
            "tared": source == "tare" and (bf != ZERO_BASELINE or bt != ZERO_BASELINE),
        }

    def tare(self):
        """Snapshot the current raw force and torque as baselines. Returns them,
        or None if no wrench has been received yet."""
        with self._lock:
            if self._latest is None:
                return None
            fx, fy, fz, tx, ty, tz = self._latest[0:6]
            self._baseline_force = (fx, fy, fz)
            self._baseline_torque = (tx, ty, tz)
        return {"force": {"x": fx, "y": fy, "z": fz},
                "torque": {"x": tx, "y": ty, "z": tz}}

    def untare(self):
        """Clear the baselines so the raw wrench is shown again."""
        with self._lock:
            self._baseline_force = ZERO_BASELINE
            self._baseline_torque = ZERO_BASELINE

    def set_config(self, scale=None, threshold=None, max_force=None,
                   torque_scale=None, torque_threshold=None, max_torque=None):
        """Update visualization parameters; ignores None args. Rejects
        non-finite or out-of-range values with ValueError (scales and max_*
        must be > 0; thresholds must be >= 0)."""
        if scale is not None:
            self.scale = _require(scale, "scale", positive=True)
        if threshold is not None:
            self.threshold = _require(threshold, "threshold", positive=False)
        if max_force is not None:
            self.max_force = _require(max_force, "max_force", positive=True)
        if torque_scale is not None:
            self.torque_scale = _require(torque_scale, "torque_scale", positive=True)
        if torque_threshold is not None:
            self.torque_threshold = _require(torque_threshold, "torque_threshold", positive=False)
        if max_torque is not None:
            self.max_torque = _require(max_torque, "max_torque", positive=True)
        return {"scale": self.scale, "threshold": self.threshold,
                "max_force": self.max_force, "torque_scale": self.torque_scale,
                "torque_threshold": self.torque_threshold,
                "max_torque": self.max_torque}

    # ------------------------------------------------------------------ #
    # marker construction
    # ------------------------------------------------------------------ #
    @staticmethod
    def _ramp_color(mag, max_mag, low, high):
        """Linear color ramp from ``low`` to ``high`` RGB, saturating at
        ``max_mag``. low/high are (r, g, b) tuples."""
        t = max(0.0, min(1.0, mag / max_mag)) if max_mag > 0 else 0.0
        return ColorRGBA(r=low[0] + (high[0] - low[0]) * t,
                         g=low[1] + (high[1] - low[1]) * t,
                         b=low[2] + (high[2] - low[2]) * t, a=0.9)

    def _arrow(self, mid, frame_id, now, tip, color):
        """Two-point ARROW from the frame origin to ``tip`` (a 3-tuple). Keeps
        the head shorter than the total length so small vectors don't invert
        the arrow; diameters scale with length but are clamped for visibility."""
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = now
        m.ns = self.ns
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.lifetime = rospy.Duration(0.5)
        m.pose.orientation.w = 1.0  # identity; avoids RViz zero-quaternion warning
        m.points = [Point(0.0, 0.0, 0.0), Point(*tip)]
        length = math.sqrt(tip[0] * tip[0] + tip[1] * tip[1] + tip[2] * tip[2])
        head_len = min(0.04, 0.3 * length)
        shaft_dia = min(0.012, max(0.004, 0.12 * length))
        m.scale = Vector3(shaft_dia, 2.0 * shaft_dia, head_len)
        m.color = color
        return m

    def _text(self, mid, frame_id, now, pos, text, color):
        """TEXT_VIEW_FACING label at ``pos`` (a 3-tuple)."""
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = now
        m.ns = self.ns
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.lifetime = rospy.Duration(0.5)
        m.pose.orientation.w = 1.0
        m.pose.position = Point(*pos)
        m.scale.z = 0.04  # text height in meters
        m.color = color
        m.text = text
        return m

    def _delete(self, mid, frame_id, now):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = now
        m.ns = self.ns
        m.id = mid
        m.action = Marker.DELETE
        return m

    def _vector_markers(self, arrow_id, text_id, frame_id, now, vec, mag,
                        scale, threshold, color, label):
        """Build [arrow, text] for a wrench component, or [delete, delete] when
        below threshold. ``vec`` is the (already net) 3-tuple; ``color`` is the
        arrow ColorRGBA; ``label`` is the text-color ColorRGBA."""
        if mag < threshold:
            return [self._delete(arrow_id, frame_id, now),
                    self._delete(text_id, frame_id, now)]
        tip = (vec[0] * scale, vec[1] * scale, vec[2] * scale)
        text_pos = (tip[0], tip[1], tip[2] + 0.04)
        return [self._arrow(arrow_id, frame_id, now, tip, color),
                self._text(text_id, frame_id, now, text_pos, label[1], label[0])]

    def _on_timer(self, _evt):
        latest, bf, bt, _source = self._read()
        if latest is None:
            return
        fx, fy, fz, tx, ty, tz = latest[0:6]
        frame_id = latest[6]
        if not all(math.isfinite(v) for v in (fx, fy, fz, tx, ty, tz)):
            return  # skip a fault frame rather than publish NaN marker coords
        now = rospy.Time.now()

        nf = (fx - bf[0], fy - bf[1], fz - bf[2])
        nt = (tx - bt[0], ty - bt[1], tz - bt[2])
        fmag = math.sqrt(nf[0] * nf[0] + nf[1] * nf[1] + nf[2] * nf[2])
        tmag = math.sqrt(nt[0] * nt[0] + nt[1] * nt[1] + nt[2] * nt[2])

        markers = []
        markers += self._vector_markers(
            _ID_FORCE_ARROW, _ID_FORCE_TEXT, frame_id, now, nf, fmag,
            self.scale, self.threshold,
            self._ramp_color(fmag, self.max_force, (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)),
            (_FORCE_LABEL, "%.1f N" % fmag))
        markers += self._vector_markers(
            _ID_TORQUE_ARROW, _ID_TORQUE_TEXT, frame_id, now, nt, tmag,
            self.torque_scale, self.torque_threshold,
            self._ramp_color(tmag, self.max_torque, (0.0, 0.3, 1.0), (1.0, 0.0, 1.0)),
            (_TORQUE_LABEL, u"%.2f N·m" % tmag))

        arr = MarkerArray()
        arr.markers = markers
        self._marker_pub.publish(arr)
