# Using flask to make an api
# import necessary libraries and functions
import logging
import math
import os
import signal
import time
from flask import Flask, Response, jsonify, request
from threading import Lock

# Log
from franka import MoveGroupPythonInterfaceTutorial, decode_moveit_error, check_joint_limits, is_user_stop_abort, PANDA_JOINT_LIMITS
from sensor_msgs.msg import JointState
from force_viz import ForceVisualizer
from files_api import bp as files_bp, store as file_store
import scene_api
import scene_v2_api
import plan_api
import robot_api
from scene_manager import SceneManager
from scene_core import SceneCore, MoveItSceneIO
from payload_id import PayloadIdentifier
from realsense_cam import RealsenseCamera
import rospy
from rosgraph_msgs.msg import Log

import actionlib
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(name)s: %(message)s',
)
log = logging.getLogger('franka_restful')

last_error = []
def rosout_cd(msg):
    global last_error
    if msg.level>=8:
        last_error.append(f"[{msg.header.stamp.secs}] {msg.name}: {msg.msg}")
rospy.Subscriber("/rosout", Log, rosout_cd)

# creating a Flask app
app = Flask(__name__)
# /files: sandboxed HTTP file store (CAD models pushed by the detector etc.)
app.register_blueprint(files_bp)
# /scene: CAD models + objects in the planning scene (wired up after the
# robot interface exists, below).
app.register_blueprint(scene_api.bp)
# /scene/info, /scene/batch, /scene/namespaces: namespaced, versioned v2 scene.
app.register_blueprint(scene_v2_api.bp)
# /plan/check_state, /plan/dry_run: plan-only checks, never move the robot.
app.register_blueprint(plan_api.bp)
# /robot/tf, /robot/description: read-only model facts for frame checks.
app.register_blueprint(robot_api.bp)

# Flask debug mode (and its auto-reloader). The reloader re-executes this
# module in a child process, so module-level singletons are built twice unless
# guarded -- see the force_viz guard below. OFF by default (safety rule S5: the
# Werkzeug debugger is a remote-execution surface and the reloader runs the
# startup -- scene restore, walls -- twice); FRANKA_RESTFUL_DEBUG=1 to enable.
DEBUG = os.environ.get('FRANKA_RESTFUL_DEBUG', '0') == '1'

@app.errorhandler(Exception)
def _on_unhandled_exception(e):
    log.exception("Unhandled exception in handler")
    return jsonify({"error": type(e).__name__, "message": str(e)}), 500

# robot instance
robot = MoveGroupPythonInterfaceTutorial()

# Thread lock
moveit_lock = Lock()

# ----- Franka gripper action clients -----
gripper_move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
gripper_grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)

rospy.loginfo("Waiting for franka_gripper action servers...")
if not gripper_move_client.wait_for_server(rospy.Duration(10.0)):
    rospy.logerr("franka_gripper/move action server not available after 10s")
if not gripper_grasp_client.wait_for_server(rospy.Duration(10.0)):
    rospy.logerr("franka_gripper/grasp action server not available after 10s")
rospy.loginfo("franka_gripper action servers are ready (or timed out).")
robot.add_floor()
# Safety walls: fence the arm into the working quadrant (people stand in the
# other regions). See MoveGroupPythonInterfaceTutorial.add_virtual_walls.
robot.add_virtual_walls()

# CAD models / detected objects in the planning scene. State lives next to
# the /files store and is re-applied here because move_group forgets its
# world on restart.
# Scene state lives OUTSIDE the /files store (safety rule S6: anyone could
# otherwise rewrite it through PUT /files/scene.json).
STATE_DIR = os.environ.get("FRANKA_STATE_DIR",
                           os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "state"))
os.makedirs(STATE_DIR, exist_ok=True)
_legacy_state = os.path.join(STATE_DIR, "scene.json")
_old_state = os.path.join(file_store.root, "scene.json")
if os.path.isfile(_old_state) and not os.path.exists(_legacy_state):
    os.replace(_old_state, _legacy_state)          # one-time migration out of /files
    rospy.logwarn("moved %s -> %s", _old_state, _legacy_state)

scene_manager = SceneManager(robot.scene, file_store, _legacy_state,
                             planning_frame=robot.move_group.get_planning_frame(),
                             logger=rospy.logwarn)
scene_api.init(scene_manager, moveit_lock)
_restored = scene_manager.restore()
rospy.loginfo("scene restored: %d models, %d objects%s", _restored["models"], _restored["objects"],
              (" -- errors: %s" % _restored["errors"]) if _restored["errors"] else "")

scene_core = SceneCore(MoveItSceneIO(), file_store, os.path.join(STATE_DIR, "scene_v2.json"),
                       planning_frame=robot.move_group.get_planning_frame(),
                       logger=rospy.logwarn, legacy=scene_manager)
scene_manager.on_change = scene_core.bump_for_legacy_change
scene_manager.is_reserved = lambda name: name in scene_core.objects
# legacy attach/detach also go through one confirmed diff (MoveIt-version independent)
scene_manager.attach_fn = scene_core.io_attach_existing
scene_manager.detach_fn = scene_core.io_detach_existing
scene_core.robot_links = robot.robot.get_link_names
_restored_v2 = scene_core.load_and_restore()
rospy.loginfo("scene v2 restored: %s", _restored_v2)


def _urdf_robot_name():
    import xml.etree.ElementTree as ET
    try:
        return ET.fromstring(rospy.get_param("/robot_description")).get("name")
    except Exception:
        return None


def _robot_info():
    """Static-ish robot facts for GET /scene/info."""
    import hashlib
    import rospkg
    pf = robot.move_group.get_planning_frame()
    def _sha(param):
        try:
            return hashlib.sha256(rospy.get_param(param).encode("utf-8")).hexdigest()
        except Exception:
            return None
    try:
        moveit_version = rospkg.RosPack().get_manifest("moveit_commander").version
    except Exception:
        moveit_version = None
    info = {
        "moveit_version": moveit_version,
        "planning_frame": pf,
        "robot_name": _urdf_robot_name(),
        "urdf_sha256": _sha("/robot_description"),
        "srdf_sha256": _sha("/robot_description_semantic"),
        "groups": {"arm": robot.move_group.get_name(), "hand": robot.hand_group.get_name()},
        "ee_link": robot.move_group.get_end_effector_link(),
    }
    if pf == "panda_link0":
        info["planning_frame_to_panda_link0"] = {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}
    else:
        info["planning_frame_to_panda_link0"] = None
        info["warning"] = "planning frame is not panda_link0; only panda_link0 frame_ids are accepted"
    return info


# The analytic IK must use the same flange -> TCP distance as the URDF (this
# robot's hand sits on a 20 mm adapter): read it instead of assuming 0.1034.
def _flange_to_tcp_from_urdf():
    import xml.etree.ElementTree as ET
    u = ET.fromstring(rospy.get_param("/robot_description"))
    z = 0.0
    for j in u.findall("joint"):
        if j.get("name") in ("panda_hand_joint", "panda_hand_tcp_joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz") if o is not None else "0 0 0").split()]
            z += xyz[2]
    return z


try:
    import panda_ik
    panda_ik.set_flange_to_tcp(_flange_to_tcp_from_urdf())
    rospy.loginfo("analytic IK: flange -> TCP = %.4f m (from URDF)", panda_ik.flange_to_tcp())
except Exception as _e:
    rospy.logerr("could not read flange -> TCP from the URDF (%s); analytic IK keeps %.4f", _e,
                 __import__("panda_ik").flange_to_tcp())

scene_v2_api.init(scene_core, moveit_lock, _robot_info)
plan_api.init(robot, scene_core, moveit_lock)
robot_api.init(robot)

# Wrist camera (Intel RealSense D435 + its 3D-printed mount) attached to the
# hand, so plan-only checks represent the real robot. Geometry is given in
# pipeFit's NOMINAL TCP frame = panda_link8 * Rz(-45 deg) * z 0.1034 -- the
# frame the hand-eye calibration was made in -- and anchored to the flange, so
# it stays right regardless of the hand adapter (the URDF TCP is 20 mm lower):
# - camera: Intel's own D435 collision box (realsense2_description,
#   25.05 x 90 x 25 mm) + 2 mm per side, placed with pipeFit's hand-eye
#   calibration (colour optical centre (-0.031, -0.032, -0.035), optical axis
#   = TCP +z, image x = TCP +y) -> centre (-0.031, 0.0005, -0.0435);
# - mount: models/panda_realsense_D4xx.stl (mvp_grasp), convex hull, mm->m,
#   rotated Rz(+90 deg) (mount x -> TCP y, flat side y=0 on the hand face,
#   mount z -> TCP +z) with its wide end on the camera's back (TCP z -0.056),
#   as in the photo of 2026-10-12.
# Override the camera box with FRANKA_WRIST_CAMERA_BOX="sx,sy,sz,cx,cy,cz",
# drop the mount with FRANKA_WRIST_MOUNT=0, everything with FRANKA_WRIST_CAMERA=0.
_CAM_DEFAULT = (0.029, 0.094, 0.029, -0.031, 0.0005, -0.0435)
_MOUNT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "panda_realsense_D4xx.stl")
_MOUNT_POSE = {"position": [-0.0185, 0.0005, -0.094], "orientation": [0.0, 0.0, 0.7071067811865476, 0.7071067811865476]}
if os.environ.get("FRANKA_WRIST_CAMERA", "1") != "0":
    import hashlib as _hashlib
    import numpy as _np
    from scene_manager import _parse_mesh_isolated
    _cam_env = os.environ.get("FRANKA_WRIST_CAMERA_BOX")
    _cam = tuple(float(v) for v in _cam_env.split(",")) if _cam_env else _CAM_DEFAULT
    _shapes = [{"type": "box", "size": list(_cam[:3]),
                "pose": {"position": list(_cam[3:6]), "orientation": [0, 0, 0, 1]}}]
    _mount = False
    if os.environ.get("FRANKA_WRIST_MOUNT", "1") != "0" and os.path.isfile(_MOUNT_FILE):
        try:
            from scipy.spatial import ConvexHull
            _v, _f = _parse_mesh_isolated(_MOUNT_FILE)
            _v = _v * 0.001
            _h = ConvexHull(_v)
            _used = _np.unique(_h.simplices)
            _remap = {o: n for n, o in enumerate(_used)}
            _sha = "builtin-mount-" + _hashlib.sha256(open(_MOUNT_FILE, "rb").read()).hexdigest()[:16]
            scene_core._mesh_cache[_sha] = (_v[_used], _np.vectorize(_remap.get)(_h.simplices))
            _shapes.append({"type": "mesh", "sha256": _sha, "scale": 1.0, "file": "models/panda_realsense_D4xx.stl",
                            "pose": _MOUNT_POSE})
            _mount = True
        except Exception as _e:
            rospy.logerr("wrist camera mount mesh not loaded (%s); camera box only", _e)
    _NOMINAL_TCP = {"position": [0.0, 0.0, 0.1034], "orientation": [0.0, 0.0, -0.3826834323650898, 0.9238795325112867]}
    _cam_obj = {"id": "wrist_camera", "frame_id": "panda_link8", "pose": _NOMINAL_TCP, "shapes": _shapes}
    _cam_spec = {"id": "wrist_camera", "link": "panda_hand",
                 "frame": "pipeFit nominal TCP = panda_link8 * Rz(-45deg) * z 0.1034",
                 "size": list(_cam[:3]), "center": list(_cam[3:6]), "placeholder": False,
                 "source": "FRANKA_WRIST_CAMERA_BOX" if _cam_env else
                           "Intel D435 collision box (+2 mm) placed by pipeFit hand-eye calibration",
                 "bracket_modelled": _mount,
                 "mount": {"file": "models/panda_realsense_D4xx.stl", "convex_hull": True, "pose": _MOUNT_POSE} if _mount else None,
                 "touch_links": ["panda_hand", "panda_leftfinger", "panda_rightfinger", "panda_link7"]}
    if scene_core.io.attach_full(_cam_obj, _cam_spec["link"], _cam_spec["touch_links"], scene_core._mesh_cache):
        scene_core.builtin_attached = _cam_spec
        rospy.loginfo("wrist camera attached (%s%s)", _cam_spec["source"], " + mount" if _mount else "")
    else:
        rospy.logerr("wrist camera collision box could NOT be attached")

# The 20 mm adapter between the flange and the hand (the URDF moves the hand
# down by panda_hand_joint z; this is the adapter's own volume). Diameter is
# an ESTIMATE (80 mm); override with FRANKA_FLANGE_ADAPTER="radius,length" (m),
# disable with FRANKA_FLANGE_ADAPTER=0.
_ad_env = os.environ.get("FRANKA_FLANGE_ADAPTER", "")
if _ad_env != "0":
    try:
        _hand_z = _flange_to_tcp_from_urdf() - 0.1034
    except Exception:
        _hand_z = 0.0
    _ad_r, _ad_l = (tuple(float(v) for v in _ad_env.split(",")) if _ad_env else (0.04, _hand_z))
    if _ad_l > 1e-4:
        _ad_obj = {"id": "flange_adapter", "frame_id": "panda_link8",
                   "pose": {"position": [0, 0, _ad_l / 2], "orientation": [0, 0, 0, 1]},
                   "shapes": [{"type": "cylinder", "radius": _ad_r, "length": _ad_l,
                               "pose": {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}}]}
        if scene_core.io.attach_full(_ad_obj, "panda_hand", ["panda_hand", "panda_link7"], scene_core._mesh_cache):
            scene_core.builtin_extra.append({"id": "flange_adapter", "link": "panda_hand", "frame": "panda_link8",
                                             "radius": _ad_r, "length": _ad_l, "estimated_radius": not _ad_env})
            rospy.loginfo("flange adapter attached (r=%.3f, l=%.3f)", _ad_r, _ad_l)
        else:
            rospy.logerr("flange adapter could NOT be attached")


# Safety gate for every motion endpoint (rules S2 + S4): refuse while the
# scene is dirty (a commit could not be confirmed or rolled back) and when
# the caller's expect_scene_version is stale. /control/stop always passes;
# dry runs (dry_run=1) don't move, so they pass too.
@app.before_request
def _motion_scene_gate():
    path = request.path
    if not path.startswith("/control/") or path == "/control/stop":
        return None
    if request.args.get("dry_run", "0") not in ("0", "false", ""):
        return None
    if scene_core.dirty:
        return jsonify({"error": "scene_dirty", "msg": "the planning scene is in an unconfirmed state; "
                        "push a successful /scene/batch or DELETE /scene/namespaces/<ns> first",
                        "dirty_reason": scene_core.dirty_reason}), 409
    expect = request.args.get("expect_scene_version")
    if expect is not None:
        try:
            expect = int(expect)
        except ValueError:
            return jsonify({"error": "bad_request", "msg": "expect_scene_version must be an integer"}), 400
        if expect != scene_core.version:
            return jsonify({"error": "version_conflict", "scene_version": scene_core.version,
                            "msg": f"scene_version is {scene_core.version}, request expects {expect}"}), 409
    return None

# End-effector force visualization: subscribes to /franka_state_controller/F_ext
# and publishes an arrow+label MarkerArray to /franka_ee_force for RViz.
# Build it only in the process that serves requests: under the debug reloader
# the module is imported twice (supervisor + worker) and only the worker has
# WERKZEUG_RUN_MAIN set; without this guard RViz would see two publishers on
# /franka_ee_force (flicker, and tare/config appear not to take effect because
# the supervisor's stale instance keeps republishing). With the reloader off
# (DEBUG=0) the env var is unset and we build it normally.
if (not DEBUG) or os.environ.get('WERKZEUG_RUN_MAIN') == 'true':
    force_viz = ForceVisualizer()
    payload_identifier = PayloadIdentifier(robot, force_viz.get_force)
else:
    force_viz = None
    payload_identifier = None

# on the terminal type: curl http://127.0.0.1:5000/
# returns hello world when we use GET.
# returns the data that we send when we use POST.
@app.route('/', methods = ['GET', 'POST'])
def home():
    if(request.method == 'GET'):

        data = "Hello, this is Franka Emika Panda"
        return jsonify({'data': data, 'help': 'GET /help for a full API usage guide'})


# Short usage notes for routes whose view function has no docstring of its own
# (mostly thin lock wrappers). Routes WITH a docstring are self-documenting via
# /help introspection -- prefer writing a docstring over adding entries here.
_HELP_NOTES = {
    "/": "API banner. GET /help for this guide.",
    "/state": "Current EE pose (meters, base frame) + orientation quaternion + rpy_deg (same orientation as roll/pitch/yaw degrees, matching the motion endpoints' params) + gripper (1=open, 0=closed) + joints (7 arm joint angles in rad; GET /joints for velocity, torque and limits).",
    "/health": "Probe gripper action servers, motion-lock state, and last ROS error. Use when the API feels stuck.",
    "/recover": "Trigger Franka automatic error recovery (clears reflex errors). 504 if not done within 15 s.",
    "/control/stop": "Stop any ongoing motion and suppress its auto-recovery retry (in-flight request returns outcome=stopped).",
    "/control/home": "Joint-space move to the canonical safe pose. WARNING: moves the arm.",
    "/control/go_to_gripper_state": "MoveIt gripper to width=<m> (max 0.032 per finger). No force feedback.",
    "/control/gripper_open": "MoveIt gripper open shortcut (0.1 m). No force feedback.",
    "/control/gripper_close": "MoveIt gripper close shortcut (0.01 m). No force feedback.",
    "/control/gripper_open_force": "Force-based open via franka_gripper/move. Params: width=0.08, speed=0.1, timeout=10, auto_recover=1, max_retries=1.",
    "/control/gripper_grasp": "Force-based grasp via franka_gripper/grasp; closes until contact then applies force. Params: width=0.0, speed=0.05, force=20 (N), eps_in=0.005, eps_out=0.005 (widen to 0.08 for unknown widths), timeout=10, auto_recover=1, max_retries=1 (use max_retries=0 while transporting a held object).",
    "/control/plan_cartesian_path": "Straight-line (MoveL) move to ABSOLUTE x,y,z in meters. Optional TCP orientation target: roll&pitch&yaw (degrees, base frame, sxyz; straight down = 180,0,0) or qx&qy&qz&qw quaternion -- overrides preserve_orientation. Other params: auto_recover=1, max_retries=1, preserve_orientation=1 (0 = reset to canonical down), max_joint_travel_deg=100 (big-swing guard: plans that wind any joint except wrist roll J7 further are rejected with 409 plan_too_large; 0 disables). WARNING: moves the arm.",
    "/control/plan_joint_path": "Joint-space plan+execute to x,y,z. Same params as plan_cartesian_path (incl. roll/pitch/yaw or qx..qw orientation target) plus planner_id=LIN|RRTConnect (use LIN for a minimal in-place reorientation, RRTConnect for large moves; RRTConnect goals are solved by IK seeded from the current joints so the arm keeps its configuration, and the least-travel of 3 plans is used). WARNING: moves the arm.",
    "/simulation/add_box": "Add the fixed demo box to the planning scene.",
    "/simulation/remove_box": "Remove the demo box from the planning scene.",
    "/simulation/attach_box": "Attach the demo box to the gripper (collision-checked as part of the hand).",
    "/simulation/detach_box": "Detach the demo box from the gripper.",
}


@app.route('/help', methods=['GET'])
def api_help():
    """Machine-readable usage guide for this API: every endpoint with its
    methods and description (auto-generated from route docstrings), plus
    conventions (units, locking, outcomes) and a quickstart."""
    import inspect
    endpoints = []
    for rule in app.url_map.iter_rules():
        if rule.endpoint == 'static':
            continue
        view = app.view_functions.get(rule.endpoint)
        doc = inspect.getdoc(view) if view else None
        endpoints.append({
            "path": rule.rule,
            "methods": sorted(m for m in rule.methods if m not in ("HEAD", "OPTIONS")),
            "description": doc or _HELP_NOTES.get(rule.rule, ""),
        })
    endpoints.sort(key=lambda e: e["path"])
    return jsonify({
        "name": "Franka Emika Panda REST API",
        "quickstart": [
            "GET /state to read the robot pose and gripper state; GET /joints for joint angles, velocities, torques and distance to limits.",
            "GET /force for the live end-effector force/torque estimate.",
            "GET /control/plan_cartesian_path?x=&y=&z= to move (meters, absolute; MOVES THE ARM).",
            "GET /control/gripper_grasp?force=20 to grasp; /control/gripper_open_force to open.",
            "GET /control/goto_joints?joints=0,-45,0,-135,0,90,45&unit=deg to move to a joint configuration (dry_run=1 to preview; MOVES THE ARM).",
            "GET /control/stop to stop motion; GET /recover to clear reflex errors.",
            "GET /camera/stream (MJPEG, browser-viewable) for a live view; /camera/color for a still; /camera/start and /camera/stop control the device.",
            "PUT /files/cad/part.stl (raw body) to store a CAD model on the server; GET /files?q=*.stl to find it.",
            "POST /scene/models {name, file, scale} to register it; POST /scene/objects {name, model, x,y,z, yaw} to place it; PUT /scene/objects/sync {source, objects:[...]} from a detector.",
            "Scene API v2 for whole-scene pushes: GET /scene/info, POST /scene/batch (atomic, primitives + meshes, namespaces), /scene/acm, /scene/padding, /scene/snapshot; plan-only: POST /plan/check_state, /plan/dry_run, /plan/ik, /plan/check_trajectory, /plan/dry_run_sequence; GET /robot/tf, /robot/description.",
        ],
        "conventions": {
            "units": "meters, Newtons, N*m, radians; positions are absolute in the robot base frame",
            "concurrency": "motion/gripper endpoints share one non-blocking lock -> 409 'Robot is busy' instead of queuing; /state /force /health /control/stop are lock-free",
            "outcomes": "motion endpoints return outcome: success | plan_failed | execute_failed | stopped | stopped_external_force | stopped_user_button | recovery_failed (+ attempts, recovery_* fields)",
            "safety": "pushing the robot triggers a reflex -> outcome=stopped_external_force (no auto-retry); the hardware user-stop button always wins and software cannot clear it",
        },
        "endpoints": endpoints,
        "readme": "https://github.com/XiaoxiaoZ/Franka-Emika-Panda-robot-REST-api",
    }), 200


_WEBUI_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webui.html")


@app.route('/ui', methods=['GET'])
def web_ui():
    """Browser control panel (single self-contained page, no install needed):
    open http://<this-host>:5000/ui from any device on the network. Presets,
    detect+pick, gripper (incl. force grasp), MoveL jog, force tare, STOP."""
    try:
        with open(_WEBUI_PATH, "r", encoding="utf-8") as f:
            html = f.read()
    except OSError:
        return jsonify({"error": "webui.html not found next to restful.py"}), 404
    return html, 200, {"Content-Type": "text/html; charset=utf-8"}


# ----- Intel RealSense D435 (color + depth over HTTP) -----
# Constructing the object is cheap (no USB access); the device opens lazily on
# the first /camera request, so the debug reloader's supervisor process never
# claims the camera. All camera endpoints are lock-free w.r.t. robot motion.
camera = RealsenseCamera()


def _camera_unavailable(err):
    return jsonify({"error": "camera_unavailable",
                    "msg": err or "RealSense D435 not reachable"}), 503


@app.route('/camera/info', methods=['GET'])
def camera_info():
    """D435 status + color-stream intrinsics (fx, fy, ppx, ppy, coeffs),
    depth_scale_m, frame age. Does not start the camera; use /camera/start or
    any image endpoint to start streaming."""
    return jsonify(camera.info()), 200


@app.route('/camera/config', methods=['GET'])
def camera_config():
    """Get or set the camera stream configuration.
    Without params: current config + every (width, height, fps) mode the
    device supports for color and depth. With width=&height=&fps= (any
    subset): restart the stream with the new COLOR mode, validated against
    the real device -- on failure the previous config is restored and 400 is
    returned. Depth picks a compatible mode automatically (aligned output is
    color-sized regardless). Common D435 color modes: 1920x1080, 1280x720,
    848x480, 640x480, 424x240 at 6/15/30 fps (60 at low res)."""
    w = request.args.get("width")
    h = request.args.get("height")
    fps = request.args.get("fps")
    if not any((w, h, fps)):
        return jsonify({
            "current": {"width": camera.width, "height": camera.height,
                        "fps": camera.fps, "streaming": camera.info()["streaming"]},
            "supported_modes": camera.supported_modes(),
        }), 200
    try:
        vals = {}
        for k, v in (("width", w), ("height", h), ("fps", fps)):
            if v is not None:
                vals[k] = int(v)
                if not (0 < vals[k] <= 4096):
                    raise ValueError(k)
    except (TypeError, ValueError):
        return jsonify({"error": "width/height/fps must be positive integers"}), 400
    ok, err = camera.set_config(**vals)
    if not ok:
        return jsonify({"error": "config_rejected", "msg": err,
                        "restored": {"width": camera.width, "height": camera.height,
                                     "fps": camera.fps}}), 400
    return jsonify({"status": "reconfigured", **camera.info()}), 200


@app.route('/camera/start', methods=['GET'])
def camera_start():
    """Start the D435 stream (color 640x480 + depth aligned to color).
    Idempotent; image endpoints also auto-start. 503 if the device is missing
    or busy."""
    ok, err = camera.ensure_started()
    if not ok:
        return _camera_unavailable(err)
    return jsonify({"status": "streaming", **camera.info()}), 200


@app.route('/camera/stop', methods=['GET'])
def camera_stop():
    """Stop the stream and release the USB device."""
    camera.stop()
    return jsonify({"status": "stopped"}), 200


@app.route('/camera/color', methods=['GET'])
def camera_color():
    """Latest color frame (auto-starts the camera). Query: format=jpg|png
    (png = LOSSLESS, for archival / segmentation models; default jpg),
    quality=1..100 for jpg (default 85). Response headers X-Frame-Id /
    X-Frame-TS identify the frameset -- compare with /camera/depth to verify
    the two came from the same instant. Viewable directly in a browser."""
    ok, err = camera.ensure_started()
    if not ok:
        return _camera_unavailable(err)
    fmt = request.args.get("format", "jpg").lower()
    if fmt not in ("jpg", "jpeg", "png"):
        return jsonify({"error": "format must be jpg or png"}), 400
    try:
        quality = int(request.args.get("quality", "85"))
    except ValueError:
        return jsonify({"error": "bad quality"}), 400
    fr = camera.latest()
    if fr is None:
        return _camera_unavailable("no frame yet")
    import cv2
    if fmt == "png":
        ok2, buf = cv2.imencode(".png", fr["color"])
        mime = "image/png"
    else:
        ok2, buf = cv2.imencode(".jpg", fr["color"],
                                [cv2.IMWRITE_JPEG_QUALITY, max(1, min(100, quality))])
        mime = "image/jpeg"
    if not ok2:
        return jsonify({"error": "encode failed"}), 500
    resp = Response(buf.tobytes(), mimetype=mime)
    resp.headers["X-Frame-Id"] = str(fr.get("id", 0))
    resp.headers["X-Frame-TS"] = repr(fr["ts"])
    return resp


@app.route('/camera/depth', methods=['GET'])
def camera_depth():
    """Latest aligned depth frame as 16-bit PNG in MILLIMETERS (0 = no data).
    Lossless; pixel (u,v) corresponds to the same (u,v) in /camera/color.
    Convert to meters: value / 1000. Headers X-Frame-Id / X-Frame-TS identify
    the frameset (match against /camera/color, or use /camera/frame to get a
    guaranteed-consistent pair in one request)."""
    ok, err = camera.ensure_started()
    if not ok:
        return _camera_unavailable(err)
    b = camera.frame_bundle()
    if b is None:
        return _camera_unavailable("no frame yet")
    import cv2
    ok2, buf = cv2.imencode(".png", b["depth_mm"])
    if not ok2:
        return jsonify({"error": "png encode failed"}), 500
    resp = Response(buf.tobytes(), mimetype="image/png")
    resp.headers["X-Depth-Units"] = "millimeters"
    resp.headers["X-Frame-Id"] = str(b["id"])
    resp.headers["X-Frame-TS"] = repr(b["ts"])
    return resp


@app.route('/camera/frame', methods=['GET'])
def camera_frame():
    """Color + depth from the SAME frameset in one request -- no risk of the
    two being a frame apart (matters right after the arm stops). Returns a
    compressed .npz: color_bgr (HxWx3 u8), depth_mm (HxW u16, 0 = no data),
    frame_id, ts, fx, fy, ppx, ppy, depth_scale_m. Load with numpy:
    d = np.load(io.BytesIO(resp.content)); d['color_bgr'], d['depth_mm']."""
    ok, err = camera.ensure_started()
    if not ok:
        return _camera_unavailable(err)
    b = camera.frame_bundle()
    if b is None:
        return _camera_unavailable("no frame yet")
    import io
    import numpy as _np
    bio = io.BytesIO()
    _np.savez_compressed(
        bio, color_bgr=b["color"], depth_mm=b["depth_mm"],
        frame_id=_np.int64(b["id"]), ts=_np.float64(b["ts"]),
        fx=_np.float64(b["fx"]), fy=_np.float64(b["fy"]),
        ppx=_np.float64(b["ppx"]), ppy=_np.float64(b["ppy"]),
        depth_scale_m=_np.float64(b["depth_scale_m"]))
    resp = Response(bio.getvalue(), mimetype="application/octet-stream")
    resp.headers["Content-Disposition"] = f"attachment; filename=frame_{b['id']}.npz"
    resp.headers["X-Frame-Id"] = str(b["id"])
    resp.headers["X-Frame-TS"] = repr(b["ts"])
    return resp


@app.route('/camera/stream', methods=['GET'])
def camera_stream():
    """Live MJPEG stream for a browser: <img src="/camera/stream">. Query:
    view=color|depth (depth = colorized preview, max_m=4.0 far clip),
    fps=10 (1..30), quality=70 (1..100). Auto-starts the camera. Only new
    framesets are encoded, so the stream never runs faster than the sensor
    and idles cheaply. Ends as soon as the browser closes the connection;
    for a single still use /camera/color."""
    ok, err = camera.ensure_started()
    if not ok:
        return _camera_unavailable(err)
    view = request.args.get("view", "color").lower()
    if view not in ("color", "depth"):
        return jsonify({"error": "view must be color or depth"}), 400
    try:
        fps = max(1.0, min(30.0, float(request.args.get("fps", "10"))))
        quality = max(1, min(100, int(request.args.get("quality", "70"))))
        max_m = float(request.args.get("max_m", "4.0"))
        if not (0.1 <= max_m <= 20):
            raise ValueError
    except ValueError:
        return jsonify({"error": "bad fps / quality / max_m"}), 400

    import cv2
    import time as _time
    min_period = 1.0 / fps
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]

    def frames():
        last_id = -1
        last_sent = 0.0
        idle_since = _time.time()
        while True:
            fr = camera.latest()
            now = _time.time()
            # Stop if the camera was stopped elsewhere (/camera/stop) or the
            # grab thread has been silent for a while -- otherwise a dead
            # stream would hold this worker thread forever.
            if fr is None or not camera.info().get("streaming"):
                if now - idle_since > 5.0:
                    return
                _time.sleep(0.1)
                continue
            if fr["id"] == last_id or now - last_sent < min_period:
                _time.sleep(0.005)
                continue
            idle_since = now
            if view == "depth":
                img = camera.depth_preview_bgr(max_m=max_m)
                if img is None:
                    _time.sleep(0.05)
                    continue
            else:
                img = fr["color"]
            ok2, buf = cv2.imencode(".jpg", img, encode_params)
            if not ok2:
                return
            last_id, last_sent = fr["id"], now
            jpg = buf.tobytes()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
                   b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n"
                   + jpg + b"\r\n")

    resp = Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame")
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
    resp.headers["Age"] = "0"
    return resp


@app.route('/camera/depth/preview', methods=['GET'])
def camera_depth_preview():
    """Colorized depth as JPEG for humans (JET colormap; query max_m=4.0 sets
    the far clip in meters; black = no data). For real data use /camera/depth."""
    ok, err = camera.ensure_started()
    if not ok:
        return _camera_unavailable(err)
    try:
        max_m = float(request.args.get("max_m", "4.0"))
        if not (0.1 <= max_m <= 20):
            raise ValueError
    except ValueError:
        return jsonify({"error": "bad max_m (0.1..20)"}), 400
    img = camera.depth_preview_bgr(max_m=max_m)
    if img is None:
        return _camera_unavailable("no frame yet")
    import cv2
    ok2, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok2:
        return jsonify({"error": "jpeg encode failed"}), 500
    return Response(buf.tobytes(), mimetype="image/jpeg")


@app.route('/camera/depth_at', methods=['GET'])
def camera_depth_at():
    """Depth at pixel (u,v) of the aligned images: median over a win x win
    window (default 5), zeros ignored. Returns depth_m and the deprojected 3D
    point_camera_m in the CAMERA frame (x right, y down, z forward, meters).
    Query: u=<0..639> v=<0..479> [win=5]."""
    ok, err = camera.ensure_started()
    if not ok:
        return _camera_unavailable(err)
    try:
        u = int(request.args.get("u"))
        v = int(request.args.get("v"))
        win = int(request.args.get("win", "5"))
        if not (1 <= win <= 51):
            raise ValueError
    except (TypeError, ValueError):
        return jsonify({"error": "need integer u, v (and optional win 1..51)"}), 400
    out = camera.depth_at(u, v, win=win)
    if out is None:
        return _camera_unavailable("no frame yet")
    if "error" in out:
        return jsonify(out), 400
    return jsonify(out), 200


# Fixed target only (no ?url= override -- that would make this an open proxy /
# SSRF vector). Override at server start via FRANKA_DETECT_URL if needed.
_DETECT_URL = os.environ.get("FRANKA_DETECT_URL", "http://172.26.0.205:5000/detect")


@app.route('/detect_proxy', methods=['GET'])
def detect_proxy():
    """Server-side proxy to the vision /detect service so the web UI can call
    it without browser CORS issues. Target is fixed at server start (env
    FRANKA_DETECT_URL, default the lab vision host); returns its JSON verbatim."""
    try:
        import requests as _rq
        r = _rq.get(_DETECT_URL, timeout=15)
        return r.text, r.status_code, {
            "Content-Type": r.headers.get("Content-Type", "application/json")}
    except Exception:
        return jsonify({"error": "detect_unreachable",
                        "msg": f"vision service at {_DETECT_URL} not reachable"}), 502


# Latest /joint_states (arm joints 1-7 + fingers, with velocity and measured
# torque), cached by a subscriber so /joints is lock-free and cheap. Read-only,
# so the reloader's duplicate subscriber in the supervisor process is harmless.
_joint_state = {"msg": None, "rx": 0.0}
def _on_joint_states(msg):
    _joint_state["msg"] = msg
    _joint_state["rx"] = time.time()
rospy.Subscriber("/joint_states", JointState, _on_joint_states, queue_size=1)


def _arm_joint_positions():
    """7 arm joint angles (rad) from the cached /joint_states, or None."""
    msg = _joint_state["msg"]
    if msg is None:
        return None
    lookup = dict(zip(msg.name, msg.position))
    try:
        return [lookup[f"panda_joint{i}"] for i in range(1, 8)]
    except KeyError:
        return None


@app.route('/joints', methods=['GET'])
def get_joints():
    """Current joint state. Arm joints panda_joint1..7 with position (rad and
    deg), velocity (rad/s), measured torque (Nm), the joint limits and the
    distance to the nearest limit (a joint closer than ~0.02 rad makes
    motion endpoints refuse with joint_near_limit). Also finger positions
    (m, each finger) and gripper width. "positions" is the plain 7-list in
    radians for scripts. stamp is the ROS time of the sample, age_s how old
    it is. Lock-free; 503 if no /joint_states has arrived yet."""
    msg = _joint_state["msg"]
    if msg is None:
        return jsonify({"error": "no_joint_state", "msg": "no /joint_states received yet"}), 503
    import math as _m
    pos = dict(zip(msg.name, msg.position))
    vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
    eff = dict(zip(msg.name, msg.effort)) if msg.effort else {}
    joints = []
    for i in range(1, 8):
        name = f"panda_joint{i}"
        if name not in pos:
            return jsonify({"error": "incomplete_joint_state", "msg": f"{name} missing from /joint_states"}), 503
        q = pos[name]
        lo, hi = PANDA_JOINT_LIMITS[i - 1]
        margin = min(q - lo, hi - q)
        joints.append({
            "name": name, "index": i,
            "position_rad": q, "position_deg": _m.degrees(q),
            "velocity_rad_s": vel.get(name), "effort_nm": eff.get(name),
            "limit_lo_rad": lo, "limit_hi_rad": hi,
            "to_limit_rad": margin, "to_limit_deg": _m.degrees(margin),
        })
    fingers = {n: pos[n] for n in ("panda_finger_joint1", "panda_finger_joint2") if n in pos}
    body = {
        "positions": [j["position_rad"] for j in joints],
        "positions_deg": [j["position_deg"] for j in joints],
        "joints": joints,
        "fingers": fingers,
        "gripper_width_m": sum(fingers.values()) if len(fingers) == 2 else None,
        "near_limit": [j["name"] for j in joints if j["to_limit_rad"] < 0.02],
        "stamp": msg.header.stamp.to_sec(),
        "age_s": round(time.time() - _joint_state["rx"], 3),
    }
    return jsonify(body), 200


@app.route('/state', methods = ['GET'])
def get_state():
    pose = robot.move_group.get_current_pose()
    gripper_width = robot.hand_group.get_current_joint_values()[0]
    #print("gripper_width:",gripper_width)
    if gripper_width>0.03:
        gripper = 1 # open
    else:
        gripper = 0 # close
    state_dict = {
        "position": {
            "x": pose.pose.position.x,
            "y": pose.pose.position.y,
            "z": pose.pose.position.z
        },
        "orientation": {
            "x": pose.pose.orientation.x,
            "y": pose.pose.orientation.y,
            "z": pose.pose.orientation.z,
            "w": pose.pose.orientation.w
        },
        "gripper": gripper
    }
    # Arm joint angles (rad, joint1..7) -- /joints has velocity/torque/limits.
    q = _arm_joint_positions()
    if q is None:
        q = robot.move_group.get_current_joint_values()
    state_dict["joints"] = list(q)
    # Same orientation as human-friendly Euler angles (degrees, sxyz, base
    # frame) -- matches the roll/pitch/yaw params the motion endpoints accept.
    try:
        from tf.transformations import euler_from_quaternion
        import math as _m
        r, p, yw = euler_from_quaternion([
            pose.pose.orientation.x, pose.pose.orientation.y,
            pose.pose.orientation.z, pose.pose.orientation.w])
        state_dict["rpy_deg"] = {"roll": _m.degrees(r), "pitch": _m.degrees(p),
                                 "yaw": _m.degrees(yw)}
    except Exception:
        pass  # tf not available -> just omit the convenience field
    return jsonify(state_dict), 200
    
@app.route('/simulation/remove_box', methods = ['GET'])
def remove_box():
    # add_floor() reuses the same single name slot, so this used to delete
    # the floor; built-in safety geometry is never removed here.
    from scene_manager import BUILTIN_OBJECTS
    if robot.box_name in BUILTIN_OBJECTS:
        return jsonify({"error": "builtin", "msg": f"{robot.box_name!r} is built-in safety geometry; "
                        "use /scene/objects for your own objects"}), 409
    robot.remove_box()
    return jsonify({"status": "success",'msg': "box removed"}), 200

@app.route('/simulation/add_box', methods = ['GET'])
def add_box():
    robot.add_box()
    return jsonify({"status": "success",'msg': "box added on the tool"}), 200

@app.route('/simulation/detach_box', methods = ['GET'])
def detach_box():
    robot.detach_box()
    return jsonify({"status": "success",'msg': "box detached"}), 200

@app.route('/simulation/attach_box', methods = ['GET'])
def attach_box():
    robot.attach_box()
    return jsonify({"status": "success",'msg': "box attached on the tool"}), 200
    
def go_to_gripper_state():
    try:
        width = float(request.args.get("width"))
    except (TypeError, ValueError):
        return jsonify({"error": "No width arg"}), 400
    try:
        result = robot.go_to_gripper_state(width)
        return jsonify({'result': result}), 200
    except Exception as error:
        return jsonify({'error': str(error)}), 500
@app.route('/control/go_to_gripper_state', methods = ['GET'])
def go_to_gripper_state_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return go_to_gripper_state()
    finally: moveit_lock.release()

def gripper_open():
    try:
        result = robot.go_to_gripper_state(0.1)
        return jsonify({'result': result}), 200
    except Exception as error:
        return jsonify({'error': str(error)}), 500

def gripper_close():
    try:
        result = robot.go_to_gripper_state(0.01)
        return jsonify({'result': result}), 200
    except Exception as error:
        return jsonify({'error': str(error)}), 500
    
def _gripper_action_call(client, server_name, goal, timeout_s, auto_recover, max_retries):
    """Run a gripper action with optional auto-recovery retry.

    On action timeout or action-reported failure: optionally call
    robot.recover() and resend the goal up to max_retries times.

    WARNING for transit scenarios (gripper is currently holding an object
    during arm motion and you are issuing a new gripper action): pass
    max_retries=0 to prevent a reset-and-retry cycle that could disturb
    the grasp.

    Returns dict with: outcome, attempts, recovery_triggered,
    recovery_succeeded, result (the action result msg, may be None).
    """
    if not client.wait_for_server(rospy.Duration(1.0)):
        return {
            'outcome': 'server_unreachable',
            'attempts': 0,
            'recovery_triggered': False,
            'recovery_succeeded': False,
            'result': None,
            'error': f'{server_name} action server not reachable',
        }

    recovery_triggered = False
    recovery_succeeded = False
    attempts = 0
    last_err = None
    last_result = None

    while attempts <= max_retries:
        attempts += 1
        client.send_goal(goal)
        finished = client.wait_for_result(rospy.Duration(timeout_s))
        if not finished:
            client.cancel_goal()
            last_err = f'{server_name} timed out after {timeout_s}s'
            if not auto_recover or attempts > max_retries:
                return {
                    'outcome': 'action_timeout',
                    'attempts': attempts,
                    'recovery_triggered': recovery_triggered,
                    'recovery_succeeded': recovery_succeeded,
                    'result': None,
                    'error': last_err,
                }
            recovery_triggered = True
            recovery_succeeded = robot.recover()
            if not recovery_succeeded:
                return {
                    'outcome': 'recovery_failed',
                    'attempts': attempts,
                    'recovery_triggered': True,
                    'recovery_succeeded': False,
                    'result': None,
                    'error': last_err,
                }
            continue

        last_result = client.get_result()
        ok = last_result is not None and bool(getattr(last_result, "success", False))
        if ok:
            return {
                'outcome': 'success',
                'attempts': attempts,
                'recovery_triggered': recovery_triggered,
                'recovery_succeeded': recovery_succeeded,
                'result': last_result,
            }
        last_err = getattr(last_result, "error", "action reported failure") if last_result else "no result"
        if not auto_recover or attempts > max_retries:
            return {
                'outcome': 'action_failed',
                'attempts': attempts,
                'recovery_triggered': recovery_triggered,
                'recovery_succeeded': recovery_succeeded,
                'result': last_result,
                'error': last_err,
            }
        recovery_triggered = True
        recovery_succeeded = robot.recover()
        if not recovery_succeeded:
            return {
                'outcome': 'recovery_failed',
                'attempts': attempts,
                'recovery_triggered': True,
                'recovery_succeeded': False,
                'result': last_result,
                'error': last_err,
            }

    return {
        'outcome': 'action_failed',
        'attempts': attempts,
        'recovery_triggered': recovery_triggered,
        'recovery_succeeded': recovery_succeeded,
        'result': last_result,
        'error': last_err,
    }


def _gripper_status_code(outcome):
    return {
        'success': 200,
        'action_timeout': 504,
        'server_unreachable': 503,
        'action_failed': 500,
        'recovery_failed': 500,
    }.get(outcome, 500)


def gripper_open1():
    """
    Open the gripper using franka_gripper/move.
    Query params:
      width (default 0.08), speed (0.1), timeout (10.0)
      auto_recover (1), max_retries (1) -- pass max_retries=0 for transit scenarios
    """
    try:
        width = float(request.args.get("width", "0.08"))
        speed = float(request.args.get("speed", "0.1"))
        timeout_s = float(request.args.get("timeout", "10.0"))
        auto_recover = request.args.get("auto_recover", "1") != "0"
        max_retries = int(request.args.get("max_retries", "1"))
    except (TypeError, ValueError):
        return jsonify({"error": "Bad width/speed/timeout/auto_recover/max_retries params"}), 400

    goal = MoveGoal()
    goal.width = width
    goal.speed = speed

    outcome = _gripper_action_call(
        gripper_move_client, "franka_gripper/move", goal, timeout_s,
        auto_recover=auto_recover, max_retries=max_retries,
    )
    return jsonify({
        "outcome": outcome['outcome'],
        "success": outcome['outcome'] == 'success',
        "attempts": outcome['attempts'],
        "recovery_triggered": outcome['recovery_triggered'],
        "recovery_succeeded": outcome['recovery_succeeded'] if outcome['recovery_triggered'] else None,
        "final_width_cmd": float(width),
        "error_msg": outcome.get('error'),
    }), _gripper_status_code(outcome['outcome'])


def gripper_close1():
    """
    Close the gripper using franka_gripper/grasp.
    Query params:
      width (default 0.0 = close until contact), speed (0.05), force (20.0),
      eps_in (0.005), eps_out (0.005), timeout (10.0)
      auto_recover (1), max_retries (1) -- pass max_retries=0 for transit scenarios
    """
    try:
        width = float(request.args.get("width", "0.0"))
        speed = float(request.args.get("speed", "0.05"))
        force = float(request.args.get("force", "20.0"))
        eps_in = float(request.args.get("eps_in", "0.005"))
        eps_out = float(request.args.get("eps_out", "0.005"))
        timeout_s = float(request.args.get("timeout", "10.0"))
        auto_recover = request.args.get("auto_recover", "1") != "0"
        max_retries = int(request.args.get("max_retries", "1"))
    except (TypeError, ValueError):
        return jsonify({"error": "Bad grasp params"}), 400

    goal = GraspGoal()
    goal.width = width
    goal.speed = speed
    goal.force = force
    goal.epsilon.inner = eps_in
    goal.epsilon.outer = eps_out

    outcome = _gripper_action_call(
        gripper_grasp_client, "franka_gripper/grasp", goal, timeout_s,
        auto_recover=auto_recover, max_retries=max_retries,
    )
    return jsonify({
        "outcome": outcome['outcome'],
        "success": outcome['outcome'] == 'success',
        "attempts": outcome['attempts'],
        "recovery_triggered": outcome['recovery_triggered'],
        "recovery_succeeded": outcome['recovery_succeeded'] if outcome['recovery_triggered'] else None,
        "used_width_cmd": float(width),
        "force": float(force),
        "error_msg": outcome.get('error'),
    }), _gripper_status_code(outcome['outcome'])

@app.route('/control/gripper_open', methods = ['GET'])
def gripper_open_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return gripper_open()
    finally: moveit_lock.release()

@app.route('/control/gripper_close', methods = ['GET'])
def gripper_close_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return gripper_close()
    finally: moveit_lock.release()

@app.route('/control/gripper_open_force', methods = ['GET'])
def gripper_open_force_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return gripper_open1()
    finally: moveit_lock.release()

@app.route('/control/gripper_grasp', methods = ['GET'])
def gripper_grasp_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return gripper_close1()
    finally: moveit_lock.release()

def _parse_orientation_args():
    """Optional TCP orientation target from query params. Two formats:
      qx,qy,qz,qw  -- quaternion in the base frame (all four required), or
      roll,pitch,yaw -- absolute Euler angles in DEGREES, base frame, 'sxyz'
                        (all three required; gripper straight down = 180,0,0).
    Returns a normalized (qx,qy,qz,qw) tuple, or None if no orientation given.
    Raises ValueError on partial/invalid input."""
    import math as _m
    q_parts = [request.args.get(k) for k in ("qx", "qy", "qz", "qw")]
    rpy_parts = [request.args.get(k) for k in ("roll", "pitch", "yaw")]
    if any(v is not None for v in q_parts):
        if any(v is None for v in q_parts):
            raise ValueError("quaternion needs all of qx,qy,qz,qw")
        q = [float(v) for v in q_parts]
    elif any(v is not None for v in rpy_parts):
        if any(v is None for v in rpy_parts):
            raise ValueError("euler orientation needs all of roll,pitch,yaw (degrees)")
        from tf.transformations import quaternion_from_euler
        r, p, yw = (_m.radians(float(v)) for v in rpy_parts)
        q = list(quaternion_from_euler(r, p, yw))  # sxyz -> (x, y, z, w)
    else:
        return None
    if not all(_m.isfinite(v) for v in q):
        raise ValueError("orientation must be finite")
    n = _m.sqrt(sum(v * v for v in q))
    if n < 1e-6:
        raise ValueError("orientation quaternion has ~zero norm")
    return tuple(v / n for v in q)


def _parse_motion_args():
    """Common parse for motion endpoints.
    Returns (x, y, z, auto_recover, max_retries, preserve_orientation,
    orientation) or raises. orientation is a normalized quaternion tuple or
    None; when given it takes precedence over preserve_orientation."""
    x = float(request.args.get("x"))
    y = float(request.args.get("y"))
    z = float(request.args.get("z"))
    auto_recover = request.args.get("auto_recover", "1") != "0"
    max_retries = int(request.args.get("max_retries", "1"))
    preserve_orientation = request.args.get("preserve_orientation", "1") != "0"
    orientation = _parse_orientation_args()
    return x, y, z, auto_recover, max_retries, preserve_orientation, orientation


def _parse_max_joint_travel():
    """Optional max_joint_travel_deg override for the big-swing guard.
    Returns radians, None for the server default, or 0 to disable."""
    raw = request.args.get("max_joint_travel_deg")
    if raw is None:
        return None
    deg = float(raw)
    if deg < 0:
        raise ValueError("max_joint_travel_deg must be >= 0")
    return math.radians(deg)


def _preflight_joint_limit_check():
    """Return (ok, response_tuple). If ok is False, response_tuple is the
    (json_body, status) to return immediately from the handler."""
    joint_values = robot.move_group.get_current_joint_values()
    near_limit = check_joint_limits(joint_values)
    if near_limit:
        return False, (jsonify({
            "error": "joint_near_limit",
            "msg": "one or more joints are too close to their limits; home the robot or freedrive it first",
            "joints_near_limit": near_limit,
            "hint": "call GET /control/home or use the Panda freedrive button to move the flagged joints away from their limits",
        }), 400)
    return True, None


def _format_motion_response(outcome, extra=None):
    """Translate plan_and_execute_with_retry outcome dict into (json_body, status)."""
    resp = _format_motion_response_inner(outcome, extra)
    try:
        log.info("motion %s -> HTTP %s outcome=%s attempts=%s from %s",
                 request.full_path, resp[1], outcome.get('outcome'), outcome.get('attempts'),
                 request.remote_addr)
    except Exception:
        pass
    return resp


def _format_motion_response_inner(outcome, extra=None):
    body = {
        "outcome": outcome['outcome'],
        "attempts": outcome['attempts'],
        "recovery_triggered": outcome['recovery_triggered'],
        "recovery_succeeded": outcome['recovery_succeeded'] if outcome['recovery_triggered'] else None,
    }
    if extra:
        body.update(extra)

    if outcome['outcome'] == 'success':
        return jsonify({**body, "msg": "Plan 100%"}), 200
    if outcome['outcome'] == 'stopped':
        body["stop_reason"] = outcome.get('stop_reason', 'user /control/stop')
        return jsonify({**body, "msg": "motion stopped by user /control/stop"}), 200
    if outcome['outcome'] == 'stopped_external_force':
        body["stop_reason"] = outcome.get('stop_reason', 'external-force reflex')
        return jsonify({**body, "msg": "motion stopped by external-force reflex (likely user hand); not retrying"}), 200
    if outcome['outcome'] == 'stopped_user_button':
        body["stop_reason"] = outcome.get('stop_reason', 'hardware user-stop button pressed')
        body["hint"] = "release the Panda user-stop button on the hardware before retrying; software cannot clear this state"
        return jsonify({**body, "msg": "motion blocked: Panda user-stop button is active"}), 409
    if outcome['outcome'] == 'plan_failed':
        last_err = last_error[-1] if last_error else None
        body["last_error"] = last_err
        # Re-classify plan-failed as user-stop if the rosout shows it.
        # User-stop is persistent state, so we don't require the error to be
        # freshly logged within this motion's baseline -- the button being
        # pressed at all is enough.
        if is_user_stop_abort(last_err):
            body["outcome"] = "stopped_user_button"
            body["stop_reason"] = last_err
            body["hint"] = "release the Panda user-stop button on the hardware before retrying; software cannot clear this state"
            return jsonify({**body, "msg": "motion blocked: Panda user-stop button is active"}), 409
        return jsonify({**body, "msg": "Plan not 100%"}), 202
    if outcome['outcome'] == 'plan_would_violate_limits':
        body["trajectory_violations"] = outcome.get('trajectory_violations', [])
        body["hint"] = "plan would drive a joint near its limit mid-path; try a different target, preserve_orientation=0, or plan_joint_path with planner_id=RRTConnect"
        return jsonify({**body, "msg": "plan rejected: would drift a joint past safe margin"}), 409
    if outcome['outcome'] == 'plan_too_large':
        body["joint_travel_deg"] = {j: round(math.degrees(t), 1) for j, t in outcome.get('joint_travel', {}).items()}
        body["max_joint_travel_deg"] = round(math.degrees(outcome.get('max_joint_travel_rad', 0.0)), 1)
        body["hint"] = "plan would swing a joint a long way (elbow flip / base spin); move in smaller steps, or pass max_joint_travel_deg=<bigger> (0 disables) if the big move is intended"
        return jsonify({**body, "msg": "plan rejected: joint travel too large"}), 409
    if outcome['outcome'] == 'execute_failed':
        body["last_error"] = last_error[-1] if last_error else None
        return jsonify({**body, "msg": "execution failed"}), 500
    if outcome['outcome'] == 'recovery_failed':
        body["last_error"] = last_error[-1] if last_error else None
        return jsonify({**body, "msg": "execute failed and recovery also failed"}), 500
    return jsonify({**body, "msg": "unknown outcome"}), 500


# Last requested motion, for /control/resume. Set at the start of each motion
# handler and left in place across stops so the caller can explicitly resume.
_last_motion = None


def _do_cartesian(x, y, z, auto_recover, max_retries, preserve_orientation, orientation=None,
                  max_joint_travel=None):
    ok, rejection = _preflight_joint_limit_check()
    if not ok:
        return rejection

    def plan_fn():
        plan, fraction = robot.plan_cartesian_path(
            x, y, z, preserve_orientation=preserve_orientation, orientation=orientation)
        return plan, fraction >= 1.0, {"fraction": fraction}

    # Baseline so the classifier only considers errors that appear AFTER this
    # motion starts. Fixes stale-classification bug where a previous stop's
    # cartesian_reflex message was being re-matched on the next motion.
    baseline = len(last_error)

    def get_new_error():
        return last_error[-1] if len(last_error) > baseline else None

    outcome = robot.plan_and_execute_with_retry(
        plan_fn,
        auto_recover=auto_recover,
        max_retries=max_retries,
        get_last_error=get_new_error,
        max_joint_travel_rad=max_joint_travel,
    )
    extra = {"fraction": str(outcome['plan_metadata'].get('fraction'))}
    if outcome['outcome'] == 'plan_failed':
        extra["moveit_error"] = f"PARTIAL_PATH (fraction={outcome['plan_metadata'].get('fraction')}, likely unreachable target or constraint too tight)"
    return _format_motion_response(outcome, extra)


def plan_cartesian_path():
    try:
        x, y, z, auto_recover, max_retries, preserve_orientation, orientation = _parse_motion_args()
        max_joint_travel = _parse_max_joint_travel()
    except (TypeError, ValueError) as e:
        return jsonify({"error": "bad motion args", "msg": str(e)}), 400

    global _last_motion
    _last_motion = {
        'kind': 'cartesian', 'x': x, 'y': y, 'z': z,
        'auto_recover': auto_recover, 'max_retries': max_retries,
        'preserve_orientation': preserve_orientation,
        'orientation': orientation,
        'max_joint_travel': max_joint_travel,
    }

    return _do_cartesian(x, y, z, auto_recover, max_retries, preserve_orientation,
                         orientation=orientation, max_joint_travel=max_joint_travel)


@app.route('/control/plan_cartesian_path', methods = ['GET'])
def plan_cartesian_path_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return plan_cartesian_path()
    finally: moveit_lock.release()


def _do_joint(x, y, z, auto_recover, max_retries, preserve_orientation, planner_id="LIN",
              orientation=None, max_joint_travel=None):
    ok, rejection = _preflight_joint_limit_check()
    if not ok:
        return rejection

    def plan_fn():
        (plan_success, plan, planning_time, error_code) = robot.plan_joint_path(
            x, y, z, preserve_orientation=preserve_orientation, planner_id=planner_id,
            orientation=orientation,
        )
        return plan, bool(plan_success), {"planning_time": planning_time, "error_code": error_code, "planner_id": planner_id}

    baseline = len(last_error)

    def get_new_error():
        return last_error[-1] if len(last_error) > baseline else None

    outcome = robot.plan_and_execute_with_retry(
        plan_fn,
        auto_recover=auto_recover,
        max_retries=max_retries,
        get_last_error=get_new_error,
        max_joint_travel_rad=max_joint_travel,
    )
    extra = {"planning_time": str(outcome['plan_metadata'].get('planning_time'))}
    if outcome['outcome'] == 'plan_failed':
        extra["moveit_error"] = decode_moveit_error(outcome['plan_metadata'].get('error_code'))
    return _format_motion_response(outcome, extra)


def plan_joint_path():
    try:
        x, y, z, auto_recover, max_retries, preserve_orientation, orientation = _parse_motion_args()
        planner_id = request.args.get("planner_id", "LIN")
        # Legacy (Kinetic-era) OMPL planner id; Noetic's panda_moveit_config
        # names it plain "RRTConnect" and warns + falls back on the old name.
        if planner_id == "RRTConnectkConfigDefault":
            planner_id = "RRTConnect"
        max_joint_travel = _parse_max_joint_travel()
    except (TypeError, ValueError) as e:
        return jsonify({"error": "bad motion args", "msg": str(e)}), 400

    global _last_motion
    _last_motion = {
        'kind': 'joint', 'x': x, 'y': y, 'z': z,
        'auto_recover': auto_recover, 'max_retries': max_retries,
        'preserve_orientation': preserve_orientation,
        'orientation': orientation,
        'planner_id': planner_id,
        'max_joint_travel': max_joint_travel,
    }

    return _do_joint(x, y, z, auto_recover, max_retries, preserve_orientation,
                     planner_id=planner_id, orientation=orientation,
                     max_joint_travel=max_joint_travel)


@app.route('/control/plan_joint_path', methods = ['GET'])
def plan_joint_path_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return plan_joint_path()
    finally: moveit_lock.release()
def _parse_goto_joints(current):
    """Target for goto_joints from query args or a JSON body.
    joints=<7 comma-separated values> (or JSON "joints": [...]) sets all
    seven; j1..j7=<value> override single joints, starting from the current
    configuration (so j7=1.2 alone just turns the wrist). unit=rad|deg
    (default rad). Raises ValueError with a readable message."""
    data = request.get_json(silent=True) if request.method == "POST" else None
    data = data if isinstance(data, dict) else {}
    get = lambda k: data.get(k, request.args.get(k))
    unit = str(get("unit") or "rad").lower()
    if unit not in ("rad", "deg"):
        raise ValueError("unit must be rad or deg")
    conv = math.radians if unit == "deg" else float
    target = list(current)
    given = False
    raw = get("joints")
    if raw is not None:
        vals = raw if isinstance(raw, list) else [v for v in str(raw).replace("[", "").replace("]", "").split(",") if v.strip()]
        if len(vals) != 7:
            raise ValueError(f"joints needs exactly 7 values, got {len(vals)}")
        target = [conv(float(v)) for v in vals]
        given = True
    for i in range(1, 8):
        v = get(f"j{i}")
        if v is not None:
            target[i - 1] = conv(float(v))
            given = True
    if not given:
        raise ValueError("give joints=<7 values> and/or j1..j7=<value>")
    for i, q in enumerate(target):
        if math.isnan(q) or math.isinf(q):
            raise ValueError(f"j{i + 1} must be finite")
    return target, unit


def _do_goto_joints(target, auto_recover, max_retries, velocity, max_joint_travel, dry_run=False):
    # Target must be clear of the limits by the same margin the controller
    # needs -- a goal ON the limit plans fine and then never executes.
    bad = check_joint_limits(target)
    if bad:
        return jsonify({"error": "target_near_limit",
                        "msg": "target puts a joint within 0.02 rad of its limit",
                        "joints_near_limit": bad}), 400
    # A goal in collision can only fail, and slowly (PTP refuses, then OMPL
    # samples until it times out) -- check it up front and say why.
    try:
        valid, contacts = robot.check_joint_state(target)
    except Exception as e:
        return jsonify({"error": "validity_check_unavailable", "msg": str(e)}), 503
    if not valid:
        return jsonify({"error": "target_in_collision",
                        "msg": "the target configuration collides with the robot itself or the planning scene",
                        "contacts": contacts}), 400
    if not dry_run:
        ok, rejection = _preflight_joint_limit_check()
        if not ok:
            return rejection

    def plan_fn():
        ok_, plan, t, err, planner = robot.plan_joint_goal(target, velocity_scaling=velocity)
        return plan, bool(ok_), {"planning_time": t, "error_code": err, "planner_used": planner}

    if dry_run:
        from franka import trajectory_joint_travel
        plan, ok_, meta = plan_fn()
        travel = trajectory_joint_travel(plan) if ok_ else {}
        limit = max_joint_travel if max_joint_travel is not None else robot.MAX_JOINT_TRAVEL_RAD
        too_far = [j for j, t in travel.items()
                   if limit and t > limit and j not in robot.JOINT_TRAVEL_EXEMPT]
        body = {
            "outcome": "planned" if ok_ else "plan_failed",
            "dry_run": True,
            "planner_used": meta["planner_used"],
            "planning_time": meta["planning_time"],
            "waypoints": len(plan.joint_trajectory.points) if ok_ else 0,
            "duration_s": plan.joint_trajectory.points[-1].time_from_start.to_sec() if ok_ and plan.joint_trajectory.points else None,
            "joint_travel_deg": {j: round(math.degrees(t), 1) for j, t in travel.items()},
            "would_be_rejected_as_too_large": too_far,
            "target_rad": target,
        }
        if not ok_:
            body["moveit_error"] = decode_moveit_error(meta["error_code"])
        return jsonify(body), 200 if ok_ else 202

    baseline = len(last_error)

    def get_new_error():
        return last_error[-1] if len(last_error) > baseline else None

    outcome = robot.plan_and_execute_with_retry(
        plan_fn,
        auto_recover=auto_recover,
        max_retries=max_retries,
        get_last_error=get_new_error,
        max_joint_travel_rad=max_joint_travel,
    )
    extra = {"planner_used": outcome['plan_metadata'].get('planner_used'),
             "planning_time": str(outcome['plan_metadata'].get('planning_time')),
             "target_rad": target}
    if outcome['outcome'] == 'plan_failed':
        extra["moveit_error"] = decode_moveit_error(outcome['plan_metadata'].get('error_code'))
    return _format_motion_response(outcome, extra)


@app.route('/control/goto_joints', methods=['GET', 'POST'])
def goto_joints_impl():
    """Move to an explicit joint configuration. joints=<7 values> (J1..J7)
    and/or j1..j7=<value> to change single joints from the current pose;
    unit=rad|deg (default rad); POST JSON {"joints": [...], "unit": "deg"}
    works too. Planned with Pilz PTP (straight line in joint space: every
    joint moves monotonically, no detours), OMPL RRTConnect as fallback.
    velocity=0.2 (0.01..0.5 of max joint speed). dry_run=1 plans only and
    reports planner, duration and per-joint travel without moving. Same
    safety as the other motion endpoints: target must be >0.02 rad inside
    the limits (400 target_near_limit) and collision-free (400
    target_in_collision, with the contact pairs); the planned path is
    collision-checked against the planning scene too; big-swing guard
    max_joint_travel_deg=100 except J7 (409 plan_too_large; 0 disables),
    auto_recover=1, max_retries=1, /control/stop, /control/resume.
    Example: /control/goto_joints?joints=0,-45,0,-135,0,90,45&unit=deg.
    WARNING: moves the arm (unless dry_run=1)."""
    try:
        current = robot.move_group.get_current_joint_values()
        target, _unit = _parse_goto_joints(current)
        data = request.get_json(silent=True) if request.method == "POST" else None
        data = data if isinstance(data, dict) else {}
        get = lambda k, d=None: data.get(k, request.args.get(k, d))
        velocity = float(get("velocity", "0.2"))
        if not (0.01 <= velocity <= 0.5):
            raise ValueError("velocity must be in 0.01..0.5")
        auto_recover = str(get("auto_recover", "1")) not in ("0", "false", "False")
        max_retries = int(get("max_retries", "1"))
        dry_run = str(get("dry_run", "0")) not in ("0", "false", "False")
        mjt = get("max_joint_travel_deg")
        max_joint_travel = None if mjt is None else math.radians(float(mjt))
        if max_joint_travel is not None and max_joint_travel < 0:
            raise ValueError("max_joint_travel_deg must be >= 0")
    except (TypeError, ValueError) as e:
        return jsonify({"error": "bad goto_joints args", "msg": str(e)}), 400

    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try:
        if not dry_run:
            global _last_motion
            _last_motion = {'kind': 'joints', 'target': target, 'auto_recover': auto_recover,
                            'max_retries': max_retries, 'velocity': velocity,
                            'max_joint_travel': max_joint_travel}
        return _do_goto_joints(target, auto_recover, max_retries, velocity, max_joint_travel, dry_run)
    finally:
        moveit_lock.release()


@app.route('/control/resume', methods = ['GET'])
def resume_impl():
    """Re-issue the last requested motion from the robot's current position.
    Useful after a stopped/stopped_external_force outcome when the caller
    wants the arm to complete the motion that was interrupted. Uses the same
    planner kind, target, and settings as the last motion request.

    Always calls /franka_control/error_recovery first to clear any lingering
    reflex state from the previous stop. Without this, the controller silently
    refuses the new execute and the resume plan fails with no new error."""
    global _last_motion
    if _last_motion is None:
        return jsonify({
            "error": "no_motion_to_resume",
            "msg": "no prior motion request has been issued since server start",
        }), 400
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try:
        # Clear reflex error state from previous stop so the new execute
        # isn't silently refused by the controller.
        robot.recover()
        lm = _last_motion
        if lm['kind'] == 'cartesian':
            return _do_cartesian(lm['x'], lm['y'], lm['z'],
                                 lm['auto_recover'], lm['max_retries'], lm['preserve_orientation'],
                                 orientation=lm.get('orientation'),
                                 max_joint_travel=lm.get('max_joint_travel'))
        if lm['kind'] == 'joint':
            return _do_joint(lm['x'], lm['y'], lm['z'],
                             lm['auto_recover'], lm['max_retries'], lm['preserve_orientation'],
                             orientation=lm.get('orientation'),
                             planner_id=lm.get('planner_id', 'LIN'),
                             max_joint_travel=lm.get('max_joint_travel'))
        if lm['kind'] == 'joints':
            return _do_goto_joints(lm['target'], lm['auto_recover'], lm['max_retries'],
                                   lm['velocity'], lm.get('max_joint_travel'))
        return jsonify({"error": f"unknown cached motion kind: {lm.get('kind')}"}), 500
    finally:
        moveit_lock.release()

@app.route('/control/nudge_joint', methods = ['GET'])
def nudge_joint_impl():
    """Minimal-motion recovery for a single joint. Bypasses the pre-flight
    joint-limit check because this endpoint IS the recovery tool for that."""
    try:
        joint_1based = int(request.args.get("joint"))
        delta_rad = float(request.args.get("delta"))
    except (TypeError, ValueError):
        return jsonify({"error": "missing or bad 'joint' (1..7) or 'delta' (rad) param"}), 400
    if not (1 <= joint_1based <= 7):
        return jsonify({"error": "joint must be 1..7"}), 400
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try:
        result = robot.nudge_joint(joint_1based - 1, delta_rad)
        status_code = 200 if result.get("executed") else 500
        return jsonify({
            "status": "nudged" if result.get("executed") else "failed",
            **result,
            "last_error": last_error[-1] if last_error else None,
        }), status_code
    finally:
        moveit_lock.release()

@app.route('/control/home', methods = ['GET'])
def home_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try:
        result = robot.go_home()
        if result.get("executed"):
            return jsonify({
                "status": "homed",
                "msg": "arm moved to canonical joint pose",
                "planner_used": result.get("planner_used"),
                "planning_time": str(result.get("planning_time")),
            }), 200
        return jsonify({
            "status": "failed",
            "msg": "home motion failed",
            "planner_tried": result.get("planner_tried"),
            "plan_error_codes": result.get("plan_error_codes"),
            "last_error": last_error[-1] if last_error else None,
        }), 500
    finally:
        moveit_lock.release()

@app.route('/force', methods = ['GET'])
def force():
    """Current end-effector external wrench estimate.
    Returns raw/net force and torque (after tare), their magnitudes, the lever
    arm (|net torque| / |net force|, ~CoM offset of a held object), and frame.
    Lock-free: reading force never blocks on (or blocks) robot motion."""
    data = force_viz.get_force()
    return jsonify(data), 200 if data.get("available") else 503

@app.route('/force/tare', methods = ['GET'])
def force_tare():
    """Zero the force display: snapshot the current raw force as the baseline
    so a subsequently grasped object's weight shows on its own. Tare with an
    empty gripper for best results."""
    baseline = force_viz.tare()
    if baseline is None:
        return jsonify({
            "status": "no_data",
            "msg": "no wrench received yet; cannot tare",
        }), 503
    return jsonify({"status": "tared", "baseline": baseline}), 200

@app.route('/force/untare', methods = ['GET'])
def force_untare():
    """Clear the baseline so the raw F_ext value is shown again."""
    force_viz.untare()
    return jsonify({"status": "untared", "msg": "baseline cleared"}), 200

@app.route('/force/config', methods = ['GET'])
def force_config():
    """Tune the visualization. Query params (all optional):
      scale=<m/N>             force arrow length per Newton (default 0.02)
      threshold=<N>           hide force arrow/label below this (default 0.5)
      max_force=<N>           force color saturates to red at this (20)
      torque_scale=<m/(N.m)>  torque arrow length per N.m (default 0.15)
      torque_threshold=<N.m>  hide torque arrow/label below this (default 0.05)
      max_torque=<N.m>        torque color saturates at this (2.0)"""
    try:
        cfg = force_viz.set_config(
            scale=request.args.get("scale"),
            threshold=request.args.get("threshold"),
            max_force=request.args.get("max_force"),
            torque_scale=request.args.get("torque_scale"),
            torque_threshold=request.args.get("torque_threshold"),
            max_torque=request.args.get("max_torque"),
        )
    except (TypeError, ValueError) as e:
        return jsonify({"error": "bad params", "msg": str(e)}), 400
    return jsonify({"status": "updated", "config": cfg}), 200

@app.route('/force/identify/empty', methods = ['GET'])
def identify_empty():
    """Empty-gripper measurement pass for payload identification.
    Captures the current joint config as base, visits N perturbed configs and
    samples raw F_ext at each, then returns to base. Run with the gripper EMPTY;
    then grasp the object and call /force/identify/loaded. Long-running; holds
    the motion lock for the duration."""
    if payload_identifier is None:
        return jsonify({"error": "identifier unavailable"}), 503
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try:
        data = payload_identifier.record_empty()
        # Install the per-pose residuals as the visualizer's compensation table
        # (does not enable it yet; call /force/compensate?on=1 to switch on).
        n_cal = force_viz.set_calibration(payload_identifier.calibration()) if force_viz else 0
        return jsonify({"status": "empty_recorded", "calibration_points": n_cal, **data}), 200
    finally:
        moveit_lock.release()

@app.route('/force/identify/loaded', methods = ['GET'])
def identify_loaded():
    """Loaded measurement pass: revisit the SAME configs while holding the
    object. Requires /force/identify/empty first (then grasp, then this)."""
    if payload_identifier is None:
        return jsonify({"error": "identifier unavailable"}), 503
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try:
        return jsonify({"status": "loaded_recorded", **payload_identifier.record_loaded()}), 200
    except ValueError as e:
        return jsonify({"error": "sequence", "msg": str(e)}), 409
    finally:
        moveit_lock.release()

@app.route('/force/identify/compute', methods = ['GET'])
def identify_compute():
    """Estimate object mass (and a rough CoM offset) from the empty + loaded
    passes. Lock-free: pure computation over the recorded samples."""
    if payload_identifier is None:
        return jsonify({"error": "identifier unavailable"}), 503
    try:
        return jsonify({"status": "ok", **payload_identifier.compute()}), 200
    except ValueError as e:
        return jsonify({"error": "incomplete", "msg": str(e)}), 409

@app.route('/force/identify/reset', methods = ['GET'])
def identify_reset():
    """Discard recorded passes so a new identification can start fresh."""
    if payload_identifier is None:
        return jsonify({"error": "identifier unavailable"}), 503
    payload_identifier.reset()
    return jsonify({"status": "reset"}), 200

@app.route('/force/compensate', methods = ['GET'])
def force_compensate():
    """Enable/disable pose-dependent residual compensation (query: on=1/0).
    Requires an empty calibration pass first (/force/identify/empty). When on,
    /force and the RViz arrows use the residual of the nearest calibrated joint
    config as the baseline, so the reported wrench is the true EXTERNAL force
    (object weight + any contact) at the current pose, not just at a tare pose.
    Falls back to the tare baseline at poses far from the calibrated set."""
    if force_viz is None:
        return jsonify({"error": "force viz unavailable"}), 503
    on = request.args.get("on", "1") != "0"
    if not force_viz.set_compensation(on):
        return jsonify({
            "error": "no_calibration",
            "msg": "run /force/identify/empty first to record an empty-gripper calibration",
        }), 409
    return jsonify({"status": "compensation_on" if on else "compensation_off",
                    "compensated": on}), 200

@app.route('/health', methods = ['GET'])
def health():
    return jsonify({
        "move_server": bool(gripper_move_client.wait_for_server(rospy.Duration(0.1))),
        "grasp_server": bool(gripper_grasp_client.wait_for_server(rospy.Duration(0.1))),
        "lock_busy": moveit_lock.locked(),
        "last_error": last_error[-1] if last_error else None,
    }), 200

@app.route('/recover', methods = ['GET'])
def recover():
    ok = robot.recover()
    if not ok:
        return jsonify({"status": "timeout", "error": "recovery did not complete within timeout"}), 504
    return jsonify({"status": "recovered","msg": "Robot recovered"}), 200

@app.route('/control/stop', methods = ['GET'])
def stop():
    robot.stop()
    return jsonify({"status": "stopped","msg": "Robot stopped"}), 200
# driver function
if __name__ == '__main__':
    host = os.environ.get('FRANKA_RESTFUL_HOST', '172.26.0.212')
    port = int(os.environ.get('FRANKA_RESTFUL_PORT', '5000'))
    # Reliable shutdown: rospy/MoveIt spin non-daemon threads and, with the
    # debug reloader, there's a supervisor + worker process, so a plain Ctrl-C
    # often leaves the process hanging or the worker respawning. Kill the whole
    # process group (supervisor + worker + all threads) so Ctrl-C / kill stops
    # everything at once. The launching shell is in a different process group and
    # is not affected.
    def _hard_stop(signum=None, frame=None):
        try:
            os.killpg(os.getpgrp(), signal.SIGKILL)
        except Exception:
            os._exit(0)
    signal.signal(signal.SIGINT, _hard_stop)
    signal.signal(signal.SIGTERM, _hard_stop)

    log.info(f"Starting Flask on {host}:{port} (override with FRANKA_RESTFUL_HOST / FRANKA_RESTFUL_PORT)")
    try:
        app.run(host=host, port=port, debug=DEBUG)
    except SystemExit as e:
        # The debug reloader restarts the worker by exiting with code 3 -- let
        # that propagate so the supervisor respawns it. Killing the process
        # group here (the old behavior) murdered the whole server on every
        # code edit. Any other exit code is a real shutdown.
        if getattr(e, "code", None) == 3:
            raise
        _hard_stop()
    except BaseException:
        _hard_stop()
    else:
        # If werkzeug returns normally but rospy/roscpp threads linger, this
        # guarantees the process group still dies.
        _hard_stop()