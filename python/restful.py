# Using flask to make an api
# import necessary libraries and functions
import logging
import os
import signal
from flask import Flask, jsonify, request
from threading import Lock

# Log
from franka import MoveGroupPythonInterfaceTutorial, decode_moveit_error, check_joint_limits, is_user_stop_abort
from force_viz import ForceVisualizer
from payload_id import PayloadIdentifier
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

# Flask debug mode (and its auto-reloader). The reloader re-executes this
# module in a child process, so module-level singletons are built twice unless
# guarded -- see the force_viz guard below. Override with FRANKA_RESTFUL_DEBUG=0.
DEBUG = os.environ.get('FRANKA_RESTFUL_DEBUG', '1') != '0'

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
    "/state": "Current EE pose (meters, base frame) + orientation quaternion + gripper (1=open, 0=closed).",
    "/health": "Probe gripper action servers, motion-lock state, and last ROS error. Use when the API feels stuck.",
    "/recover": "Trigger Franka automatic error recovery (clears reflex errors). 504 if not done within 15 s.",
    "/control/stop": "Stop any ongoing motion and suppress its auto-recovery retry (in-flight request returns outcome=stopped).",
    "/control/home": "Joint-space move to the canonical safe pose. WARNING: moves the arm.",
    "/control/go_to_gripper_state": "MoveIt gripper to width=<m> (max 0.032 per finger). No force feedback.",
    "/control/gripper_open": "MoveIt gripper open shortcut (0.1 m). No force feedback.",
    "/control/gripper_close": "MoveIt gripper close shortcut (0.01 m). No force feedback.",
    "/control/gripper_open_force": "Force-based open via franka_gripper/move. Params: width=0.08, speed=0.1, timeout=10, auto_recover=1, max_retries=1.",
    "/control/gripper_grasp": "Force-based grasp via franka_gripper/grasp; closes until contact then applies force. Params: width=0.0, speed=0.05, force=20 (N), eps_in=0.005, eps_out=0.005 (widen to 0.08 for unknown widths), timeout=10, auto_recover=1, max_retries=1 (use max_retries=0 while transporting a held object).",
    "/control/plan_cartesian_path": "Straight-line (MoveL) move to ABSOLUTE x,y,z in meters. Params: auto_recover=1, max_retries=1, preserve_orientation=1 (0 lets the wrist reorient). WARNING: moves the arm.",
    "/control/plan_joint_path": "Joint-space plan+execute to x,y,z. Same params as plan_cartesian_path plus planner_id=LIN|RRTConnectkConfigDefault. WARNING: moves the arm.",
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
            "methods": sorted(m for m in rule.methods if m in ("GET", "POST")),
            "description": doc or _HELP_NOTES.get(rule.rule, ""),
        })
    endpoints.sort(key=lambda e: e["path"])
    return jsonify({
        "name": "Franka Emika Panda REST API",
        "quickstart": [
            "GET /state to read the robot pose and gripper state.",
            "GET /force for the live end-effector force/torque estimate.",
            "GET /control/plan_cartesian_path?x=&y=&z= to move (meters, absolute; MOVES THE ARM).",
            "GET /control/gripper_grasp?force=20 to grasp; /control/gripper_open_force to open.",
            "GET /control/stop to stop motion; GET /recover to clear reflex errors.",
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
    return jsonify(state_dict), 200
    
@app.route('/simulation/remove_box', methods = ['GET'])
def remove_box():
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

def _parse_motion_args():
    """Common parse for motion endpoints.
    Returns (x, y, z, auto_recover, max_retries, preserve_orientation) or raises."""
    x = float(request.args.get("x"))
    y = float(request.args.get("y"))
    z = float(request.args.get("z"))
    auto_recover = request.args.get("auto_recover", "1") != "0"
    max_retries = int(request.args.get("max_retries", "1"))
    preserve_orientation = request.args.get("preserve_orientation", "1") != "0"
    return x, y, z, auto_recover, max_retries, preserve_orientation


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
        body["hint"] = "plan would drive a joint near its limit mid-path; try a different target, preserve_orientation=0, or plan_joint_path with planner_id=RRTConnectkConfigDefault"
        return jsonify({**body, "msg": "plan rejected: would drift a joint past safe margin"}), 409
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


def _do_cartesian(x, y, z, auto_recover, max_retries, preserve_orientation):
    ok, rejection = _preflight_joint_limit_check()
    if not ok:
        return rejection

    def plan_fn():
        plan, fraction = robot.plan_cartesian_path(x, y, z, preserve_orientation=preserve_orientation)
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
    )
    extra = {"fraction": str(outcome['plan_metadata'].get('fraction'))}
    if outcome['outcome'] == 'plan_failed':
        extra["moveit_error"] = f"PARTIAL_PATH (fraction={outcome['plan_metadata'].get('fraction')}, likely unreachable target or constraint too tight)"
    return _format_motion_response(outcome, extra)


def plan_cartesian_path():
    try:
        x, y, z, auto_recover, max_retries, preserve_orientation = _parse_motion_args()
    except (TypeError, ValueError):
        return jsonify({"error": "Error xy args"}), 400

    global _last_motion
    _last_motion = {
        'kind': 'cartesian', 'x': x, 'y': y, 'z': z,
        'auto_recover': auto_recover, 'max_retries': max_retries,
        'preserve_orientation': preserve_orientation,
    }

    return _do_cartesian(x, y, z, auto_recover, max_retries, preserve_orientation)


@app.route('/control/plan_cartesian_path', methods = ['GET'])
def plan_cartesian_path_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return plan_cartesian_path()
    finally: moveit_lock.release()


def _do_joint(x, y, z, auto_recover, max_retries, preserve_orientation, planner_id="LIN"):
    ok, rejection = _preflight_joint_limit_check()
    if not ok:
        return rejection

    def plan_fn():
        (plan_success, plan, planning_time, error_code) = robot.plan_joint_path(
            x, y, z, preserve_orientation=preserve_orientation, planner_id=planner_id,
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
    )
    extra = {"planning_time": str(outcome['plan_metadata'].get('planning_time'))}
    if outcome['outcome'] == 'plan_failed':
        extra["moveit_error"] = decode_moveit_error(outcome['plan_metadata'].get('error_code'))
    return _format_motion_response(outcome, extra)


def plan_joint_path():
    try:
        x, y, z, auto_recover, max_retries, preserve_orientation = _parse_motion_args()
        planner_id = request.args.get("planner_id", "LIN")
    except (TypeError, ValueError):
        return jsonify({"error": "Error xy args"}), 400

    global _last_motion
    _last_motion = {
        'kind': 'joint', 'x': x, 'y': y, 'z': z,
        'auto_recover': auto_recover, 'max_retries': max_retries,
        'preserve_orientation': preserve_orientation,
        'planner_id': planner_id,
    }

    return _do_joint(x, y, z, auto_recover, max_retries, preserve_orientation, planner_id=planner_id)


@app.route('/control/plan_joint_path', methods = ['GET'])
def plan_joint_path_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return plan_joint_path()
    finally: moveit_lock.release()
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
                                 lm['auto_recover'], lm['max_retries'], lm['preserve_orientation'])
        if lm['kind'] == 'joint':
            return _do_joint(lm['x'], lm['y'], lm['z'],
                             lm['auto_recover'], lm['max_retries'], lm['preserve_orientation'],
                             planner_id=lm.get('planner_id', 'LIN'))
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