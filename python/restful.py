# Using flask to make an api
# import necessary libraries and functions
import logging
import os
from flask import Flask, jsonify, request
from threading import Lock

# Log
from franka import MoveGroupPythonInterfaceTutorial, decode_moveit_error
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

# on the terminal type: curl http://127.0.0.1:5000/
# returns hello world when we use GET.
# returns the data that we send when we use POST.
@app.route('/', methods = ['GET', 'POST'])
def home():
    if(request.method == 'GET'):

        data = "Hello, this is Franka Emika Panda"
        return jsonify({'data': data})
    
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
    
def gripper_open1():
    """
    Open the gripper using franka_gripper/move.
    Optional query params:
      width: target opening in meters (default 0.08)
      speed: opening speed in m/s (default 0.1)
      timeout: seconds to wait for result (default 10.0)
    """
    try:
        width = float(request.args.get("width", "0.08"))  # Panda max ≈ 0.08 m
        speed = float(request.args.get("speed", "0.1"))
        timeout_s = float(request.args.get("timeout", "10.0"))
    except (TypeError, ValueError):
        return jsonify({"error": "Bad width/speed/timeout params"}), 400

    if not gripper_move_client.wait_for_server(rospy.Duration(1.0)):
        return jsonify({"error": "franka_gripper/move action server not reachable"}), 503

    goal = MoveGoal()
    goal.width = width
    goal.speed = speed

    gripper_move_client.send_goal(goal)
    finished = gripper_move_client.wait_for_result(rospy.Duration(timeout_s))
    if not finished:
        gripper_move_client.cancel_goal()
        return jsonify({"error": f"franka_gripper move timed out after {timeout_s}s"}), 504
    result = gripper_move_client.get_result()

    ok = result is not None and bool(getattr(result, "success", False))
    return jsonify({
        "success": ok,
        "error_msg": None if ok else getattr(result, "error", "no result"),
        "final_width_cmd": float(width),
    }), 200 if ok else 500


def gripper_close1():
    """
    Close the gripper until it senses an object, using franka_gripper/grasp.

    Optional query params:
      width: expected object width (m). Use 0.0 if unknown (close fully until contact)
      speed: closing speed (m/s, default 0.05)
      force: grip force (N, default 20.0)
      eps_in: epsilon.inner, allowed inner tolerance (default 0.005)
      eps_out: epsilon.outer, allowed outer tolerance (default 0.005)
      timeout: seconds to wait for result (default 10.0)
    """
    try:
        width = float(request.args.get("width", "0.0"))
        speed = float(request.args.get("speed", "0.05"))
        force = float(request.args.get("force", "20.0"))
        eps_in = float(request.args.get("eps_in", "0.005"))
        eps_out = float(request.args.get("eps_out", "0.005"))
        timeout_s = float(request.args.get("timeout", "10.0"))
    except (TypeError, ValueError):
        return jsonify({"error": "Bad grasp params"}), 400

    if not gripper_grasp_client.wait_for_server(rospy.Duration(1.0)):
        return jsonify({"error": "franka_gripper/grasp action server not reachable"}), 503

    goal = GraspGoal()
    goal.width = width
    goal.speed = speed
    goal.force = force
    goal.epsilon.inner = eps_in
    goal.epsilon.outer = eps_out

    gripper_grasp_client.send_goal(goal)
    finished = gripper_grasp_client.wait_for_result(rospy.Duration(timeout_s))
    if not finished:
        gripper_grasp_client.cancel_goal()
        return jsonify({"error": f"franka_gripper grasp timed out after {timeout_s}s"}), 504
    result = gripper_grasp_client.get_result()

    ok = result is not None and bool(getattr(result, "success", False))
    return jsonify({
        "success": ok,
        "error_msg": None if ok else getattr(result, "error", "no result"),
        "used_width_cmd": float(width),
        "force": float(force)
    }), 200 if ok else 500

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
    if outcome['outcome'] == 'plan_failed':
        body["last_error"] = last_error[-1] if last_error else None
        return jsonify({**body, "msg": "Plan not 100%"}), 202
    if outcome['outcome'] == 'execute_failed':
        body["last_error"] = last_error[-1] if last_error else None
        return jsonify({**body, "msg": "execution failed"}), 500
    if outcome['outcome'] == 'recovery_failed':
        body["last_error"] = last_error[-1] if last_error else None
        return jsonify({**body, "msg": "execute failed and recovery also failed"}), 500
    return jsonify({**body, "msg": "unknown outcome"}), 500


def plan_cartesian_path():
    try:
        x, y, z, auto_recover, max_retries, preserve_orientation = _parse_motion_args()
    except (TypeError, ValueError):
        return jsonify({"error": "Error xy args"}), 400

    def plan_fn():
        plan, fraction = robot.plan_cartesian_path(x, y, z, preserve_orientation=preserve_orientation)
        return plan, fraction >= 1.0, {"fraction": fraction}

    outcome = robot.plan_and_execute_with_retry(plan_fn, auto_recover=auto_recover, max_retries=max_retries)
    extra = {"fraction": str(outcome['plan_metadata'].get('fraction'))}
    if outcome['outcome'] == 'plan_failed':
        extra["moveit_error"] = f"PARTIAL_PATH (fraction={outcome['plan_metadata'].get('fraction')}, likely unreachable target or constraint too tight)"
    return _format_motion_response(outcome, extra)


@app.route('/control/plan_cartesian_path', methods = ['GET'])
def plan_cartesian_path_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return plan_cartesian_path()
    finally: moveit_lock.release()


def plan_joint_path():
    try:
        x, y, z, auto_recover, max_retries, preserve_orientation = _parse_motion_args()
    except (TypeError, ValueError):
        return jsonify({"error": "Error xy args"}), 400

    def plan_fn():
        (plan_success, plan, planning_time, error_code) = robot.plan_joint_path(x, y, z, preserve_orientation=preserve_orientation)
        return plan, bool(plan_success), {"planning_time": planning_time, "error_code": error_code}

    outcome = robot.plan_and_execute_with_retry(plan_fn, auto_recover=auto_recover, max_retries=max_retries)
    extra = {"planning_time": str(outcome['plan_metadata'].get('planning_time'))}
    if outcome['outcome'] == 'plan_failed':
        extra["moveit_error"] = decode_moveit_error(outcome['plan_metadata'].get('error_code'))
    return _format_motion_response(outcome, extra)


@app.route('/control/plan_joint_path', methods = ['GET'])
def plan_joint_path_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return plan_joint_path()
    finally: moveit_lock.release()
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
    log.info(f"Starting Flask on {host}:{port} (override with FRANKA_RESTFUL_HOST / FRANKA_RESTFUL_PORT)")
    app.run(host=host, port=port, debug=True)