# Using flask to make an api
# import necessary libraries and functions
from flask import Flask, jsonify, request
from threading import Lock

# Log
from franka import MoveGroupPythonInterfaceTutorial
import rospy
from rosgraph_msgs.msg import Log

import actionlib
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal


last_error = []
def rosout_cd(msg):
    global last_error
    if msg.level>=8:
        last_error.append(f"[{msg.header.stamp.secs}] {msg.name}: {msg.msg}")
rospy.Subscriber("/rosout", Log, rosout_cd)

# creating a Flask app
app = Flask(__name__)

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
    if gripper_width>0.035:
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
        return jsonify({'error': error}), 500      
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
        return jsonify({'error': error}), 500   

def gripper_close():
    try:
        result = robot.go_to_gripper_state(0.01)
        return jsonify({'result': result}), 200
    except Exception as error:
        return jsonify({'error': error}), 500   
    
def gripper_open1():
    """
    Open the gripper using franka_gripper/move.
    Optional query params:
      width: target opening in meters (default 0.08)
      speed: opening speed in m/s (default 0.1)
    """
    try:
        width = float(request.args.get("width", "0.08"))  # Panda max ≈ 0.08 m
        speed = float(request.args.get("speed", "0.1"))
    except (TypeError, ValueError):
        return jsonify({"error": "Bad width/speed params"}), 400

    goal = MoveGoal()
    goal.width = width
    goal.speed = speed

    try:
        gripper_move_client.send_goal(goal)
        gripper_move_client.wait_for_result()
        result = gripper_move_client.get_result()
    except Exception as e:
        return jsonify({"error": f"franka_gripper move failed: {e}"}), 500

    return jsonify({
        "success": bool(getattr(result, "success", True)),
        "final_width_cmd": float(width)
    }), 200


def gripper_close1():
    """
    Close the gripper until it senses an object, using franka_gripper/grasp.

    Optional query params:
      width: expected object width (m). Use 0.0 if unknown (close fully until contact)
      speed: closing speed (m/s, default 0.05)
      force: grip force (N, default 20.0)
      eps_in: epsilon.inner, allowed inner tolerance (default 0.005)
      eps_out: epsilon.outer, allowed outer tolerance (default 0.005)
    """
    try:
        width = float(request.args.get("width", "0.0"))
        speed = float(request.args.get("speed", "0.05"))
        force = float(request.args.get("force", "20.0"))
        eps_in = float(request.args.get("eps_in", "0.005"))
        eps_out = float(request.args.get("eps_out", "0.005"))
    except (TypeError, ValueError):
        return jsonify({"error": "Bad grasp params"}), 400

    goal = GraspGoal()
    goal.width = width
    goal.speed = speed
    goal.force = force
    goal.epsilon.inner = eps_in
    goal.epsilon.outer = eps_out

    try:
        gripper_grasp_client.send_goal(goal)
        gripper_grasp_client.wait_for_result()
        result = gripper_grasp_client.get_result()
    except Exception as e:
        return jsonify({"error": f"franka_gripper grasp failed: {e}"}), 500

    return jsonify({
        "success": bool(getattr(result, "success", True)),
        "used_width_cmd": float(width),
        "force": float(force)
    }), 200
     
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

def plan_cartesian_path():
    try:
        x = float(request.args.get("x"))
        y = float(request.args.get("y"))
        z = float(request.args.get("z"))
    except (TypeError, ValueError):
        return jsonify({"error": "Error xy args"}), 400
    plan, result = robot.plan_cartesian_path(x, y, z)
    robot.display_trajectory(plan)
    if result<1.0:
        print("plan:",plan)
        return jsonify({"msg": "Plan not 100%","result": str(result)}), 202
    #print("result:",result)
    else:
        success = robot.execute_plan(plan)
        print(success)
        if not success:
            return jsonify({"error": last_error or "Exception failed"}), 500
        return jsonify({"msg": "Plan 100%", "result": str(result)}), 200
@app.route('/control/plan_cartesian_path', methods = ['GET'])
def plan_cartesian_path_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return plan_cartesian_path()
    finally: moveit_lock.release()
    
def plan_joint_path():
    try:
        x = float(request.args.get("x"))
        y = float(request.args.get("y"))
        z = float(request.args.get("z"))
    except (TypeError, ValueError):
        return jsonify({"error": "Error xy args"}), 400
    (plan_success, plan, planning_time, error_code) = robot.plan_joint_path(x, y, z)
    robot.display_trajectory(plan)
    if not plan_success:
        return jsonify({"error_code": str(error_code), "palnning_time": str(planning_time)}), 202
    success = robot.execute_plan(plan)
    print(success)
    if not success:
        return jsonify({"error": last_error or "Exception failed"}), 500
    return jsonify({"msg": "Plan 100%", "planning_time": str(planning_time)}), 200
@app.route('/control/plan_joint_path', methods = ['GET'])
def plan_joint_path_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return plan_joint_path()
    finally: moveit_lock.release()
@app.route('/recover', methods = ['GET'])
def recover():
    robot.recover()
    return jsonify({"status": "recovered","msg": "Robot recovered"}), 200

@app.route('/control/stop', methods = ['GET'])
def stop():
    robot.stop()
    return jsonify({"status": "stopped","msg": "Robot stopped"}), 200
# driver function
if __name__ == '__main__':
    app.run(host="192.168.0.102", port=5000, debug = True)