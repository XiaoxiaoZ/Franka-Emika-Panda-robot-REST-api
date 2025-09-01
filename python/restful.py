# Using flask to make an api
# import necessary libraries and functions
from flask import Flask, jsonify, request
from threading import Lock

# Log
from franka import MoveGroupPythonInterfaceTutorial
import rospy
from rosgraph_msgs.msg import Log



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

robot.add_floor()

# on the terminal type: curl http://127.0.0.1:5000/
# returns hello world when we use GET.
# returns the data that we send when we use POST.
@app.route('/', methods = ['GET', 'POST'])
def home():
    if(request.method == 'GET'):

        data = "Hello, this is Franka Emika Panda"
        return jsonify({'data': data})
    
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
@app.route('/control/gripper_open', methods = ['GET'])
def gripper_open_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return gripper_open()
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

@app.route('/control/stop', methods = ['GET'])
def stop():
    robot.stop()
    return jsonify({"status": "stopped","msg": "Robot stopped"}), 200
# driver function
if __name__ == '__main__':
    app.run(debug = True)