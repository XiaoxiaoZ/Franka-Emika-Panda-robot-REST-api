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
    return jsonify({'msg': "box removed"})

@app.route('/simulation/add_box', methods = ['GET'])
def add_box():
    robot.add_box()
    return jsonify({'msg': "box added on the tool"})

@app.route('/simulation/detach_box', methods = ['GET'])
def detach_box():
    robot.detach_box()
    return jsonify({'msg': "box detached"})

@app.route('/simulation/attach_box', methods = ['GET'])
def attach_box():
    robot.attach_box()
    return jsonify({'msg': "box attached on the tool"})
    
def go_to_gripper_state():
    try:
        width = float(request.args.get("width"))
    except (TypeError, ValueError):
        return jsonify({"error": "No width arg"})
    try:
        result = robot.go_to_gripper_state(width)
        return jsonify({'result': result})
    except Exception as error:
        return jsonify({'error': error})      
@app.route('/control/go_to_gripper_state', methods = ['GET'])
def go_to_gripper_state_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return go_to_gripper_state()
    finally: moveit_lock.release()

def plan_cartesian_path():
    try:
        x = float(request.args.get("x"))
        y = float(request.args.get("y"))
        z = float(request.args.get("z"))
    except (TypeError, ValueError):
        return jsonify({"error": "Error xy args"})
    plan, result = robot.plan_cartesian_path(x, y, z)
    robot.display_trajectory(plan)
    if result<1.0:
        print("plan:",plan)
        return jsonify({"msg": "Plan not 100%","result": str(result)})
    #print("result:",result)
    else:
        success = robot.execute_plan(plan)
        print(success)
        if not success:
            return jsonify({"error": last_error or "Exception failed"})
        return jsonify({"msg": "Plan 100%", "result": str(result)})
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
        return jsonify({"error": "Error xy args"})
    (plan_success, plan, planning_time, error_code) = robot.plan_joint_path(x, y, z)
    robot.display_trajectory(plan)
    if not plan_success:
        return jsonify({"error_code": str(error_code), "palnning_time": str(planning_time)})
    success = robot.execute_plan(plan)
    print(success)
    if not success:
        return jsonify({"error": last_error or "Exception failed"})
    return jsonify({"msg": "Plan 100%", "planning_time": str(planning_time)})
@app.route('/control/plan_joint_path', methods = ['GET'])
def plan_joint_path_impl():
    if not moveit_lock.acquire(blocking=False):
        return jsonify({"error": "Robot is busy"}), 409
    try: return plan_joint_path()
    finally: moveit_lock.release()

@app.route('/control/stop', methods = ['GET'])
def stop():
    robot.stop()
    return jsonify({"msg": "Robot stopped"})
# driver function
if __name__ == '__main__':
    app.run(debug = True)