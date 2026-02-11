from flask import Flask, jsonify, request
import threading
import time
import requests

app = Flask(__name__)

# State variables
state = {
    "vX": 0.0,
    "vY": 0.0,
    "vZ": 0.0,
    "vGripperWidth": 0.0,
    "vStartOperation": 0,
    "vStartOperationDone": 0
}

# Lock for thread safety
lock = threading.Lock()

# Configuration for the existing restful server
RESTFUL_SERVER_URL = "http://172.26.0.212:5000"

def control_loop():
    """Background thread to monitor vStartOperation and trigger robot actions."""
    while True:
        with lock:
            start_op = state["vStartOperation"]
            start_op_done = state["vStartOperationDone"]
            
            # Trigger operation
            if start_op == 1 and start_op_done == 0:
                print("Starting operation...")
                try:
                    # 1. Move robot to (vX, vY, vZ)
                    x, y, z = state["vX"], state["vY"], state["vZ"]
                    print(f"Moving robot to x={x}, y={y}, z={z}")
                    resp_move = requests.get(f"{RESTFUL_SERVER_URL}/control/plan_joint_path", params={"x": x, "y": y, "z": z})
                    print(f"Move response: {resp_move.status_code} - {resp_move.text}")
                    
                    # 2. Set gripper width
                    width = state["vGripperWidth"]
                    print(f"Setting gripper width to {width}")
                    resp_grip = requests.get(f"{RESTFUL_SERVER_URL}/control/go_to_gripper_state", params={"width": width})
                    print(f"Gripper response: {resp_grip.status_code} - {resp_grip.text}")
                    
                    # Mark operation as done
                    state["vStartOperationDone"] = 1
                    print("Operation done. Waiting for reset...")
                    
                except requests.exceptions.RequestException as e:
                    print(f"Error communicating with restful server: {e}")
                except Exception as e:
                    print(f"Unexpected error: {e}")

            # Reset operation
            elif start_op == 0 and start_op_done == 1:
                print("Resetting vStartOperationDone to 0")
                state["vStartOperationDone"] = 0
        
        time.sleep(0.1)  # Sleep to prevent high CPU usage

# Start the control loop thread
thread = threading.Thread(target=control_loop, daemon=True)
thread.start()

@app.route('/<variable>', methods=['GET', 'POST'])
def handle_variable(variable):
    if variable not in state:
        return jsonify({"error": "Variable not found"}), 404

    if request.method == 'GET':
        with lock:
            return jsonify({variable: state[variable]})

    elif request.method == 'POST':
        # Expecting raw string data like "value=1"
        data = request.get_data(as_text=True)
        if not data.startswith("value="):
            return jsonify({"error": "Invalid format. Expected 'value=<val>'"}), 400
        
        try:
            value_str = data.split("value=")[1]
            if variable in ["vX", "vY", "vZ", "vGripperWidth"]:
                new_value = float(value_str)
            elif variable in ["vStartOperation", "vStartOperationDone"]:
                new_value = int(value_str)
            else:
                 return jsonify({"error": "Unknown variable type"}), 500
                 
            with lock:
                state[variable] = new_value
            
            return jsonify({"success": True, variable: new_value})
            
        except ValueError:
             return jsonify({"error": "Invalid value type"}), 400

if __name__ == '__main__':
    # Run on a different port to avoid conflict with restful.py (default 5000)
    app.run(host="0.0.0.0", port=5001, debug=True)
