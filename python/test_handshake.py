import requests
import time
import threading
from flask import Flask, request, jsonify

# Configuration
HANDSHAKE_SERVER_URL = "http://127.0.0.1:5001"
MOCK_RESTFUL_PORT = 5000

# Mock restful server for testing
app = Flask(__name__)

@app.route('/control/plan_cartesian_path', methods=['GET'])
def plan_cartesian_path():
    x = request.args.get('x')
    y = request.args.get('y')
    z = request.args.get('z')
    print(f"[Mock REST] Received plan_cartesian_path: x={x}, y={y}, z={z}")
    return jsonify({"status": "success", "msg": "Plan executed"}), 200

@app.route('/control/go_to_gripper_state', methods=['GET'])
def go_to_gripper_state():
    width = request.args.get('width')
    print(f"[Mock REST] Received go_to_gripper_state: width={width}")
    return jsonify({"status": "success", "result": "Gripper moved"}), 200

def run_mock_server():
    app.run(port=MOCK_RESTFUL_PORT)

def test_handshake():
    # Start mock server in a separate thread
    flask_thread = threading.Thread(target=run_mock_server, daemon=True)
    flask_thread.start()
    
    # Wait for server to start
    time.sleep(2)
    
    print("\n--- Starting Test ---")
    
    # 1. Set variables
    print("Setting variables...")
    requests.post(f"{HANDSHAKE_SERVER_URL}/vX", data="value=0.5")
    requests.post(f"{HANDSHAKE_SERVER_URL}/vY", data="value=-0.2")
    requests.post(f"{HANDSHAKE_SERVER_URL}/vZ", data="value=0.4")
    requests.post(f"{HANDSHAKE_SERVER_URL}/vGripperWidth", data="value=0.02")
    
    # Verify variables set
    resp = requests.get(f"{HANDSHAKE_SERVER_URL}/vX").json()
    assert resp['vX'] == 0.5
    print("Variables set correctly.")

    # 2. Start Operation
    print("Starting operation...")
    requests.post(f"{HANDSHAKE_SERVER_URL}/vStartOperation", data="value=1")
    
    # Wait for operation to complete (handshake server should call mock server)
    for _ in range(10):
        resp = requests.get(f"{HANDSHAKE_SERVER_URL}/vStartOperationDone").json()
        if resp['vStartOperationDone'] == 1:
            print("Operation completed (vStartOperationDone == 1).")
            break
        time.sleep(0.5)
    else:
        print("Timeout waiting for operation completion.")
        return

    # 3. Reset Operation
    print("Resetting operation...")
    requests.post(f"{HANDSHAKE_SERVER_URL}/vStartOperation", data="value=0")
    
    # Wait for reset
    for _ in range(10):
        resp = requests.get(f"{HANDSHAKE_SERVER_URL}/vStartOperationDone").json()
        if resp['vStartOperationDone'] == 0:
            print("Reset completed (vStartOperationDone == 0).")
            break
        time.sleep(0.5)
    else:
        print("Timeout waiting for reset.")
        return

    print("\n--- Test Passed Successfully ---")

if __name__ == "__main__":
    test_handshake()
