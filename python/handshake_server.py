from fastapi import FastAPI, Request, Response
from fastapi.responses import JSONResponse
import threading
import time
import requests
import uvicorn
from contextlib import asynccontextmanager

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

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Start the control loop thread
    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()
    yield
    # Clean up if needed

app = FastAPI(lifespan=lifespan)

@app.api_route("/{variable}", methods=["GET", "POST"])
async def handle_variable(variable: str, request: Request):
    if variable not in state:
        return JSONResponse(content={"error": "Variable not found"}, status_code=404)

    if request.method == "GET":
        with lock:
            val = state[variable]
            if variable in ["vX", "vY", "vZ", "vGripperWidth"]:
                return float(val)
            elif variable in ["vStartOperation", "vStartOperationDone"]:
                return int(val)
            return val

    elif request.method == "POST":
        # Expecting raw string data like "value=1"
        body_bytes = await request.body()
        try:
            data = body_bytes.decode("utf-8")
        except UnicodeDecodeError:
            return JSONResponse(content={"error": "Invalid encoding"}, status_code=400)

        if not data.startswith("value="):
            return JSONResponse(content={"error": "Invalid format. Expected 'value=<val>'"}, status_code=400)
        
        try:
            value_str = data.split("value=")[1]
            if variable in ["vX", "vY", "vZ", "vGripperWidth"]:
                new_value = float(value_str)
            elif variable in ["vStartOperation", "vStartOperationDone"]:
                new_value = int(value_str)
            else:
                 return JSONResponse(content={"error": "Unknown variable type"}, status_code=500)
                 
            with lock:
                state[variable] = new_value
            
            return JSONResponse(content={"success": True, variable: new_value})
            
        except ValueError:
             return JSONResponse(content={"error": "Invalid value type"}, status_code=400)

if __name__ == "__main__":
    # Run on a different port to avoid conflict with restful.py (default 5000)
    uvicorn.run(app, host="172.26.0.212", port=5001)
