# FrankaServer RESTful API

This project provides a RESTful API for controlling the Franka Emika Panda robot using Flask and ROS.

## Features

- **Simulation Control**
  - Add, remove, attach, and detach a box in the simulation for path planning. 
- **Robot Control**
  - Move the gripper to a specified width.
  - Plan and execute Cartesian or joint paths.

## Endpoints

###

- `GET /state`
  Get robot state. Gripper 1 is open, 0 is closed.
  Example response:
  ```json
    {
    "gripper": 1,
    "orientation": {
      "w": 0.00028813095071412737,
      "x": -0.9999999068105678,
      "y": -9.515382186451469e-05,
      "z": -0.00030709145380621787
    },
    "position": {
      "x": 0.5000186889335273,
      "y": 0.3999107757901367,
      "z": 0.220021696649158
    }
  }

### Simulation

- `GET /simulation/add_box`  
  Adds a box to the tool.

- `GET /simulation/remove_box`  
  Removes the box.

- `GET /simulation/attach_box`  
  Attaches the box to the tool.

- `GET /simulation/detach_box`  
  Detaches the box.

### Control

- `GET /control/go_to_gripper_state?width=<float>`  
  Moves the gripper to the specified width.
  
- `GET /control/gripper_open`  
  Make gripper open.

- `GET /control/gripper_close`  
  Make gripper close.

- `GET /control/plan_cartesian_path?x=<float>&y=<float>&z=<float>`  
  Plans and executes a Cartesian path.

- `GET /control/plan_joint_path?x=<float>&y=<float>&z=<float>`  
  Plans and executes a joint path.

- `GET /control/stop`  
  Stop the ongoing execution of the robot and clean the path planned.

## Usage

1. **Start ROS and required nodes.**
2. **Run the server:**
   ```bash
   python3 python/restful.py
   ```
3. **Test endpoints:**
   ```bash
   curl "http://127.0.0.1:5000/control/go_to_gripper_state?width=0.01"
   ```

## Requirements

- Python 3
- Flask
- ROS (with `franka` package)

## Notes

- The API must be run in an environment with ROS/moveit and the Franka robot interface available.