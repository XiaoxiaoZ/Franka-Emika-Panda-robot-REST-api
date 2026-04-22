# FrankaServer RESTful API

A Flask-based RESTful API for controlling the Franka Emika Panda robot via ROS + MoveIt.

## Features

- **Robot Control** — gripper width, Cartesian paths, joint-space paths, emergency stop, error recovery
- **Planning-scene helpers** — add/remove/attach/detach a box for collision avoidance *(basic; see Current State below)*
- **Health probe** — quick `/health` check to diagnose a stuck server without restarting

## Endpoints

### State

- `GET /state`
  Get current pose and gripper state. `gripper`: 1 = open, 0 = closed.
  ```json
  {
    "gripper": 1,
    "orientation": { "x": ..., "y": ..., "z": ..., "w": ... },
    "position":    { "x": ..., "y": ..., "z": ... }
  }
  ```

- `GET /health`
  Probe action-server reachability and internal lock state. Use when the API feels stuck.
  ```json
  {
    "move_server": true,
    "grasp_server": true,
    "lock_busy": false,
    "last_error": null
  }
  ```

### Control

- `GET /control/plan_cartesian_path?x=<float>&y=<float>&z=<float>`
  Plan and execute a straight-line Cartesian path to the target. Returns 202 with a fraction if the plan is incomplete, 200 on full execution.

- `GET /control/plan_joint_path?x=<float>&y=<float>&z=<float>`
  Plan and execute a joint-space path (Pilz LIN with OMPL fallback).

- `GET /control/go_to_gripper_state?width=<float>`
  Move the gripper to the specified width (meters).

- `GET /control/gripper_open` / `GET /control/gripper_close`
  Shortcut for opening (0.1 m) / closing (0.01 m) the gripper.

- `GET /control/stop`
  Stop any ongoing motion and clear the current plan.

- `GET /recover`
  Trigger Franka automatic error recovery. Returns 504 if recovery doesn't complete within 15 s.

### Simulation (planning scene)

- `GET /simulation/add_box` / `GET /simulation/remove_box`
- `GET /simulation/attach_box` / `GET /simulation/detach_box`

See *Current State* below — this API is basic and has known limitations.

## Usage

1. **Start ROS + MoveIt** (one command brings up both the FCI driver and `move_group`):
   ```bash
   sudo -E roslaunch panda_moveit_config franka_control.launch robot_ip:=192.168.1.100
   ```
   - Uses `panda_moveit_config`'s `franka_control.launch`, **not** the one from the `franka_control` package (that one doesn't start `move_group`, and `restful.py` will hang on `waitForService`).
   - `sudo` is required because Franka's FCI needs realtime scheduling permissions.
   - Omit `robot_ip:=...` for simulation mode.

2. **Run the server:**
   ```bash
   python3 python/restful.py
   ```

3. **Call an endpoint:**
   ```bash
   curl -i "http://172.26.0.212:5000/health"
   curl -i "http://172.26.0.212:5000/control/plan_cartesian_path?x=0.3&y=0.4&z=0.3"
   ```

## Configuration

Server bind host and port are configurable via environment variables (defaults shown):

| Variable | Default | Notes |
|---|---|---|
| `FRANKA_RESTFUL_HOST` | `172.26.0.212` | Use `0.0.0.0` to bind all interfaces, `127.0.0.1` to lock to localhost |
| `FRANKA_RESTFUL_PORT` | `5000` | |

Example:
```bash
FRANKA_RESTFUL_HOST=0.0.0.0 python3 python/restful.py
```

## Requirements

- Python 3
- Flask
- ROS Noetic (or compatible) with:
  - `franka_ros` / `franka_control`
  - `panda_moveit_config`
  - `moveit_commander` (Python bindings)

## Current State

Some parts of this codebase are known-limited or known-broken. Expect follow-up work:

- **`python/handshake_server.py` — legacy.** A FastAPI state-variable bridge on port 5001. Poll-loops over HTTP back into `restful.py`. Its original use case may no longer apply; kept in place pending a decision to delete or rewrite.
- **Force-based gripper (`gripper_open1` / `gripper_close1` in `restful.py`) — broken.** Action-client-based gripper functions are currently unwired and have known runtime bugs. Don't wire them up without debugging first.
- **Simulation / scene API — basic.** `/simulation/add_box` hardcodes a single fixed-size cube; `self.box_name` is a single slot that `add_floor()` overwrites, which can cause `/simulation/remove_box` to remove the wrong object. No shape/size/pose parameters. Scene mutations don't take the motion lock. Needs a dedicated redesign pass.

## Notes

- The API must run in a shell with ROS sourced (`source /opt/ros/noetic/setup.bash` + your workspace).
- Motion endpoints serialize via a non-blocking lock — concurrent motion requests get 409 "Robot is busy" rather than queuing.
- Motions apply a path constraint keeping the end-effector aligned with the hardcoded target orientation (±0.5 rad tolerance) to prevent mid-path IK flips.
