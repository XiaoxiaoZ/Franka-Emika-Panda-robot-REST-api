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
  Plan and execute a straight-line Cartesian path to the target.
  Optional query params shared with `plan_joint_path`:
  - `auto_recover=0/1` (default 1) — on execute failure, call `/franka_control/error_recovery` and retry
  - `max_retries=N` (default 1) — retries after recovery
  - `preserve_orientation=0/1` (default 1) — keep current wrist orientation vs. reset to canonical `(1,0,0,0)`. Set to 0 when you want the wrist to re-orient.
  Response includes `outcome`, `attempts`, `recovery_triggered`, `recovery_succeeded`, `fraction`, and (on failure) `last_error` and `moveit_error`.

- `GET /control/plan_joint_path?x=<float>&y=<float>&z=<float>`
  Plan and execute a joint-space path (Pilz LIN with OMPL fallback). Accepts the same auto-recovery and orientation query params as `plan_cartesian_path`.

- `GET /control/go_to_gripper_state?width=<float>`
  Move the gripper to the specified width (meters).

- `GET /control/gripper_open` / `GET /control/gripper_close`
  MoveIt-based gripper control — shortcut for opening (0.1 m) / closing (0.01 m). No force feedback.

- `GET /control/gripper_open_force?width=<m>&speed=<m/s>&timeout=<s>&auto_recover=0/1&max_retries=N`
  Force-based open via `/franka_gripper/move`. Timeout defaults to 10 s; returns 504 on timeout.
  Shares the recovery wrapper with `/control/gripper_grasp` (same query params, same response fields: `outcome`, `attempts`, `recovery_triggered`, `recovery_succeeded`).

- `GET /control/gripper_grasp?width=<m>&speed=<m/s>&force=<N>&eps_in=<m>&eps_out=<m>&timeout=<s>&auto_recover=0/1&max_retries=N`
  Force-based grasp via `/franka_gripper/grasp`. Use `width=0.0` to close until contact; widen `eps_out` (e.g. 0.08) for unknown object widths to get correct success reporting.
  On failure (timeout or action-reported failure), the wrapper optionally calls `/franka_control/error_recovery` and retries up to `max_retries` times.
  **For transit scenarios** (gripper is holding an object mid-transport and you're issuing a new gripper action), pass `max_retries=0` to guarantee no reset-and-retry that might disturb the grasp.
  Outcomes: `success`, `action_timeout` (504), `action_failed` (500), `recovery_failed` (500), `server_unreachable` (503).

- `GET /control/stop`
  Stop any ongoing motion and clear the current plan. Sets a stop flag that suppresses the auto-recovery retry, so the in-flight motion request returns `outcome: stopped` (not `execute_failed`) and does not resume.

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
- **Gripper auto-recovery** — implemented as a generic retry wrapper (`_gripper_action_call`). On failure or timeout, optionally calls `/franka_control/error_recovery` and retries the action. Default is one retry. **Context-awareness is the caller's responsibility:** for transit scenarios (gripper already holding an object mid-transport), pass `max_retries=0` to avoid a reset-and-retry that could disturb the grasp.
- **Simulation / scene API — basic.** `/simulation/add_box` hardcodes a single fixed-size cube; `self.box_name` is a single slot that `add_floor()` overwrites, which can cause `/simulation/remove_box` to remove the wrong object. No shape/size/pose parameters. Scene mutations don't take the motion lock. Needs a dedicated redesign pass.

## Notes

- The API must run in a shell with ROS sourced (`source /opt/ros/noetic/setup.bash` + your workspace).
- Motion endpoints serialize via a non-blocking lock — concurrent motion requests get 409 "Robot is busy" rather than queuing.
- Motions apply an orientation path constraint (±0.5 rad) to prevent mid-path IK flips. Constraint is suppressed when `preserve_orientation=0` (so the wrist is allowed to freely reorient to the new target).
- Motion endpoints auto-recover from reflex errors by default: on execute failure, `/franka_control/error_recovery` is called and the motion is retried once (configurable via `max_retries`). Recovery status is surfaced in the response (`recovery_triggered`, `recovery_succeeded`, `attempts`).
- Auto-recovery is suppressed for **user-initiated stops** (two kinds):
  - `GET /control/stop` → motion request returns `outcome: stopped`.
  - **Pushing the robot by hand** → triggers a `cartesian_reflex` or `joint_reflex`, which the wrapper classifies as external-force and reports as `outcome: stopped_external_force`. Robot does not resume when you let go.
  - Other reflex types (e.g. `joint_motion_generator_position_limits_violation`) are still auto-recovered because they represent controller-internal violations worth retrying.
