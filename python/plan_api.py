"""/plan/* -- plan-only checks. Nothing here moves the robot or the gripper
(safety rule S1): only move_group's stateless services are used
(/check_state_validity, /plan_kinematic_path, /compute_cartesian_path,
/compute_fk) -- never MoveGroupCommander.execute/go, never an action goal.
Because the shared MoveGroupCommander is not touched, these endpoints do
not take the motion lock and can run while the arm is moving.

    POST /plan/check_state   {joints:[7] | use_current, gripper_width_m?, expect_scene_version?}
    POST /plan/dry_run       {target:{type: pose|joints|cartesian, ...}, planner: RRTConnect|PTP|LIN,
                              start: "current" | {joints:[7]}, gripper_width_m?, velocity_scaling,
                              acceleration_scaling, allowed_planning_time_s, attempts,
                              expect_scene_version?, max_step_rad?}
"""
import copy
import json
import math
import threading
import time

from flask import Blueprint, jsonify, request

from scene_core import SceneError, canon_pose

bp = Blueprint("plan", __name__)
_robot = None
_core = None
_svc = {}
_plans = {}                 # plan_id -> (expires_at, record)
_plans_lock = threading.Lock()
_plan_seq = [0]
PLAN_TTL_S = 600
ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
FINGERS = ["panda_finger_joint1", "panda_finger_joint2"]
PLANNERS = {"RRTConnect": "ompl", "PTP": "pilz_industrial_motion_planner", "LIN": "pilz_industrial_motion_planner"}
TARGET_LIMIT_MARGIN_RAD = 0.02


def init(robot, core):
    global _robot, _core
    _robot, _core = robot, core


@bp.errorhandler(SceneError)
def _on_error(e):
    return jsonify({"error": e.code, "msg": e.msg, **e.extra}), e.status


def _err(status, code, msg, path=None, **extra):
    if path is not None:
        extra["path"] = path
    return SceneError(status, code, msg, **extra)


def _service(name, cls):
    import rospy
    if name not in _svc:
        try:
            rospy.wait_for_service(name, timeout=3.0)
        except rospy.ROSException:
            raise _err(503, "move_group_unavailable", f"{name} is not available")
        _svc[name] = rospy.ServiceProxy(name, cls)
    return _svc[name]


def _body():
    limit = 1024 * 1024
    raw = request.stream.read(limit + 1)
    if len(raw) > limit:
        raise _err(413, "too_large", "body exceeds 1 MB")
    try:
        d = json.loads(raw.decode("utf-8") or "{}")
    except (UnicodeDecodeError, ValueError) as e:
        raise _err(400, "bad_request", f"body is not valid JSON: {e}")
    if not isinstance(d, dict):
        raise _err(400, "bad_request", "body must be a JSON object")
    return d


def _check_scene_version(d):
    exp = d.get("expect_scene_version")
    if exp is not None and exp != _core.version:
        raise _err(409, "version_conflict", f"scene_version is {_core.version}, request expects {exp}",
                   scene_version=_core.version)


def _joints7(v, path):
    if not isinstance(v, list) or len(v) != 7:
        raise _err(400, "bad_request", "joints must be a list of 7 numbers (rad)", path)
    out = []
    for i, x in enumerate(v):
        if isinstance(x, bool) or not isinstance(x, (int, float)) or math.isnan(x) or math.isinf(x):
            raise _err(400, "bad_request", "joint values must be finite numbers (rad)", f"{path}[{i}]")
        out.append(float(x))
    return out


def _width(d, path="gripper_width_m"):
    w = d.get(path)
    if w is None:
        return None
    if isinstance(w, bool) or not isinstance(w, (int, float)) or not (0.0 <= w <= 0.08):
        raise _err(400, "bad_request", "gripper_width_m is the total opening, 0..0.08 m", path)
    return float(w)


def _state(joints=None, width=None):
    """Current RobotState (keeps attached objects) with optional overrides."""
    st = _robot.robot.get_current_state()
    names = list(st.joint_state.name)
    pos = list(st.joint_state.position)
    if joints is not None:
        for n, q in zip(ARM_JOINTS, joints):
            pos[names.index(n)] = q
    if width is not None:
        for n in FINGERS:
            if n in names:
                pos[names.index(n)] = width / 2.0
    st.joint_state.position = pos
    # MUST be a diff: RobotCommander's state carries no attached objects, and
    # a full (is_diff=False) state makes MoveIt drop every attached body --
    # wrist camera, grasped part -- from the check.
    st.is_diff = True
    st.attached_collision_objects = []
    return st


def _arm_of(state):
    lookup = dict(zip(state.joint_state.name, state.joint_state.position))
    return [lookup[n] for n in ARM_JOINTS]


def _validity(state):
    from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
    srv = _service("/check_state_validity", GetStateValidity)
    res = srv(GetStateValidityRequest(robot_state=state, group_name=_robot.move_group.get_name()))
    contacts = [{"a": c.contact_body_1, "b": c.contact_body_2, "depth_m": c.depth,
                 "point": [c.position.x, c.position.y, c.position.z]} for c in res.contacts]
    return bool(res.valid), contacts


def _limit_margin_ok(joints, margin=TARGET_LIMIT_MARGIN_RAD):
    from franka import check_joint_limits
    return not check_joint_limits(joints, margin_rad=margin)


# --------------------------------------------------------- check_state ----

@bp.route("/plan/check_state", methods=["POST"])
def check_state():
    """Collision-check one robot configuration against the current scene,
    without moving. JSON: joints:[7] (rad) or use_current:true,
    gripper_width_m (total opening 0..0.08; each finger gets half),
    expect_scene_version. Returns valid, contacts [{a, b, depth_m, point}],
    joint_limit_margin_ok (>0.02 rad from every limit), scene_version.
    No motion lock."""
    d = _body()
    _check_scene_version(d)
    joints = None if d.get("use_current") else _joints7(d.get("joints"), "joints")
    width = _width(d)
    st = _state(joints, width)
    valid, contacts = _validity(st)
    q = _arm_of(st)
    return jsonify({"valid": valid, "contacts": contacts, "joints": q,
                    "joint_limit_margin_ok": _limit_margin_ok(q),
                    "scene_version": _core.version, "scene_dirty": _core.dirty}), 200


# ------------------------------------------------------------- dry_run ----

def _pose_msg(p):
    from geometry_msgs.msg import Pose
    m = Pose()
    m.position.x, m.position.y, m.position.z = p["position"]
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = p["orientation"]
    return m


def _ik_near(pose, start_state, start_q):
    """Analytic IK (panda_ik, q7 sweep) closest to ``start_q`` that is clear of
    the limits and collision-free -- the same choice real motions make."""
    import panda_ik
    from tf.transformations import quaternion_matrix
    T = quaternion_matrix(pose["orientation"])
    T[:3, 3] = pose["position"]
    sols = panda_ik.solutions_near(T, start_q, weights=_robot.IK_JOINT_WEIGHTS,
                                   limit_margin=_robot.IK_LIMIT_MARGIN_RAD)
    for cost, q in sols[:40]:
        st = copy.deepcopy(start_state)
        names = list(st.joint_state.name)
        pos = list(st.joint_state.position)
        for n, v in zip(ARM_JOINTS, q):
            pos[names.index(n)] = v
        st.joint_state.position = pos
        valid, _ = _validity(st)
        if valid:
            return q, len(sols)
    return None, len(sols)


def _joint_goal(q):
    from moveit_msgs.msg import Constraints, JointConstraint
    c = Constraints()
    for n, v in zip(ARM_JOINTS, q):
        c.joint_constraints.append(JointConstraint(joint_name=n, position=v, tolerance_above=1e-4,
                                                   tolerance_below=1e-4, weight=1.0))
    return c


def _pose_goal(pose, link, frame):
    from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    c = Constraints()
    pc = PositionConstraint()
    pc.header.frame_id = frame
    pc.link_name = link
    bv = BoundingVolume()
    bv.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[1e-4]))
    cp = Pose()
    cp.position.x, cp.position.y, cp.position.z = pose["position"]
    cp.orientation.w = 1.0
    bv.primitive_poses.append(cp)
    pc.constraint_region = bv
    pc.weight = 1.0
    oc = OrientationConstraint()
    oc.header.frame_id = frame
    oc.link_name = link
    oc.orientation = _pose_msg(pose).orientation
    oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 1e-3
    oc.weight = 1.0
    c.position_constraints.append(pc)
    c.orientation_constraints.append(oc)
    return c


def _interp_check(traj_points, start_state, max_step_rad):
    """Densify the joint path to max_step_rad and collision-check every
    sample (move_group only checks the planner's own waypoints)."""
    names = list(start_state.joint_state.name)
    base = list(start_state.joint_state.position)
    idx = [names.index(n) for n in ARM_JOINTS]
    first_bad, contacts, checked = None, [], 0
    for k in range(len(traj_points) - 1):
        a, b = traj_points[k], traj_points[k + 1]
        n = max(1, int(math.ceil(max(abs(x - y) for x, y in zip(a, b)) / max_step_rad)))
        for j in range(n + 1 if k == len(traj_points) - 2 else n):
            t = j / n
            q = [x + (y - x) * t for x, y in zip(a, b)]
            st = copy.deepcopy(start_state)
            pos = list(base)
            for i, v in zip(idx, q):
                pos[i] = v
            st.joint_state.position = pos
            valid, c = _validity(st)
            checked += 1
            if not valid:
                return False, k, c, checked
    if len(traj_points) == 1:
        valid, c = _validity(start_state)
        return valid, (None if valid else 0), c, 1
    return True, None, [], checked


def _fk(q, link="panda_hand_tcp"):
    from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
    srv = _service("/compute_fk", GetPositionFK)
    st = _state(q)
    req = GetPositionFKRequest()
    req.header.frame_id = _robot.move_group.get_planning_frame()
    req.fk_link_names = [link]
    req.robot_state = st
    res = srv(req)
    if res.error_code.val != 1 or not res.pose_stamped:
        return None
    p = res.pose_stamped[0].pose
    return {"position": [p.position.x, p.position.y, p.position.z],
            "orientation": [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]}


@bp.route("/plan/dry_run", methods=["POST"])
def dry_run():
    """Plan only -- never executes. JSON: target {type: "pose", link
    (panda_hand_tcp), pose:{frame_id, position, orientation}} | {type:
    "joints", joints:[7]} | {type: "cartesian", waypoints:[pose...],
    eef_step_m (0.005)}; planner RRTConnect | PTP | LIN (pipeline set
    explicitly); start "current" | {joints:[7]}; gripper_width_m;
    velocity_scaling (0.2); acceleration_scaling (0.05);
    allowed_planning_time_s (5); attempts (3); max_step_rad (0.01, for
    the interpolated collision recheck); expect_scene_version. Pose
    targets for RRTConnect/PTP are converted to joints with the same
    nearest analytic-IK choice real motions use. Returns success,
    error_code, trajectory {joint_names, points [{t, q}]}, duration_s,
    joint_travel_deg, final_tcp_pose, would_be_rejected {target_near_limit,
    near_limit, too_large}, trajectory_valid + contacts (interpolated
    recheck), plan_id (kept 10 min). No motion lock."""
    from moveit_msgs.msg import MotionPlanRequest, RobotState
    from moveit_msgs.srv import GetMotionPlan, GetCartesianPath, GetCartesianPathRequest
    from franka import decode_moveit_error, check_trajectory_for_limits, trajectory_joint_travel
    t0 = time.time()
    d = _body()
    _check_scene_version(d)
    planner = d.get("planner", "RRTConnect")
    if planner not in PLANNERS:
        raise _err(400, "bad_request", f"planner must be one of {list(PLANNERS)}", "planner")
    pipeline = PLANNERS[planner]
    vel = float(d.get("velocity_scaling", 0.2))
    acc = float(d.get("acceleration_scaling", 0.05))
    if not (0.01 <= vel <= 1.0 and 0.01 <= acc <= 1.0):
        raise _err(400, "bad_request", "velocity/acceleration_scaling must be in 0.01..1")
    t_alloc = float(d.get("allowed_planning_time_s", 5.0))
    attempts = int(d.get("attempts", 3))
    max_step = float(d.get("max_step_rad", 0.01))
    if not (0.001 <= max_step <= 0.1):
        raise _err(400, "bad_request", "max_step_rad must be in 0.001..0.1")
    width = _width(d)
    start = d.get("start", "current")
    if start == "current":
        start_state = _state(None, width)
    elif isinstance(start, dict):
        start_state = _state(_joints7(start.get("joints"), "start.joints"), width)
    else:
        raise _err(400, "bad_request", 'start must be "current" or {joints:[7]}', "start")
    start_q = _arm_of(start_state)
    target = d.get("target")
    if not isinstance(target, dict) or target.get("type") not in ("pose", "joints", "cartesian"):
        raise _err(400, "bad_request", "target.type must be pose | joints | cartesian", "target.type")
    ttype = target["type"]
    link = target.get("link", _robot.move_group.get_end_effector_link())
    frame = _robot.move_group.get_planning_frame()
    resp = {"planner": planner, "pipeline": pipeline, "scene_version": _core.version,
            "target_type": ttype, "start_joints": start_q}
    start_valid, start_contacts = _validity(start_state)
    if not start_valid:
        resp.update(success=False, error_code="START_STATE_IN_COLLISION", start_contacts=start_contacts)
        return jsonify(resp), 200

    goal_q = None
    if ttype == "joints":
        goal_q = _joints7(target.get("joints"), "target.joints")
    elif ttype == "pose":
        pd = target.get("pose") or {}
        if (pd.get("frame_id") or "panda_link0") not in ("panda_link0", frame):
            raise _err(400, "bad_frame", "pose.frame_id must be panda_link0", "target.pose.frame_id")
        pose = canon_pose({k: v for k, v in pd.items() if k != "frame_id"}, "target.pose")
        if planner != "LIN":
            if link != "panda_hand_tcp":
                raise _err(400, "bad_request", "pose targets for RRTConnect/PTP must use link panda_hand_tcp")
            goal_q, n_sols = _ik_near(pose, start_state, start_q)
            if goal_q is None:
                resp.update(success=False, error_code="NO_IK_SOLUTION" if n_sols == 0 else "GOAL_IN_COLLISION",
                            ik_solutions_found=n_sols)
                return jsonify(resp), 200
    if goal_q is not None:
        resp["goal_joints"] = goal_q
        resp["would_be_rejected"] = {"target_near_limit": not _limit_margin_ok(goal_q)}
        gs = copy.deepcopy(start_state)
        names = list(gs.joint_state.name)
        pos = list(gs.joint_state.position)
        for n, v in zip(ARM_JOINTS, goal_q):
            pos[names.index(n)] = v
        gs.joint_state.position = pos
        g_valid, g_contacts = _validity(gs)
        if not g_valid:
            resp.update(success=False, error_code="GOAL_IN_COLLISION", goal_contacts=g_contacts)
            return jsonify(resp), 200

    fraction = None
    if ttype == "cartesian":
        wps = target.get("waypoints")
        if not isinstance(wps, list) or not wps:
            raise _err(400, "bad_request", "cartesian target needs waypoints:[pose...]", "target.waypoints")
        eef_step = float(target.get("eef_step_m", 0.005))
        srv = _service("/compute_cartesian_path", GetCartesianPath)
        creq = GetCartesianPathRequest()
        creq.header.frame_id = frame
        creq.start_state = start_state
        creq.group_name = _robot.move_group.get_name()
        creq.link_name = link
        creq.waypoints = [_pose_msg(canon_pose(w, f"target.waypoints[{i}]")) for i, w in enumerate(wps)]
        creq.max_step = eef_step
        creq.jump_threshold = 0.0
        creq.avoid_collisions = True
        cres = srv(creq)
        fraction = cres.fraction
        traj = cres.solution
        ok = cres.error_code.val == 1 and fraction >= 0.999
        code = decode_moveit_error(cres.error_code.val) if cres.error_code.val != 1 else (
            "SUCCESS" if ok else "PARTIAL_PATH")
    else:
        srv = _service("/plan_kinematic_path", GetMotionPlan)
        mreq = MotionPlanRequest()
        mreq.group_name = _robot.move_group.get_name()
        mreq.pipeline_id = pipeline
        mreq.planner_id = planner
        mreq.start_state = start_state
        mreq.num_planning_attempts = 1 if pipeline != "ompl" else max(1, attempts)
        mreq.allowed_planning_time = t_alloc
        mreq.max_velocity_scaling_factor = vel
        mreq.max_acceleration_scaling_factor = acc
        if goal_q is not None:
            mreq.goal_constraints = [_joint_goal(goal_q)]
        else:
            mreq.goal_constraints = [_pose_goal(pose, link, frame)]
        mres = srv(mreq).motion_plan_response
        traj = mres.trajectory
        ok = mres.error_code.val == 1
        code = "SUCCESS" if ok else decode_moveit_error(mres.error_code.val)
    resp.update(success=ok, error_code=code)
    if fraction is not None:
        resp["fraction"] = fraction
    pts = traj.joint_trajectory.points if ok else []
    if not ok:
        resp["elapsed_ms"] = int((time.time() - t0) * 1000)
        return jsonify(resp), 200

    jn = list(traj.joint_trajectory.joint_names)
    order = [jn.index(n) for n in ARM_JOINTS]
    path = [[p.positions[i] for i in order] for p in pts]
    resp["trajectory"] = {"joint_names": ARM_JOINTS,
                          "points": [{"t": p.time_from_start.to_sec(), "q": q} for p, q in zip(pts, path)]}
    resp["duration_s"] = pts[-1].time_from_start.to_sec() if pts else 0.0
    travel = trajectory_joint_travel(traj)
    resp["joint_travel_deg"] = {j: round(math.degrees(t), 2) for j, t in travel.items() if j in ARM_JOINTS}
    too_large = sorted(j for j, t in travel.items()
                       if t > _robot.MAX_JOINT_TRAVEL_RAD and j not in _robot.JOINT_TRAVEL_EXEMPT)
    near = check_trajectory_for_limits(traj, margin_rad=0.02)
    wbr = resp.setdefault("would_be_rejected", {})
    wbr.update(near_limit=bool(near), too_large=bool(too_large), too_large_joints=too_large)
    if near:
        wbr["near_limit_first"] = near[0]
    if goal_q is None and path:
        wbr["target_near_limit"] = not _limit_margin_ok(path[-1])
    resp["final_tcp_pose"] = _fk(path[-1]) if path else None
    valid, bad_seg, contacts, checked = _interp_check(path, start_state, max_step)
    resp.update(trajectory_valid=valid, contacts=contacts, interpolated_states_checked=checked)
    if not valid:
        resp["first_invalid_segment"] = bad_seg
    with _plans_lock:
        now = time.time()
        for k in [k for k, (exp, _) in _plans.items() if exp < now]:
            _plans.pop(k)
        _plan_seq[0] += 1
        pid = f"p-{_plan_seq[0]:06d}"
        _plans[pid] = (now + PLAN_TTL_S, {"scene_version": _core.version, "start_joints": start_q,
                                          "points": path, "planner": planner})
    resp["plan_id"] = pid
    resp["elapsed_ms"] = int((time.time() - t0) * 1000)
    return jsonify(resp), 200
