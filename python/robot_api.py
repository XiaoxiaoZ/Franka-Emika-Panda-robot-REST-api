"""/robot/* -- read-only robot model facts for frame / calibration checks.
Never moves anything (safety rule S1).

    GET /robot/tf?from=panda_link0&to=panda_hand_tcp[&joints=q1,...,q7]
    GET /robot/description
"""
import hashlib
import math

from flask import Blueprint, jsonify, request

from scene_core import SceneError

bp = Blueprint("robot", __name__)
_robot = None
ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


def init(robot):
    global _robot
    _robot = robot


@bp.errorhandler(SceneError)
def _on_error(e):
    return jsonify({"error": e.code, "msg": e.msg, **e.extra}), e.status


def _mat(p):
    from tf.transformations import quaternion_matrix
    T = quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


@bp.route("/robot/tf", methods=["GET"])
def robot_tf():
    """Pose of link `to` in link `from` (both robot links), from the robot
    model's forward kinematics (/compute_fk, nominal kinematics). Without
    joints= it uses the current joint state; with joints=q1,...,q7 (rad) it
    evaluates that configuration instead. Returns {pose:{position,
    orientation:[qx,qy,qz,qw]}, rpy_deg, from, to, joints, stamp}."""
    import rospy
    from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
    from tf.transformations import quaternion_from_matrix, euler_from_matrix
    import numpy as np
    src = request.args.get("from", "panda_link0")
    dst = request.args.get("to", "panda_hand_tcp")
    links = set(_robot.robot.get_link_names())
    for name, v in (("from", src), ("to", dst)):
        if v not in links:
            raise SceneError(404, "unknown_object", f"{name}={v!r} is not a robot link")
    st = _robot.robot.get_current_state()
    st.is_diff = True
    names, pos = list(st.joint_state.name), list(st.joint_state.position)
    raw = request.args.get("joints")
    if raw is not None:
        try:
            q = [float(v) for v in raw.split(",")]
        except ValueError:
            raise SceneError(400, "bad_request", "joints must be 7 comma-separated numbers (rad)")
        if len(q) != 7 or any(math.isnan(v) or math.isinf(v) for v in q):
            raise SceneError(400, "bad_request", "joints must be 7 finite numbers (rad)")
        for n, v in zip(ARM_JOINTS, q):
            pos[names.index(n)] = v
        st.joint_state.position = pos
    try:
        rospy.wait_for_service("/compute_fk", timeout=3.0)
    except rospy.ROSException:
        raise SceneError(503, "move_group_unavailable", "/compute_fk is not available")
    fk = rospy.ServiceProxy("/compute_fk", GetPositionFK)
    req = GetPositionFKRequest()
    req.header.frame_id = _robot.move_group.get_planning_frame()
    req.fk_link_names = [src, dst]
    req.robot_state = st
    res = fk(req)
    if res.error_code.val != 1 or len(res.pose_stamped) != 2:
        raise SceneError(500, "fk_failed", f"compute_fk error {res.error_code.val}")
    T = np.linalg.inv(_mat(res.pose_stamped[0].pose)) @ _mat(res.pose_stamped[1].pose)
    q = quaternion_from_matrix(T)
    if q[3] < 0:
        q = -q
    r, p, y = euler_from_matrix(T)
    return jsonify({"from": src, "to": dst,
                    "pose": {"position": [float(v) for v in T[:3, 3]], "orientation": [float(v) for v in q]},
                    "rpy_deg": [math.degrees(r), math.degrees(p), math.degrees(y)],
                    "joints": [pos[names.index(n)] for n in ARM_JOINTS],
                    "stamp": res.pose_stamped[1].header.stamp.to_sec()}), 200


@bp.route("/robot/description", methods=["GET"])
def robot_description():
    """URDF and SRDF as loaded by move_group (+ sha256), joint limits of the
    arm, planning frame, groups, end-effector link and link names -- for
    offline FK/IK and frame checks."""
    import rospy
    from franka import PANDA_JOINT_LIMITS
    urdf = rospy.get_param("/robot_description", "")
    srdf = rospy.get_param("/robot_description_semantic", "")
    return jsonify({
        "urdf": urdf, "srdf": srdf,
        "urdf_sha256": hashlib.sha256(urdf.encode("utf-8")).hexdigest(),
        "srdf_sha256": hashlib.sha256(srdf.encode("utf-8")).hexdigest(),
        "joint_limits": {n: {"lower": lo, "upper": hi} for n, (lo, hi) in zip(ARM_JOINTS, PANDA_JOINT_LIMITS)},
        "planning_frame": _robot.move_group.get_planning_frame(),
        "groups": {"arm": _robot.move_group.get_name(), "hand": _robot.hand_group.get_name()},
        "ee_link": _robot.move_group.get_end_effector_link(),
        "links": _robot.robot.get_link_names(),
    }), 200
