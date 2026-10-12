"""Acceptance tests T12-T17, T21 (+ legacy attach = Q2, S1) against the SIM dev server."""
import json, math, os, time, urllib.error, urllib.request
import numpy as np

B = "http://127.0.0.1:5100"
assert os.environ.get("ROS_MASTER_URI") == "http://127.0.0.1:11312", "source sim/env.sh first"
S = os.environ.get("SIM_DIR", "/tmp/franka_sim")
results = []


def req(method, path, body=None, timeout=120):
    data = json.dumps(body).encode() if body is not None else None
    r = urllib.request.Request(B + path, data=data, method=method,
                               headers={"Content-Type": "application/json"} if body is not None else {})
    try:
        with urllib.request.urlopen(r, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read() or b"{}")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def check(tid, cond, detail=""):
    results.append((tid, bool(cond)))
    print(f"{'PASS' if cond else 'FAIL'} {tid}: {detail}")


import rospy
from moveit_msgs.msg import ExecuteTrajectoryActionGoal, PlanningSceneComponents
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPlanningScene
from tf.transformations import quaternion_matrix, quaternion_from_euler
rospy.init_node("plan_p0_test", anonymous=True, disable_signals=True)
exec_goals = []
rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, lambda m: exec_goals.append(m))
fk = rospy.ServiceProxy("/compute_fk", GetPositionFK)
gps = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
time.sleep(1.0)
HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


def fk_pose(q, link):
    st = gps(PlanningSceneComponents(components=PlanningSceneComponents.ROBOT_STATE)).scene.robot_state
    names = list(st.joint_state.name); pos = list(st.joint_state.position)
    for i, v in enumerate(q):
        pos[names.index(f"panda_joint{i+1}")] = v
    st.joint_state.position = pos
    r = fk(GetPositionFKRequest(fk_link_names=[link], robot_state=st,
                                header=type(GetPositionFKRequest().header)(frame_id="panda_link0")))
    p = r.pose_stamped[0].pose
    return np.array([p.position.x, p.position.y, p.position.z]), [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]


def joints_now():
    return req("GET", "/joints")[1]["positions"]


q_before = joints_now()

# T21: restart (done by the caller) restored the scene with the same hash
saved = json.load(open(os.path.join(S, "hash_before_restart.json")))["scene_hash"]
info = req("GET", "/scene/info")[1]
names = {co.id for co in gps(PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_NAMES)).scene.world.collision_objects}
check("T21", info["scene_hash"] == saved and not info["dirty"] and {"pipefit.test.bin", "pipefit.test.tube", "pipefit.test.cube"} <= names,
      f"hash equal {info['scene_hash'] == saved}, dirty {info['dirty']}, objects back {sorted(n for n in names if n.startswith('pipefit'))}")
check("CAMERA_ATTACHED", bool(info.get("wrist_camera")),
      f"wrist_camera size {info.get('wrist_camera', {}).get('size')} centre {info.get('wrist_camera', {}).get('center')}")

# T12: home is valid (with the camera box on the hand)
st, r = req("POST", "/plan/check_state", {"joints": HOME, "gripper_width_m": 0.08})
check("T12", st == 200 and r["valid"] and r["joint_limit_margin_ok"], f"valid={r.get('valid')} contacts={r.get('contacts')}")

# T13: a box at link7 makes home invalid with (panda_link7, blocker)
p7, q7 = fk_pose(HOME, "panda_link7")
p7 = p7 + quaternion_matrix(q7)[:3, :3] @ np.array([0, 0, 0.06])   # link7's body (its frame origin is in a gap)
blocker = {"id": "pipefit.test.blocker", "pose": {"position": list(p7), "orientation": [0, 0, 0, 1]},
           "shapes": [{"type": "box", "size": [0.04, 0.04, 0.04]}]}
req("POST", "/scene/batch", {"namespace": "pipefit.test", "mode": "upsert", "objects": [blocker]})
st, r = req("POST", "/plan/check_state", {"joints": HOME})
pairs = {tuple(sorted((c["a"], c["b"]))) for c in r.get("contacts", [])}
check("T13", st == 200 and not r["valid"] and ("panda_link7", "pipefit.test.blocker") in pairs, f"valid={r.get('valid')} pairs={sorted(pairs)}")
req("POST", "/scene/batch", {"namespace": "pipefit.test", "mode": "remove", "ids": ["pipefit.test.blocker"]})

# T14: pose 0.1 m above the cube (cube centre [0.45, 0, 0.30]) with each planner
down = [1.0, 0.0, 0.0, 0.0]
target = {"type": "pose", "link": "panda_hand_tcp", "pose": {"frame_id": "panda_link0", "position": [0.45, 0.0, 0.40], "orientation": down}}
for planner, pipe in (("RRTConnect", "ompl"), ("PTP", "pilz_industrial_motion_planner"), ("LIN", "pilz_industrial_motion_planner")):
    st, r = req("POST", "/plan/dry_run", {"target": target, "planner": planner, "start": {"joints": HOME}})
    fp = r.get("final_tcp_pose") or {}
    err = np.linalg.norm(np.array(fp.get("position", [9, 9, 9])) - np.array([0.45, 0, 0.40])) if fp else 9
    check(f"T14-{planner}", st == 200 and r.get("success") and r.get("pipeline") == pipe and r.get("trajectory_valid")
          and len(r["trajectory"]["points"]) > 1 and err < 0.002,
          f"success={r.get('success')} {r.get('error_code')} pipeline={r.get('pipeline')} pts={len((r.get('trajectory') or {}).get('points', []))} "
          f"dur={r.get('duration_s', 0):.2f}s tcp_err={err*1000:.2f}mm travel_max={max((r.get('joint_travel_deg') or {'x': 0}).values())}deg "
          f"checked={r.get('interpolated_states_checked')} {r.get('elapsed_ms')}ms")
q_after = joints_now()
check("T14-no-motion", max(abs(a - b) for a, b in zip(q_before, q_after)) < 1e-4 and len(exec_goals) == 0,
      f"max joint change {max(abs(a - b) for a, b in zip(q_before, q_after)):.2e} rad, execute goals {len(exec_goals)}")

# T15: target inside the cube
inside = dict(target, pose=dict(target["pose"], position=[0.45, 0.0, 0.30]))
# gripper closed: open fingers (80 mm) would straddle the 50 mm cube and the pose would really be free
st, r = req("POST", "/plan/dry_run", {"target": inside, "planner": "RRTConnect", "start": {"joints": HOME}, "gripper_width_m": 0.0})
check("T15", st == 200 and r.get("success") is False and r.get("error_code") == "GOAL_IN_COLLISION",
      f"success={r.get('success')} {r.get('error_code')} ik_solutions_found={r.get('ik_solutions_found')}")

# T16: a 3 mm plate across the middle of a PTP sweep (J1 +0.8 rad)
goal = list(HOME); goal[0] += 0.8
mid = list(HOME); mid[0] += 0.4
pm, _ = fk_pose(mid, "panda_hand_tcp")
tang = np.array([-pm[1], pm[0], 0.0]); tang /= np.linalg.norm(tang)
yaw = math.atan2(tang[1], tang[0])
plate = {"id": "pipefit.test.plate", "pose": {"position": list(pm), "orientation": list(quaternion_from_euler(0, 0, yaw))},
         "shapes": [{"type": "box", "size": [0.003, 0.25, 0.30]}]}
st0, r0 = req("POST", "/plan/dry_run", {"target": {"type": "joints", "joints": goal}, "planner": "PTP", "start": {"joints": HOME}})
req("POST", "/scene/batch", {"namespace": "pipefit.test", "mode": "upsert", "objects": [plate]})
st, r = req("POST", "/plan/dry_run", {"target": {"type": "joints", "joints": goal}, "planner": "PTP", "start": {"joints": HOME}})
req("POST", "/scene/batch", {"namespace": "pipefit.test", "mode": "remove", "ids": ["pipefit.test.plate"]})
check("T16", r0.get("success") and r0.get("trajectory_valid") and (r.get("success") is False or r.get("trajectory_valid") is False),
      f"without plate: success={r0.get('success')} valid={r0.get('trajectory_valid')}; with plate: success={r.get('success')} "
      f"{r.get('error_code')} trajectory_valid={r.get('trajectory_valid')} contacts={[(c['a'], c['b']) for c in (r.get('contacts') or [])][:3]}")

# T17: a small box inside the camera volume (TCP frame (-0.050, -0.032, -0.047)) -> camera contact
cur = joints_now()
pt, qt = fk_pose(cur, "panda_hand_tcp")
R = quaternion_matrix(qt)[:3, :3]
spot = pt + R @ np.array(info["wrist_camera"]["center"])
cam_blocker = {"id": "pipefit.test.camblock", "pose": {"position": list(spot), "orientation": [0, 0, 0, 1]},
               "shapes": [{"type": "box", "size": [0.008, 0.008, 0.008]}]}
req("POST", "/scene/batch", {"namespace": "pipefit.test", "mode": "upsert", "objects": [cam_blocker]})
st, r = req("POST", "/plan/check_state", {"use_current": True})
req("POST", "/scene/batch", {"namespace": "pipefit.test", "mode": "remove", "ids": ["pipefit.test.camblock"]})
pairs = {tuple(sorted((c["a"], c["b"]))) for c in r.get("contacts", [])}
check("T17", st == 200 and not r["valid"] and any("wrist_camera" in p for p in pairs), f"valid={r.get('valid')} pairs={sorted(pairs)}")

# Q2: legacy attach/detach on MoveIt 1.1.16
st_a, ra = req("POST", "/scene/objects/manualbox/attach", {})
att = [a.object.id for a in gps(PlanningSceneComponents(components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)).scene.robot_state.attached_collision_objects]
st_d, rd = req("POST", "/scene/objects/manualbox/detach", {})
world_after = {co.id for co in gps(PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_NAMES)).scene.world.collision_objects}
check("Q2-legacy-attach", st_a == 200 and "manualbox" in att and st_d == 200 and "manualbox" in world_after,
      f"attach {st_a} attached={'manualbox' in att}; detach {st_d} back in world={'manualbox' in world_after}")

# bad input
st, r = req("POST", "/plan/dry_run", {"target": {"type": "joints", "joints": [0] * 6}})
check("BAD_INPUT", st == 400 and r.get("path") == "target.joints", f"{st} {r.get('error')} path={r.get('path')}")
check("S1", len(exec_goals) == 0 and max(abs(a - b) for a, b in zip(q_before, joints_now())) < 1e-4,
      f"execute goals {len(exec_goals)}, arm unchanged")
print("\n%d/%d passed" % (sum(ok for _, ok in results), len(results)))
