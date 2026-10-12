"""P1 acceptance tests (T18, T19, padding, /robot/*, /plan/ik, check_trajectory,
dry_run_sequence, persistence) against the SIM dev server only.

    python3 test_p1.py phase1   # everything, leaves an ACM entry + padding + an attached object
    (restart the dev server)
    python3 test_p1.py phase2   # checks those were restored, then cleans up
"""
import json, math, os, sys, time, urllib.error, urllib.request
import numpy as np

B = "http://127.0.0.1:5100"
assert os.environ.get("ROS_MASTER_URI") == "http://127.0.0.1:11312", "source env.sh first (sim master only)"
SIM_DIR = os.environ.get("SIM_DIR", "/tmp/franka_sim")
PHASE = sys.argv[1] if len(sys.argv) > 1 else "phase1"
NS = "pipefit.p1"
CUBE = NS + ".cube"
results = []


def req(method, path, body=None, timeout=180):
    data = json.dumps(body).encode() if body is not None else None
    r = urllib.request.Request(B + path, data=data, method=method,
                               headers={"Content-Type": "application/json"} if body is not None else {})
    try:
        with urllib.request.urlopen(r, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read() or b"{}")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def check(tid, cond, detail=""):
    results.append(bool(cond))
    print(f"{'PASS' if cond else 'FAIL'} {tid}: {detail}")


import rospy
from moveit_msgs.msg import ExecuteTrajectoryActionGoal, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from tf.transformations import quaternion_matrix, euler_from_quaternion
rospy.init_node("p1_test", anonymous=True, disable_signals=True)
exec_goals = []
rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, lambda m: exec_goals.append(m))
gps = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
time.sleep(1.0)
HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
DOWN = [1.0, 0.0, 0.0, 0.0]


def scene(comp):
    return gps(PlanningSceneComponents(components=comp)).scene


def world():
    return {co.id for co in scene(PlanningSceneComponents.WORLD_OBJECT_NAMES).world.collision_objects}


def attached():
    return [a.object.id for a in scene(PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS).robot_state.attached_collision_objects]


def acm_dict():
    m = scene(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX).allowed_collision_matrix
    n = list(m.entry_names)
    return {(a, b): bool(m.entry_values[i].enabled[j]) for i, a in enumerate(n) for j, b in enumerate(n)}, \
        dict(zip(m.default_entry_names, m.default_entry_values))


def tcp_at(q):
    return req("GET", "/robot/tf?from=panda_link0&to=panda_hand_tcp&joints=" + ",".join(map(str, q)))[1]["pose"]


def box(oid, pos, size, q=(0, 0, 0, 1)):
    return {"id": oid, "pose": {"position": list(map(float, pos)), "orientation": list(q)},
            "shapes": [{"type": "box", "size": list(size)}]}


q_before = req("GET", "/joints")[1]["positions"]
BUILTIN_ATTACHED = sorted(attached())          # wrist camera, flange adapter ...

if PHASE == "phase1":
    req("DELETE", f"/scene/namespaces/{NS}?force_detach=1")
    req("DELETE", "/scene/namespaces/pipefit.test?force_detach=1")   # the other suites' mock scene would block the pick
    req("DELETE", "/scene/acm")
    req("POST", "/scene/padding", {"default_m": 0.0})

    # ---------------- /robot/tf, /robot/description ----------------
    st, r = req("GET", "/robot/tf?from=panda_link8&to=panda_hand_tcp")
    yaw = math.degrees(euler_from_quaternion(r["pose"]["orientation"])[2])
    f2t = req("GET", "/scene/info")[1]["flange_to_tcp_m"]       # 0.1034 stock, 0.1234 with this robot's adapter
    check("T21b robot/tf link8->tcp", st == 200 and np.allclose(r["pose"]["position"], [0, 0, f2t], atol=1e-6) and abs(yaw + 45) < 1e-4,
          f"xyz {np.round(r['pose']['position'], 5).tolist()} yaw {yaw:.4f} deg")
    st, r = req("GET", "/robot/description")
    check("robot/description", st == 200 and "<robot" in r["urdf"] and "<robot" in r["srdf"] and len(r["joint_limits"]) == 7
          and r["urdf_sha256"] == req("GET", "/scene/info")[1]["urdf_sha256"], f"urdf {len(r['urdf'])} B, srdf {len(r['srdf'])} B, links {len(r['links'])}")

    # ---------------- T18: v2 attach / detach ----------------
    tcp = tcp_at(HOME)
    Rt = quaternion_matrix(tcp["orientation"])[:3, :3]
    cube_pos = list(np.array(tcp["position"]) + Rt @ np.array([0, 0, 0.01]))   # 1 cm along the fingers: clear of the camera box
    req("POST", "/scene/batch", {"namespace": NS, "mode": "replace",
                                 "objects": [box(CUBE, cube_pos, (0.05, 0.05, 0.05))]})
    _, r0 = req("POST", "/plan/check_state", {"joints": HOME, "gripper_width_m": 0.02})
    st_a, ra = req("POST", f"/scene/objects/{CUBE}/attach", {})
    _, r1 = req("POST", "/plan/check_state", {"joints": HOME, "gripper_width_m": 0.02})
    att = attached()
    st_d, rd = req("POST", f"/scene/objects/{CUBE}/detach", {})
    _, r2 = req("POST", "/plan/check_state", {"joints": HOME, "gripper_width_m": 0.02})
    moved = np.linalg.norm(np.array(rd.get("pose", {}).get("position", [9, 9, 9])) - np.array(cube_pos))
    check("T18 attach/detach", not r0["valid"] and st_a == 200 and ra.get("confirmed") and CUBE in att and r1["valid"]
          and st_d == 200 and CUBE in world() and CUBE not in attached() and not r2["valid"] and moved < 1e-4,
          f"fingers in cube: before {r0['valid']} -> attached {r1['valid']} -> detached {r2['valid']}; "
          f"pose after detach off by {moved*1000:.3f} mm; version {rd.get('scene_version')}")

    # ---------------- T19: ACM ----------------
    acm_before, defaults_before = acm_dict()
    st, r = req("POST", "/scene/acm", {"entries": [{"a": "panda_leftfinger", "b": CUBE, "allowed": True},
                                                   {"a": "panda_rightfinger", "b": CUBE, "allowed": True}]})
    _, rv = req("POST", "/plan/check_state", {"joints": HOME, "gripper_width_m": 0.02})
    st_del, rdel = req("DELETE", f"/scene/acm?namespace={NS}")
    _, ri = req("POST", "/plan/check_state", {"joints": HOME, "gripper_width_m": 0.02})
    acm_after, defaults_after = acm_dict()
    # self pairs (a, a) are never checked by MoveIt and are not preserved by an ACM message
    diff = [(k, v, acm_after.get(k)) for k, v in acm_before.items() if CUBE not in k and k[0] != k[1] and acm_after.get(k) != v]
    others_same = not diff
    check("T19 ACM allow/remove", st == 200 and rv["valid"] and st_del == 200 and not ri["valid"]
          and defaults_after == defaults_before and others_same,
          f"allowed -> valid {rv['valid']}; deleted -> valid {ri['valid']}; defaults unchanged {defaults_after == defaults_before}; "
          f"other entries unchanged {others_same} {diff[:4]}; contacts while allowed {[(c['a'], c['b']) for c in rv.get('contacts', [])]}")
    sc_pairs = [(k, v) for k, v in acm_after.items() if CUBE in k and any(x.endswith("_sc") for x in k) and v is not True]
    check("ACM keeps *_sc default-allowed for new objects", not sc_pairs, f"non-allowed cube/_sc pairs: {sc_pairs[:3]}")
    st, r = req("POST", "/scene/acm", {"entries": [{"a": "panda_link3", "b": "no_such_thing", "allowed": True}]})
    check("ACM bad name", st == 404 and r.get("path") == "entries[0].b", f"{st} {r.get('error')} {r.get('path')}")

    req("POST", "/scene/batch", {"namespace": NS, "mode": "remove", "ids": [CUBE]})   # clear the fingers

    # ---------------- padding ----------------
    p7 = req("GET", "/robot/tf?from=panda_link0&to=panda_link7&joints=" + ",".join(map(str, HOME)))[1]["pose"]
    R7 = quaternion_matrix(p7["orientation"])[:3, :3]
    # a small box 5 mm off link7's side (link7 body is ~ +0.06 along its z axis, radius ~ 0.045)
    probe = None
    for d_ in ([1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]):
        d_ = np.array(d_, float)
        for r_off in np.arange(0.02, 0.10, 0.0025):
            pos = np.array(p7["position"]) + R7 @ (d_ * r_off + np.array([0, 0, 0.06]))
            req("POST", "/scene/batch", {"namespace": NS, "mode": "upsert", "objects": [box(NS + ".probe", pos, (0.004, 0.004, 0.004))]})
            if req("POST", "/plan/check_state", {"joints": HOME})[1]["valid"]:
                probe = pos + R7 @ (d_ * 0.004)      # 4 mm outside the first free spot
                break
        if probe is not None:
            break
    req("POST", "/scene/batch", {"namespace": NS, "mode": "upsert", "objects": [box(NS + ".probe", probe, (0.004, 0.004, 0.004))]})
    v0 = req("POST", "/plan/check_state", {"joints": HOME})[1]["valid"]
    st, rp = req("POST", "/scene/padding", {"default_m": 0.012, "links": {"panda_leftfinger": 0.002, "panda_rightfinger": 0.002}})
    v1 = req("POST", "/plan/check_state", {"joints": HOME})[1]["valid"]
    gp = req("GET", "/scene/padding")[1]
    check("padding", st == 200 and v0 and not v1 and abs(gp["applied"]["panda_link7"] - 0.012) < 1e-9 and abs(gp["applied"]["panda_leftfinger"] - 0.002) < 1e-9,
          f"probe valid without padding {v0}, with 12 mm padding {v1}; applied link7 {gp['applied'].get('panda_link7')}")
    req("POST", "/scene/padding", {"default_m": 0.0})
    req("POST", "/scene/batch", {"namespace": NS, "mode": "remove", "ids": [NS + ".probe"]})

    # ---------------- /plan/ik ----------------
    st, r = req("POST", "/plan/ik", {"pose": tcp, "seeds": [HOME], "n_solutions": 4})
    best = r["solutions"][0]["q"] if r.get("solutions") else [9] * 7
    distinct = all(np.max(np.abs(np.array(a["q"]) - np.array(b["q"]))) > 0.2
                   for i, a in enumerate(r.get("solutions", [])) for b in r["solutions"][i + 1:])
    check("plan/ik", st == 200 and r["reachable"] and np.max(np.abs(np.array(best) - np.array(HOME))) < 0.03 and distinct,
          f"{len(r.get('solutions', []))} solutions (of {r.get('analytic_solutions')} analytic), best off home by "
          f"{np.max(np.abs(np.array(best) - np.array(HOME))):.4f} rad, distinct {distinct}, valid {[s['valid'] for s in r.get('solutions', [])]}")

    # ---------------- check_trajectory ----------------
    goal = list(HOME); goal[0] += 1.0
    st, dr = req("POST", "/plan/dry_run", {"target": {"type": "joints", "joints": goal}, "planner": "PTP", "start": {"joints": HOME}})
    st1, c1 = req("POST", "/plan/check_trajectory", {"plan_id": dr.get("plan_id")})
    mid = list(HOME); mid[0] += 0.5
    pm = tcp_at(mid)["position"]
    req("POST", "/scene/batch", {"namespace": NS, "mode": "upsert", "objects": [box(NS + ".wall", pm, (0.06, 0.06, 0.06))]})
    pts = [{"t": i * 0.1, "q": list(np.array(HOME) + (np.array(goal) - np.array(HOME)) * i / 10)} for i in range(11)]
    st2, c2 = req("POST", "/plan/check_trajectory", {"points": pts})
    req("POST", "/scene/batch", {"namespace": NS, "mode": "remove", "ids": [NS + ".wall"]})
    check("check_trajectory", st1 == 200 and c1["valid"] and st2 == 200 and not c2["valid"] and 1 <= c2["first_invalid_index"] <= 9,
          f"plan_id path valid {c1.get('valid')} ({c1.get('states_checked')} states); with wall: valid {c2.get('valid')} "
          f"first_invalid_index {c2.get('first_invalid_index')} t={c2.get('first_invalid_t')}")

    # ---------------- dry_run_sequence: pick a cube off the floor ----------------
    req("POST", "/scene/batch", {"namespace": NS, "mode": "replace",
                                 "objects": [box(CUBE, (0.45, 0.0, 0.0255), (0.05, 0.05, 0.05))]})
    v_before = req("GET", "/scene/info")[1]["scene_version"]
    acm0, _ = acm_dict()
    pre = {"type": "pose", "pose": {"position": [0.45, 0.0, 0.15], "orientation": DOWN}}
    # the custom fingers reach ~33 mm past the TCP (hand + 0.1034): grasp with the TCP 6 cm up
    grasp = {"type": "pose", "pose": {"position": [0.45, 0.0, 0.06], "orientation": DOWN}}
    lift = {"type": "pose", "pose": {"position": [0.45, 0.0, 0.15], "orientation": DOWN}}
    seq = {"start": {"joints": HOME}, "gripper_width_m": 0.08, "defaults": {"velocity_scaling": 0.2},
           "steps": [{"plan": {"target": pre, "planner": "PTP"}},
                     {"plan": {"target": grasp, "planner": "LIN"}},
                     {"acm": {"entries": [{"a": "panda_leftfinger", "b": CUBE, "allowed": True},
                                          {"a": "panda_rightfinger", "b": CUBE, "allowed": True},
                                          {"a": CUBE, "b": "floor", "allowed": True}]}},
                     {"gripper_width_m": 0.048},
                     {"attach": {"id": CUBE}},
                     {"plan": {"target": lift, "planner": "LIN"}}]}
    st, r = req("POST", "/plan/dry_run_sequence", seq)
    acm1, _ = acm_dict()
    objs = {o["id"]: o for o in req("GET", f"/scene/objects?namespace={NS}")[1]["objects"]}
    kinds = [(s["kind"], s["ok"]) for s in r.get("steps", [])]
    last = (r.get("steps") or [{}])[-1]
    check("sequence pick", st == 200 and r["success"] and r["restored"] and len(r["steps"]) == 6,
          f"steps {kinds} {r.get('elapsed_ms')} ms {'' if r.get('success') else (last.get('error_code'), last.get('goal_contacts') or last.get('contacts'))}")
    lift_end = (r.get("steps") or [{}])[-1].get("final_tcp_pose", {}).get("position")
    check("sequence restored scene", CUBE in world() and sorted(attached()) == BUILTIN_ATTACHED and acm1 == acm0
          and req("GET", "/scene/info")[1]["scene_version"] == v_before
          and np.allclose(objs[CUBE]["pose"]["position"], [0.45, 0.0, 0.0255]),
          f"cube back in world, only built-ins attached {sorted(attached())}, ACM identical {acm1 == acm0}, version unchanged, lift ended at {np.round(lift_end, 3).tolist() if lift_end else None}")
    # cube sunk 2 mm into the floor (inflated geometry overlaps) and no cube-floor ACM entry:
    # the attached cube is in collision with the floor -> must fail at attach (or the lift)
    seq_fail = json.loads(json.dumps(seq))
    seq_fail["steps"][2]["acm"]["entries"] = seq_fail["steps"][2]["acm"]["entries"][:2]
    req("POST", "/scene/batch", {"namespace": NS, "mode": "upsert", "objects": [box(CUBE, (0.45, 0.0, 0.023), (0.05, 0.05, 0.05))]})
    acm0, _ = acm_dict()
    st, r = req("POST", "/plan/dry_run_sequence", seq_fail)
    check("sequence failure + restore", st == 200 and not r["success"] and r["first_failed_step"] in (4, 5) and r["restored"]
          and CUBE in world() and sorted(attached()) == BUILTIN_ATTACHED and acm_dict()[0] == acm0,
          f"first_failed_step {r.get('first_failed_step')} ({r['steps'][-1]['kind']}: {r['steps'][-1].get('error_code') or r['steps'][-1].get('contacts')}), restored {r.get('restored')}")

    # ---------------- persistence: leave state for phase 2 ----------------
    req("POST", "/scene/batch", {"namespace": NS, "mode": "replace",
                                 "objects": [box(CUBE, cube_pos, (0.05, 0.05, 0.05)), box(NS + ".far", (0.5, -0.3, 0.05), (0.05, 0.05, 0.05))]})
    req("POST", f"/scene/objects/{CUBE}/attach", {})
    req("POST", "/scene/acm", {"entries": [{"a": "panda_link5", "b": NS + ".far", "allowed": True}]})
    req("POST", "/scene/padding", {"default_m": 0.004})
    snap = req("GET", "/scene/snapshot")[1]
    json.dump({"scene_hash": snap["scene_hash"]}, open(os.path.join(SIM_DIR, "p1_hash.json"), "w"))
    check("snapshot", snap["objects"] and snap["acm"] and snap["padding"] == {"default_m": 0.004, "links": {}},
          f"{len(snap['objects'])} objects, {len(snap['acm'])} acm entries, padding {snap['padding']}")

else:  # phase2 -- after a server restart
    saved = json.load(open(os.path.join(SIM_DIR, "p1_hash.json")))["scene_hash"]
    info = req("GET", "/scene/info")[1]
    acm, _ = acm_dict()
    pad = req("GET", "/scene/padding")[1]
    check("restart restore", info["scene_hash"] == saved and not info["dirty"] and CUBE in attached()
          and acm.get(("panda_link5", NS + ".far")) is True and abs(pad["applied"].get("panda_link3", 0) - 0.004) < 1e-9,
          f"hash equal {info['scene_hash'] == saved}, dirty {info['dirty']}, cube re-attached {CUBE in attached()}, "
          f"ACM entry back {acm.get(('panda_link5', NS + '.far'))}, padding {pad['applied'].get('panda_link3')}")
    st, r = req("DELETE", f"/scene/namespaces/{NS}?force_detach=1")
    req("DELETE", "/scene/acm")
    req("POST", "/scene/padding", {"default_m": 0.0})
    acm, _ = acm_dict()
    check("cleanup", st == 200 and CUBE not in world() and CUBE not in attached() and not any(NS in k[0] or NS in k[1] for k in acm),
          f"removed {r.get('removed')}, ACM rows of the namespace gone")

check("S1 no motion", len(exec_goals) == 0 and max(abs(a - b) for a, b in zip(q_before, req("GET", "/joints")[1]["positions"])) < 1e-4,
      f"execute goals {len(exec_goals)}")
print("\n%d/%d passed" % (sum(results), len(results)))
