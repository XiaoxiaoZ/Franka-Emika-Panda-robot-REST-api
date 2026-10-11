"""Acceptance tests T1-T11, T20 (+ hash, S1, S4 gate) for scene API v2.
Runs against the SIM dev server (127.0.0.1:5100, ROS master :11312) only."""
import hashlib, json, math, os, struct, sys, threading, time, urllib.error, urllib.request

B = "http://127.0.0.1:5100"
assert os.environ.get("ROS_MASTER_URI") == "http://127.0.0.1:11312", "source sim/env.sh first"
S = os.environ.get("SIM_DIR", "/tmp/franka_sim")
results = []


def req(method, path, body=None, raw=None, timeout=60):
    data = raw if raw is not None else (json.dumps(body).encode() if body is not None else None)
    hdr = {"Content-Type": "application/json"} if body is not None else {}
    r = urllib.request.Request(B + path, data=data, method=method, headers=hdr)
    try:
        with urllib.request.urlopen(r, timeout=timeout) as resp:
            txt = resp.read()
            return resp.status, (json.loads(txt) if txt and resp.headers.get_content_type() == "application/json" else {}), resp.headers
    except urllib.error.HTTPError as e:
        txt = e.read()
        try:
            return e.code, json.loads(txt or b"{}"), e.headers
        except ValueError:
            return e.code, {"raw": txt[:200].decode(errors="replace")}, e.headers


def check(tid, cond, detail=""):
    results.append((tid, bool(cond), detail))
    print(f"{'PASS' if cond else 'FAIL'} {tid}: {detail}")


# ---------- independent hash implementation (from the spec text) ----------
def f(v, d):
    s = "%.*f" % (d, v)
    return s[1:] if s.startswith("-") and float(s) == 0 else s


def npose(p):
    q = list(p.get("orientation", [0, 0, 0, 1])); n = math.sqrt(sum(x * x for x in q)); q = [x / n for x in q]
    if q[3] < 0: q = [-x for x in q]
    return {"position": [f(x, 6) for x in p.get("position", [0, 0, 0])], "orientation": [f(x, 9) for x in q]}


def client_hash(objs, sha_of):
    out = []
    for o in sorted(objs, key=lambda o: o["id"]):
        shapes = []
        for s in o["shapes"]:
            t = s["type"]
            dims = {"box": lambda: s["size"], "sphere": lambda: [s["radius"]],
                    "cylinder": lambda: [s["radius"], s["length"]], "mesh": lambda: [s.get("scale", 1.0)]}[t]()
            shapes.append({"type": t, "dims": [f(x, 6) for x in dims],
                           "pose": npose(s.get("pose") or {}), "mesh_sha256": sha_of(s) if t == "mesh" else None})
        out.append({"id": o["id"], "frame_id": o.get("frame_id", "panda_link0"), "pose": npose(o["pose"]),
                    "attached_link": None, "shapes": shapes})
    return "sha256:" + hashlib.sha256(json.dumps(out, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


# ---------- fixtures ----------
def stl_bytes(tris):
    out = bytearray(b"\0" * 80) + struct.pack("<I", len(tris))
    for t in tris:
        out += struct.pack("<3f", 0, 0, 0)
        for v in t: out += struct.pack("<3f", *v)
        out += b"\0\0"
    return bytes(out)


def cube_tris(a):
    h = a / 2; v = [(x, y, z) for x in (-h, h) for y in (-h, h) for z in (-h, h)]
    F = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return [[v[i] for i in t] for t in F]


cube = stl_bytes(cube_tris(0.05)); cube_sha = hashlib.sha256(cube).hexdigest()
big = stl_bytes([[(i * 1e-4, 0, 0), (i * 1e-4, 1e-3, 0), (i * 1e-4, 0, 1e-3)] for i in range(20500)])
big_sha = hashlib.sha256(big).hexdigest()
P = lambda pos, q=(0, 0, 0, 1): {"position": list(pos), "orientation": list(q)}
NS = "pipefit.test"

bin_obj = {"id": NS + ".bin", "frame_id": "panda_link0",
           "pose": P([0.298314, 0.335163, 0.002034], [0.001367, -0.000596, 0.689865, 0.723937]),
           "shapes": [
               {"type": "box", "size": [0.379318, 0.297335, 0.006153], "pose": P([0, 0, -0.003077])},
               {"type": "box", "size": [0.003, 0.288459, 0.195009], "pose": P([0.168029, 0, 0.097156], [0, 0.035268, 0, 0.999378])},
               {"type": "box", "size": [0.003, 0.288459, 0.195009], "pose": P([-0.168029, 0, 0.097156], [0, -0.035268, 0, 0.999378])},
               {"type": "box", "size": [0.003, 0.370442, 0.195015], "pose": P([0, 0.127076, 0.097156], [-0.025078, 0.025078, 0.706662, 0.706662])},
               {"type": "box", "size": [0.003, 0.370442, 0.195015], "pose": P([0, -0.127076, 0.097156], [0.025078, -0.025078, 0.706662, 0.706662])}],
           "meta": {"kind": "bin"}}
tube_obj = {"id": NS + ".tube", "frame_id": "panda_link0",
            "pose": P([0.356597, 0.219995, 0.027297], [-0.350493, -0.903623, -0.231204, 0.084643]),
            "shapes": [
                {"type": "sphere", "radius": 0.015056, "pose": P([0, 0, 0])},
                {"type": "sphere", "radius": 0.010992, "pose": P([0.017844, 0, 0])},
                {"type": "sphere", "radius": 0.010992, "pose": P([0.027458, 0.00073, 0])},
                {"type": "cylinder", "radius": 0.015056, "length": 0.017844, "pose": P([0.008922, 0, 0], [0, 0.707107, 0, 0.707107])},
                {"type": "cylinder", "radius": 0.010992, "length": 0.009752, "pose": P([0.022706, 0.000365, 0], [-0.026456, 0.706612, 0.026456, 0.706612])}]}
cube_obj = {"id": NS + ".cube", "frame_id": "panda_link0", "pose": P([0.45, 0, 0.30]),
            "shapes": [{"type": "mesh", "file": f"pipefit/meshes/{cube_sha}.stl", "sha256": cube_sha, "scale": 1.0}]}
payload = {"namespace": "pipefit", "mode": "replace", "objects": [bin_obj, tube_obj, cube_obj]}

# ---------- S1 watcher: move_group must never get an execute goal ----------
import rospy
from moveit_msgs.msg import ExecuteTrajectoryActionGoal, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
rospy.init_node("scene_p0_test", anonymous=True, disable_signals=True)
exec_goals = []
rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, lambda m: exec_goals.append(m))
gps = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
def world():
    return {co.id: co for co in gps(PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)).scene.world.collision_objects}
time.sleep(1.0)

# ---------- tests ----------
st, info, _ = req("GET", "/scene/info")
check("T1", st == 200 and info.get("planning_frame_to_panda_link0") == {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}
      and set(["box", "sphere", "cylinder", "mesh"]) <= set(info.get("shape_types", [])),
      f"planning_frame={info.get('planning_frame')} moveit={info.get('moveit_version')} dirty={info.get('dirty')}")
req("DELETE", "/scene/namespaces/pipefit")          # clean start
v0 = req("GET", "/scene/info")[1]["scene_version"]

path = f"/files/pipefit/meshes/{cube_sha}.stl"
req("DELETE", path)
s1 = req("PUT", path + "?overwrite=0", raw=cube)[0]
s2 = req("PUT", path + "?overwrite=0", raw=cube)[0]
etag = req("HEAD", path)[2].get("ETag")
check("T2", (s1, s2) == (201, 409) and etag == cube_sha, f"first {s1}, second {s2}, ETag==sha {etag == cube_sha}")

st, r, _ = req("POST", "/scene/batch", payload)
w = world()
cyl = [list(p.dimensions) for p in w.get(NS + ".tube").primitives if p.type == 3] if NS + ".tube" in w else []
check("T3", st == 200 and r.get("confirmed") and r.get("scene_version") == v0 + 1 and r["counts"]["primitives"] == 10
      and {NS + ".bin", NS + ".tube", NS + ".cube"} <= set(w)
      and len(w[NS + ".bin"].primitives) == 5 and len(w[NS + ".tube"].primitives) == 5 and len(w[NS + ".cube"].meshes) == 1
      and cyl and all(abs(d[0] - c["length"]) < 1e-9 and abs(d[1] - c["radius"]) < 1e-9
                      for d, c in zip(cyl, [s for s in tube_obj["shapes"] if s["type"] == "cylinder"])),
      f"HTTP {st} version {r.get('scene_version')} counts {r.get('counts')} cylinder dims {cyl}")
mine = client_hash(payload["objects"], lambda s: s["sha256"])
check("HASH", r.get("namespace_hash") == mine, f"server {r.get('namespace_hash', '')[:23]}.. client {mine[:23]}..")
v1 = r.get("scene_version")

st, r, _ = req("POST", "/scene/batch", payload)
check("T4", st == 200 and r.get("unchanged") is True and r.get("scene_version") == v1, f"unchanged={r.get('unchanged')} version {r.get('scene_version')}")

bad = json.loads(json.dumps(payload)); bad["objects"][2] = dict(cube_obj, shapes=[{"type": "sphere", "radius": -0.01}])
st, r, _ = req("POST", "/scene/batch", bad)
check("T5", st == 400 and r.get("error") == "bad_shape" and r.get("path") == "objects[2].shapes[0].radius"
      and req("GET", "/scene/info")[1]["scene_version"] == v1 and set(world()) >= {NS + ".bin", NS + ".tube", NS + ".cube"},
      f"{st} {r.get('error')} path={r.get('path')}")

miss = json.loads(json.dumps(payload)); miss["objects"][2]["shapes"][0].update(file="pipefit/meshes/nope.stl", sha256=None)
st_a, r_a, _ = req("POST", "/scene/batch", miss)
req("PUT", "/files/pipefit/meshes/trunc.stl", raw=cube[:150])
tr = json.loads(json.dumps(payload)); tr["objects"][2]["shapes"][0].update(file="pipefit/meshes/trunc.stl", sha256=None)
st_b, r_b, _ = req("POST", "/scene/batch", tr)
check("T6", st_a == 404 and r_a.get("error") == "unknown_mesh" and st_b == 422 and r_b.get("error") == "mesh_unreadable"
      and req("GET", "/scene/info")[1]["scene_version"] == v1, f"missing {st_a} {r_a.get('error')}, truncated {st_b} {r_b.get('error')}")

req("PUT", f"/files/pipefit/meshes/{big_sha}.stl", raw=big)
bg = json.loads(json.dumps(payload)); bg["objects"][2]["shapes"][0].update(file=f"pipefit/meshes/{big_sha}.stl", sha256=big_sha)
st, r, _ = req("POST", "/scene/batch", bg)
check("T7", st == 422 and r.get("error") == "limits_exceeded" and req("GET", "/scene/info")[1]["scene_version"] == v1, f"{st} {r.get('error')}: {r.get('msg', '')[:70]}")

st, r, _ = req("POST", "/scene/batch", dict(payload, if_version=v1 - 1))
check("T8", st == 409 and r.get("error") == "version_conflict", f"{st} {r.get('error')}")

# T9: a legacy (manual) object must survive a pipefit replace
req("PUT", "/files/cad/cube.stl", raw=cube)
req("POST", "/scene/models", {"name": "testcube", "file": "cad/cube.stl", "overwrite": 1})
st_m, _, _ = req("POST", "/scene/objects", {"name": "manualbox", "model": "testcube", "x": 0.3, "y": -0.3, "z": 0.5})
small = dict(payload, objects=[tube_obj])
st, r, _ = req("POST", "/scene/batch", small)
w = world()
check("T9", st_m == 201 and st == 200 and r.get("removed") == [NS + ".bin", NS + ".cube"] and "manualbox" in w
      and {"floor", "virtual_wall_x", "virtual_wall_y", "virtual_ceiling"} <= set(w),
      f"manual add {st_m}; replace removed {r.get('removed')}; manualbox+builtins present {('manualbox' in w)}")
req("POST", "/scene/batch", payload)

e1 = req("POST", "/scene/batch", {"namespace": "pipefit", "mode": "upsert", "objects": [dict(tube_obj, id="")]})
e2 = req("POST", "/scene/batch", {"namespace": "builtin", "mode": "upsert", "objects": [dict(tube_obj, id="builtin.x")]})
e3 = req("DELETE", "/scene/namespaces/")
e4 = req("DELETE", "/scene/namespaces/builtin")
w = world()
check("T10", [e[0] for e in (e1, e2, e3, e4)] == [400, 400, 400, 400] and {"floor", "virtual_wall_x", "virtual_wall_y", "virtual_ceiling"} <= set(w),
      f"codes {[e[0] for e in (e1, e2, e3, e4)]} {[e[1].get('error') for e in (e1, e2, e3, e4)]}")

# T11a: commit fails once -> rollback restores the previous state, not dirty
before = set(world()); vb = req("GET", "/scene/info")[1]["scene_version"]
req("POST", "/scene/_test/fail_next_apply?n=1")
st, r, _ = req("POST", "/scene/batch", dict(payload, objects=[tube_obj, cube_obj]))
inf = req("GET", "/scene/info")[1]
check("T11a", st == 504 and r.get("rolled_back") is True and not inf["dirty"] and inf["scene_version"] == vb and set(world()) == before,
      f"{st} {r.get('error')} rolled_back={r.get('rolled_back')} dirty={inf['dirty']}")
# T11b: commit AND rollback fail -> dirty, motion refused, cleared by a good batch
req("POST", "/scene/_test/fail_next_apply?n=2")
st, r, _ = req("POST", "/scene/batch", dict(payload, objects=[tube_obj, cube_obj]))
inf = req("GET", "/scene/info")[1]
ms, mr, _ = req("GET", "/control/goto_joints?j7=0.5")
msd, _, _ = req("GET", "/control/goto_joints?j7=0.5&dry_run=1")
mstop, _, _ = req("GET", "/control/stop")
st2, r2, _ = req("POST", "/scene/batch", payload)
inf2 = req("GET", "/scene/info")[1]
check("T11b", st == 504 and r.get("rolled_back") is False and inf["dirty"] and ms == 409 and mr.get("error") == "scene_dirty"
      and mstop == 200 and st2 == 200 and not inf2["dirty"],
      f"dirty={inf['dirty']} motion={ms} {mr.get('error')} dry_run={msd} stop={mstop}; clean batch {st2} -> dirty={inf2['dirty']}")

# S2: stale expect_scene_version on a motion endpoint
cur = inf2["scene_version"]
st, r, _ = req("GET", f"/control/goto_joints?j7=0.5&expect_scene_version={cur - 1}")
check("S2", st == 409 and r.get("error") == "version_conflict", f"{st} {r.get('error')}")

# T20: a slow commit holds the motion lock -> motion endpoints get 409 busy
req("POST", "/scene/_test/delay_next_apply?s=3")
out = {}
t = threading.Thread(target=lambda: out.update(r=req("POST", "/scene/batch", dict(payload, objects=[bin_obj, tube_obj])))); t.start()
time.sleep(1.0)
ms, mr, _ = req("GET", "/control/goto_joints?j7=0.5")
t.join()
check("T20", ms == 409 and out["r"][0] == 200, f"motion during commit: {ms} {mr.get('error') or mr.get('msg')}; batch {out['r'][0]}")
req("POST", "/scene/batch", payload)

# validate_only + remove + objects listing
st, r, _ = req("POST", "/scene/batch", dict(payload, objects=[tube_obj], validate_only=True))
check("VALIDATE_ONLY", st == 200 and r.get("validate_only") and r.get("would_change") and set(world()) >= {NS + ".bin", NS + ".cube"}, f"would remove {r.get('removed')}")
st, r, _ = req("GET", "/scene/objects?namespace=pipefit&geometry=1")
check("LIST", st == 200 and [o["id"] for o in r["objects"]] == sorted(o["id"] for o in payload["objects"])
      and len(r["objects"][0]["shapes"]) == 5, f"ids {[o['id'] for o in r['objects']]}")
st, r, _ = req("GET", "/scene/objects")
check("LIST_LEGACY_FORMAT", st == 200 and any(o.get("name") == "manualbox" for o in r["objects"]), "old format unchanged")

check("S1", len(exec_goals) == 0, f"execute_trajectory goals seen: {len(exec_goals)}")
print("\n%d/%d passed" % (sum(ok for _, ok, _ in results), len(results)))
json.dump({"scene_hash": req("GET", "/scene/info")[1]["scene_hash"]}, open(os.path.join(S, "hash_before_restart.json"), "w"))
