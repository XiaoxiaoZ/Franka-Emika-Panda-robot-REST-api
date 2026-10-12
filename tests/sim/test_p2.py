"""P2 (colours, cleanup) tests against the SIM dev server only.
    python3 test_p2.py phase1 ; restart the dev server ; python3 test_p2.py phase2
"""
import hashlib, json, os, struct, sys, time, urllib.error, urllib.request

B = "http://127.0.0.1:5100"
assert os.environ.get("ROS_MASTER_URI") == "http://127.0.0.1:11312", "source env.sh first (sim master only)"
PHASE = sys.argv[1] if len(sys.argv) > 1 else "phase1"
NS = "pipefit.p2"
results = []


def req(method, path, body=None, raw=None, timeout=120):
    data = raw if raw is not None else (json.dumps(body).encode() if body is not None else None)
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


import threading
import rospy
from moveit_msgs.msg import PlanningScene
rospy.init_node("p2_test", anonymous=True, disable_signals=True)

# Colours are what RViz receives on the monitored scene topic (move_group's own
# scene drops them a moment after publishing -- see scene_core._ensure_colors).
_latest = {"cols": {}, "t": 0.0}
_cv = threading.Condition()


def _on_scene(m):
    if m.object_colors:
        with _cv:
            _latest["cols"] = {c.id: [round(c.color.r, 3), round(c.color.g, 3), round(c.color.b, 3), round(c.color.a, 3)]
                               for c in m.object_colors}
            _latest["t"] = time.time()
            _cv.notify_all()


rospy.Subscriber("/move_group/monitored_planning_scene", PlanningScene, _on_scene)
time.sleep(1.0)


def scene_colors(timeout=3.0):
    """Colour table of the next monitored-scene message that carries colours
    (published right after a change, and by the keeper every 2 s)."""
    t_req = time.time()
    with _cv:
        _cv.wait_for(lambda: _latest["t"] > t_req, timeout=timeout)
        return dict(_latest["cols"])


def box(oid, x, color=None):
    o = {"id": oid, "pose": {"position": [x, -0.3, 0.05], "orientation": [0, 0, 0, 1]},
         "shapes": [{"type": "box", "size": [0.04, 0.04, 0.04]}]}
    if color is not None:
        o["color"] = color
    return o


def stl(seed):
    tri = [[(seed * 1e-3, 0, 0), (seed * 1e-3, 1e-2, 0), (seed * 1e-3, 0, 1e-2)]]
    out = bytearray(b"\0" * 80) + struct.pack("<I", len(tri))
    for t in tri:
        out += struct.pack("<3f", 0, 0, 0)
        for v in t:
            out += struct.pack("<3f", *v)
        out += b"\0\0"
    return bytes(out)


if PHASE == "phase1":
    req("DELETE", f"/scene/namespaces/{NS}")
    # ---------------- colours ----------------
    st, r = req("POST", "/scene/batch", {"namespace": NS, "mode": "replace",
                                         "objects": [box(NS + ".red", 0.4, [1, 0, 0, 0.6]), box(NS + ".blue", 0.5, [0, 0, 1]),
                                                     box(NS + ".plain", 0.6)]})
    cols = scene_colors()
    check("colours applied with the geometry", st == 200 and cols.get(NS + ".red") == [1, 0, 0, 0.6] and cols.get(NS + ".blue") == [0, 0, 1, 1]
          and NS + ".plain" not in cols, f"{ {k: v for k, v in cols.items() if NS in k} }")
    v1, h1 = r["scene_version"], r["namespace_hash"]
    st, r = req("POST", "/scene/batch", {"namespace": NS, "mode": "replace",
                                         "objects": [box(NS + ".red", 0.4, [0, 1, 0, 1]), box(NS + ".blue", 0.5, [0, 0, 1]),
                                                     box(NS + ".plain", 0.6)]})
    check("colour-only change: no version bump, hash unchanged, colour updated",
          st == 200 and r["unchanged"] and r["scene_version"] == v1 and r["namespace_hash"] == h1
          and r.get("recolored") == [NS + ".red"] and scene_colors().get(NS + ".red") == [0, 1, 0, 1],
          f"unchanged {r.get('unchanged')} version {r.get('scene_version')}=={v1} hash_same {r.get('namespace_hash') == h1} "
          f"recolored {r.get('recolored')} scene {scene_colors().get(NS + '.red')}")
    st, r = req("POST", "/scene/batch", {"namespace": NS, "mode": "upsert", "objects": [box(NS + ".bad", 0.7, [1, 2, 0])]})
    check("bad colour rejected", st == 400 and r.get("path") == "objects[0].color", f"{st} {r.get('error')} {r.get('path')}")
    # MoveIt drops all object colours when attached objects change: the API must restore them
    st_a, _ = req("POST", f"/scene/objects/{NS}.blue/attach", {"link": "panda_hand"})
    after_attach = scene_colors()
    st_d, _ = req("POST", f"/scene/objects/{NS}.blue/detach", {})
    after_detach = scene_colors()
    check("colours survive attach/detach", st_a == 200 and st_d == 200 and after_attach.get(NS + ".red") == [0, 1, 0, 1]
          and after_detach.get(NS + ".red") == [0, 1, 0, 1] and after_detach.get(NS + ".blue") == [0, 0, 1, 1]
          and after_detach.get("virtual_wall_x") == [1, 0.25, 0.2, 0.35],
          f"after attach red {after_attach.get(NS + '.red')}, after detach red {after_detach.get(NS + '.red')} "
          f"blue {after_detach.get(NS + '.blue')} wall {after_detach.get('virtual_wall_x')}")
    lst = {o["id"]: o for o in req("GET", f"/scene/objects?namespace={NS}")[1]["objects"]}
    check("colour listed", lst[NS + ".red"]["color"] == [0, 1, 0, 1] and lst[NS + ".plain"]["color"] is None, "")

    # ---------------- cleanup ----------------
    for name in ("a", "b", "c", "legacy"):
        req("DELETE", f"/files/pipefit/meshes/p2_{name}.stl")
    for i, name in enumerate(("a", "b", "c", "legacy")):
        req("PUT", f"/files/pipefit/meshes/p2_{name}.stl", raw=stl(i + 1))
    req("POST", "/scene/batch", {"namespace": NS, "mode": "upsert", "objects": [
        {"id": NS + ".mesh", "pose": {"position": [0.5, -0.2, 0.05], "orientation": [0, 0, 0, 1]},
         "shapes": [{"type": "mesh", "file": "pipefit/meshes/p2_a.stl"}]}]})
    req("POST", "/scene/models", {"name": "p2legacy", "file": "pipefit/meshes/p2_legacy.stl", "overwrite": 1})
    st, r = req("GET", "/scene/unreferenced?prefix=pipefit/meshes&older_than_s=0")
    names = sorted(f["path"].rsplit("/", 1)[-1] for f in r["files"] if "p2_" in f["path"])
    check("unreferenced lists only b, c", st == 200 and names == ["p2_b.stl", "p2_c.stl"], f"{names}")
    st, r = req("POST", "/scene/cleanup", {})
    check("default cleanup is a dry run and protects fresh files",
          st == 200 and r["dry_run"] and not any("p2_" in f for f in r["files"])
          and {"pipefit/meshes/p2_b.stl", "pipefit/meshes/p2_c.stl"} <= set(r["protected"]),
          f"would delete {[f for f in r['files'] if 'p2_' in f]}, protected {[f for f in r['protected'] if 'p2_' in f]}")
    # a legacy model in use is never offered for deletion
    req("POST", "/scene/objects", {"name": "p2legacyobj", "model": "p2legacy", "x": 0.6, "y": -0.2, "z": 0.05})
    st, r = req("POST", "/scene/cleanup", {"older_than_s": 0, "dry_run": True, "models": True})
    check("model in use is never listed", st == 200 and "p2legacy" not in r["models"], f"unused models: {r['models']}")
    req("DELETE", "/scene/objects/p2legacyobj")
    # real cleanup: unreferenced files b, c go; a (v2 object) and legacy (model file) stay; the now unused model goes
    st, r = req("POST", "/scene/cleanup", {"older_than_s": 0, "dry_run": False, "models": True})
    files_dir = os.path.join(os.environ.get("SIM_DIR", "/tmp/franka_sim"), "files", "pipefit", "meshes")
    exist = {n: os.path.exists(os.path.join(files_dir, f"p2_{n}.stl")) for n in ("a", "b", "c", "legacy")}
    check("cleanup deletes only unreferenced files",
          st == 200 and not r["dry_run"] and exist == {"a": True, "b": False, "c": False, "legacy": True},
          f"deleted {[f for f in r['files'] if 'p2_' in f]}, files left {exist}")
    st_m, _ = req("GET", "/scene/models/p2legacy")
    check("unused legacy model removed with models=true", "p2legacy" in r["models"] and st_m == 404,
          f"removed models {r['models']}, GET model -> {st_m}")
    json.dump({"version": req("GET", "/scene/info")[1]["scene_version"]}, open(os.path.join(os.environ.get("SIM_DIR", "/tmp/franka_sim"), "p2.json"), "w"))

else:
    for _ in range(25):           # the colour keeper reconciles within ~2 s of startup
        cols = scene_colors()
        if cols.get(NS + ".red") and cols.get("virtual_wall_x"):
            break
        time.sleep(0.2)
    check("colours restored after restart", cols.get(NS + ".red") == [0, 1, 0, 1] and cols.get(NS + ".blue") == [0, 0, 1, 1],
          f"{ {k: v for k, v in cols.items() if NS in k} }")
    check("built-in walls/ceiling keep their colour after the camera attach at startup",
          all(cols.get(n) == [1, 0.25, 0.2, 0.35] for n in ("virtual_wall_x", "virtual_wall_y", "virtual_ceiling")),
          f"{ {n: cols.get(n) for n in ('virtual_wall_x', 'virtual_wall_y', 'virtual_ceiling')} }")
    req("DELETE", f"/scene/namespaces/{NS}")
    req("DELETE", "/files/pipefit/meshes/p2_a.stl")
    req("DELETE", "/files/pipefit/meshes/p2_legacy.stl")

print("\n%d/%d passed" % (sum(results), len(results)))
