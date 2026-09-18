"""Named CAD models + scene objects for the MoveIt planning scene.

Two layers, so a model file is parsed once and can be placed many times
(e.g. every time the object detector re-runs):

* **models** -- a mesh file in the /files store (STL/OBJ/DAE/PLY), parsed
  with pyassimp into a shape_msgs/Mesh at a given scale; optionally reduced
  to its convex hull (much faster collision checking for detailed CAD).
* **objects** -- instances of a model at a pose in a frame. Each carries a
  ``source`` tag ("manual", "detector", ...) so :meth:`SceneManager.sync`
  can replace one source's objects wholesale without touching the others
  or the built-in safety geometry (walls, ceiling, floor).

Everything is persisted to ``scene.json`` and re-applied by
:meth:`restore` at server start, since move_group forgets its world on
restart.
"""
import copy
import json
import math
import os
import threading
import time

import numpy as np

from file_store import FileStoreError

MESH_EXTENSIONS = (".stl", ".obj", ".dae", ".ply")
BUILTIN_OBJECTS = ("floor", "virtual_wall_x", "virtual_wall_y", "virtual_ceiling")
COLLISION_MODES = ("mesh", "convex_hull")


class SceneError(Exception):
    def __init__(self, status, code, msg, **extra):
        super().__init__(msg)
        self.status = status
        self.code = code
        self.msg = msg
        self.extra = extra


# ---------------------------------------------------------------- meshes ----

def _walk_assimp(node, parent_tf, scene, out):
    """Collect (vertices, faces) of every mesh under ``node`` with the node
    transforms applied (moveit_commander.make_mesh only takes meshes[0])."""
    tf = parent_tf @ np.asarray(node.transformation, dtype=float).reshape(4, 4)
    for m in node.meshes:
        verts = np.asarray(m.vertices, dtype=float)
        if verts.size == 0:
            continue
        faces = [f for f in np.asarray(m.faces) if len(f) == 3] if hasattr(m.faces[0], "__len__") \
            else [f.indices for f in m.faces if len(f.indices) == 3]
        if not faces:
            continue
        hom = np.hstack([verts, np.ones((len(verts), 1))])
        out.append(((hom @ tf.T)[:, :3], np.asarray(faces, dtype=int)))
    for child in node.children:
        _walk_assimp(child, tf, scene, out)


def load_mesh_file(abs_path, scale=1.0, collision="mesh"):
    """Parse a mesh file into (shape_msgs/Mesh, stats dict)."""
    import pyassimp
    from shape_msgs.msg import Mesh, MeshTriangle
    from geometry_msgs.msg import Point

    if collision not in COLLISION_MODES:
        raise SceneError(400, "bad_collision", f"collision must be one of {COLLISION_MODES}")
    try:
        scene = pyassimp.load(abs_path)
    except Exception as e:
        raise SceneError(422, "mesh_unreadable", f"pyassimp could not read the file: {e}")
    try:
        parts = []
        _walk_assimp(scene.rootnode, np.eye(4), scene, parts)
    finally:
        pyassimp.release(scene)
    if not parts:
        raise SceneError(422, "mesh_empty", "no triangles found in the file")
    verts = []
    faces = []
    offset = 0
    for v, f in parts:
        verts.append(v)
        faces.append(f + offset)
        offset += len(v)
    verts = np.vstack(verts) * float(scale)
    faces = np.vstack(faces)
    source_triangles = int(len(faces))

    if collision == "convex_hull":
        from scipy.spatial import ConvexHull
        try:
            hull = ConvexHull(verts)
        except Exception as e:
            raise SceneError(422, "hull_failed", f"convex hull failed (degenerate mesh?): {e}")
        used = np.unique(hull.simplices)
        remap = {old: new for new, old in enumerate(used)}
        verts = verts[used]
        faces = np.vectorize(remap.get)(hull.simplices)

    msg = Mesh()
    msg.vertices = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in verts]
    msg.triangles = [MeshTriangle(vertex_indices=[int(a), int(b), int(c)]) for a, b, c in faces]
    lo, hi = verts.min(axis=0), verts.max(axis=0)
    stats = {
        "triangles": int(len(faces)),
        "source_triangles": source_triangles,
        "vertices": int(len(verts)),
        "bbox_min": [round(float(v), 4) for v in lo],
        "bbox_max": [round(float(v), 4) for v in hi],
        "bbox_size": [round(float(v), 4) for v in (hi - lo)],
    }
    return msg, stats


# ----------------------------------------------------------------- poses ----

def pose_from_dict(d, default=None):
    """geometry_msgs/Pose from {x,y,z} + {roll,pitch,yaw} (deg) or {qx,qy,qz,qw}.
    Missing position/orientation falls back to ``default`` (a Pose) or identity."""
    from geometry_msgs.msg import Pose
    from tf.transformations import quaternion_from_euler

    def num(k, fallback):
        v = d.get(k)
        if v is None:
            return fallback
        v = float(v)
        if math.isnan(v) or math.isinf(v):
            raise SceneError(400, "bad_pose", f"{k} must be finite")
        return v

    p = Pose()
    base = default if default is not None else Pose(orientation=type(p.orientation)(w=1.0))
    p.position.x = num("x", base.position.x)
    p.position.y = num("y", base.position.y)
    p.position.z = num("z", base.position.z)
    if any(d.get(k) is not None for k in ("qx", "qy", "qz", "qw")):
        q = [num("qx", 0.0), num("qy", 0.0), num("qz", 0.0), num("qw", 1.0)]
        n = math.sqrt(sum(v * v for v in q))
        if n < 1e-6:
            raise SceneError(400, "bad_pose", "quaternion has ~zero norm")
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = [v / n for v in q]
    elif any(d.get(k) is not None for k in ("roll", "pitch", "yaw")):
        q = quaternion_from_euler(math.radians(num("roll", 0.0)), math.radians(num("pitch", 0.0)),
                                  math.radians(num("yaw", 0.0)))
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    else:
        p.orientation = copy.deepcopy(base.orientation)
    return p


def pose_to_dict(p):
    from tf.transformations import euler_from_quaternion
    q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    r, pi, y = euler_from_quaternion(q)
    return {"x": p.position.x, "y": p.position.y, "z": p.position.z,
            "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3],
            "roll": math.degrees(r), "pitch": math.degrees(pi), "yaw": math.degrees(y)}


def _valid_name(name):
    import re
    return bool(name) and re.match(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$", name) is not None


# --------------------------------------------------------------- manager ----

class SceneManager:
    def __init__(self, psi, store, state_file, planning_frame="panda_link0", logger=None):
        self.psi = psi                    # moveit_commander.PlanningSceneInterface
        self.store = store                # file_store.FileStore
        self.state_file = state_file
        self.planning_frame = planning_frame
        self.log = logger or (lambda *a: None)
        self.models = {}                  # name -> info dict
        self.objects = {}                 # name -> info dict
        self._meshes = {}                 # model name -> shape_msgs/Mesh
        self._lock = threading.RLock()

    # ---- persistence ----

    def save(self):
        tmp = self.state_file + ".tmp"
        os.makedirs(os.path.dirname(self.state_file), exist_ok=True)
        with open(tmp, "w") as f:
            json.dump({"models": self.models, "objects": self.objects}, f, indent=2, sort_keys=True)
        os.replace(tmp, self.state_file)

    def restore(self):
        """Re-register models and re-add objects from scene.json. Models whose
        file vanished are kept with an 'error' field; their objects are skipped."""
        if not os.path.isfile(self.state_file):
            return {"models": 0, "objects": 0, "errors": []}
        with open(self.state_file) as f:
            data = json.load(f)
        errors = []
        with self._lock:
            for name, info in data.get("models", {}).items():
                try:
                    self.register_model(name, info["file"], info.get("scale", 1.0),
                                        info.get("collision", "mesh"), overwrite=True, persist=False)
                except SceneError as e:
                    info = dict(info, error=e.msg)
                    self.models[name] = info
                    errors.append(f"model {name}: {e.msg}")
            for name, info in data.get("objects", {}).items():
                info.pop("error", None)
                try:
                    self._apply_object(info)
                    if info.get("attached_link"):
                        self._apply_attach(info)
                except SceneError as e:
                    # keep the record so re-registering the model brings it back
                    info["error"] = e.msg
                    errors.append(f"object {name}: {e.msg}")
                self.objects[name] = info
            self.save()
        return {"models": len(self.models), "objects": len(self.objects), "errors": errors}

    # ---- models ----

    def list_models(self):
        return [dict(m, in_use=sorted(o for o, i in self.objects.items() if i["model"] == m["name"]))
                for m in sorted(self.models.values(), key=lambda m: m["name"])]

    def get_model(self, name):
        if name not in self.models:
            raise SceneError(404, "unknown_model", f"no model named {name!r}")
        return dict(self.models[name])

    def register_model(self, name, file, scale=1.0, collision="mesh", overwrite=False, persist=True):
        if not _valid_name(name):
            raise SceneError(400, "bad_name", "name must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}")
        try:
            scale = float(scale)
        except (TypeError, ValueError):
            raise SceneError(400, "bad_scale", "scale must be a number")
        if not (0 < scale < 1000):
            raise SceneError(400, "bad_scale", "scale must be in (0, 1000)")
        try:
            abs_path = self.store.resolve(file)
        except FileStoreError as e:
            raise SceneError(e.status, e.code, e.msg)
        if not os.path.isfile(abs_path):
            raise SceneError(404, "file_not_found", f"no file {file!r} in the /files store")
        if not abs_path.lower().endswith(MESH_EXTENSIONS):
            raise SceneError(415, "unsupported_format",
                             f"mesh must be one of {MESH_EXTENSIONS} (export STEP/IGES to STL first)")
        with self._lock:
            if name in self.models and not overwrite:
                raise SceneError(409, "conflict", f"model {name!r} exists (pass overwrite=1)")
            mesh, stats = load_mesh_file(abs_path, scale, collision)
            info = {
                "name": name, "file": self.store.relpath(abs_path), "sha256": self.store.sha256(abs_path),
                "scale": scale, "collision": collision, "registered": time.strftime("%Y-%m-%dT%H:%M:%S"),
                **stats,
            }
            self.models[name] = info
            self._meshes[name] = mesh
            # objects already placed with this model pick up the new mesh
            for obj in self.objects.values():
                if obj["model"] == name:
                    self._apply_object(obj)
                    obj.pop("error", None)
                    if obj.get("attached_link"):
                        self._apply_attach(obj)
            if persist:
                self.save()
        return dict(info)

    def delete_model(self, name, force=False):
        with self._lock:
            if name not in self.models:
                raise SceneError(404, "unknown_model", f"no model named {name!r}")
            users = [o for o, i in self.objects.items() if i["model"] == name]
            if users and not force:
                raise SceneError(409, "in_use", f"model {name!r} is used by objects {users} (pass force=1 to remove them too)")
            for o in users:
                self.remove_object(o, persist=False)
            self.models.pop(name)
            self._meshes.pop(name, None)
            self.save()

    # ---- objects ----

    def _mesh_for(self, model):
        if model not in self._meshes:
            if model in self.models:
                raise SceneError(409, "model_unavailable",
                                 f"model {model!r} could not be loaded: {self.models[model].get('error')}")
            raise SceneError(404, "unknown_model", f"no model named {model!r}")
        return self._meshes[model]

    def _apply_object(self, info):
        """Push (ADD/replace) a world object into the planning scene."""
        from moveit_msgs.msg import CollisionObject
        co = CollisionObject()
        co.id = info["name"]
        co.header.frame_id = info.get("frame") or self.planning_frame
        co.pose = pose_from_dict(info["pose"])
        co.meshes = [self._mesh_for(info["model"])]
        from geometry_msgs.msg import Pose
        co.mesh_poses = [Pose(orientation=type(co.pose.orientation)(w=1.0))]
        co.operation = CollisionObject.ADD
        self.psi.add_object(co)

    def _apply_attach(self, info):
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
        aco = AttachedCollisionObject()
        aco.link_name = info["attached_link"]
        aco.touch_links = info.get("touch_links") or [info["attached_link"]]
        aco.object = CollisionObject()
        aco.object.id = info["name"]
        aco.object.operation = CollisionObject.ADD
        self.psi.attach_object(aco, link=info["attached_link"], touch_links=aco.touch_links)

    def list_objects(self):
        """Our objects plus whatever else is in the scene (built-ins, demo box)."""
        out = {name: dict(info, builtin=False) for name, info in self.objects.items()}
        try:
            known = self.psi.get_known_object_names()
            others = [n for n in known if n not in out]
            poses = self.psi.get_object_poses(others) if others else {}
            for n in others:
                out[n] = {"name": n, "model": None, "source": "scene", "builtin": n in BUILTIN_OBJECTS,
                          "frame": self.planning_frame,
                          "pose": pose_to_dict(poses[n]) if n in poses else None}
            attached = self.psi.get_attached_objects()
            for n, aco in attached.items():
                out.setdefault(n, {"name": n, "model": None, "source": "scene", "builtin": False})
                out[n]["attached_link"] = aco.link_name
        except Exception as e:
            self.log("scene listing (move_group) failed: %s", e)
        return [out[k] for k in sorted(out)]

    def get_object(self, name):
        if name not in self.objects:
            raise SceneError(404, "not_found", f"no object named {name!r}")
        return dict(self.objects[name])

    def add_object(self, name, model, pose_dict, frame=None, source="manual", overwrite=True, persist=True):
        if not _valid_name(name):
            raise SceneError(400, "bad_name", "name must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}")
        if name in BUILTIN_OBJECTS:
            raise SceneError(409, "builtin", f"{name!r} is built-in safety geometry")
        with self._lock:
            if name in self.objects and not overwrite:
                raise SceneError(409, "conflict", f"object {name!r} exists")
            self._mesh_for(model)
            pose = pose_to_dict(pose_from_dict(pose_dict))
            info = {"name": name, "model": model, "pose": pose, "frame": frame or self.planning_frame,
                    "source": source or "manual", "attached_link": None,
                    "updated": time.strftime("%Y-%m-%dT%H:%M:%S")}
            if self.objects.get(name, {}).get("attached_link"):
                self.detach(name, persist=False)
            self._apply_object(info)
            self.objects[name] = info
            if persist:
                self.save()
        return dict(info)

    def update_object(self, name, pose_dict=None, frame=None):
        with self._lock:
            info = self.objects.get(name)
            if info is None:
                raise SceneError(404, "not_found", f"no object named {name!r}")
            if info.get("attached_link"):
                raise SceneError(409, "attached", "detach the object before moving it")
            if pose_dict:
                info["pose"] = pose_to_dict(pose_from_dict(pose_dict, default=pose_from_dict(info["pose"])))
            if frame:
                info["frame"] = frame
            info["updated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
            self._apply_object(info)
            self.save()
        return dict(info)

    def remove_object(self, name, force=False, persist=True):
        with self._lock:
            if name in BUILTIN_OBJECTS and not force:
                raise SceneError(409, "builtin", f"{name!r} is built-in safety geometry (force=1 to remove anyway)")
            info = self.objects.pop(name, None)
            if info is None and name not in self.psi.get_known_object_names() \
                    and name not in self.psi.get_attached_objects():
                raise SceneError(404, "not_found", f"no object named {name!r}")
            if info and info.get("attached_link"):
                self.psi.remove_attached_object(info["attached_link"], name)
            self.psi.remove_world_object(name)
            if persist:
                self.save()

    def attach(self, name, link="panda_hand", touch_links=None, persist=True):
        with self._lock:
            info = self.objects.get(name)
            if info is None:
                raise SceneError(404, "not_found", f"no object named {name!r}")
            info["attached_link"] = link
            info["touch_links"] = touch_links or [link, "panda_leftfinger", "panda_rightfinger", "panda_hand_tcp"]
            self._apply_attach(info)
            if persist:
                self.save()
        return dict(info)

    def detach(self, name, persist=True):
        with self._lock:
            info = self.objects.get(name)
            if info is None:
                raise SceneError(404, "not_found", f"no object named {name!r}")
            if info.get("attached_link"):
                # MoveIt puts a detached object back into the world where the
                # gripper left it; record that pose instead of the old one.
                self.psi.remove_attached_object(info["attached_link"], name)
                info["attached_link"] = None
                info.pop("touch_links", None)
                try:
                    poses = self.psi.get_object_poses([name])
                    if name in poses:
                        info["pose"] = pose_to_dict(poses[name])
                        info["frame"] = self.planning_frame
                except Exception as e:
                    self.log("could not read pose of detached %s: %s", name, e)
                info["updated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
            if persist:
                self.save()
        return dict(info)

    def sync(self, source, wanted):
        """Make the set of objects tagged ``source`` equal to ``wanted``
        (list of {name, model, x,y,z, roll/pitch/yaw | qx..qw, frame}).
        Objects of other sources and built-ins are untouched. Attached
        objects of this source are left alone too (something is holding them)."""
        if not source or source in ("scene", "manual"):
            raise SceneError(400, "bad_source", "sync needs a source tag other than 'manual'/'scene'")
        with self._lock:
            unknown = sorted({w.get("model") for w in wanted if w.get("model") not in self._meshes})
            if unknown:
                raise SceneError(404, "unknown_model", f"models not registered: {unknown}", unknown_models=unknown)
            names = [w.get("name") for w in wanted]
            if len(set(names)) != len(names):
                raise SceneError(400, "bad_request", "duplicate object names in sync")
            result = {"added": [], "updated": [], "removed": [], "kept_attached": []}
            for w in wanted:
                name = w.get("name")
                existing = self.objects.get(name)
                if existing and existing["source"] != source:
                    raise SceneError(409, "conflict", f"object {name!r} belongs to source {existing['source']!r}")
                if existing and existing.get("attached_link"):
                    result["kept_attached"].append(name)
                    continue
                self.add_object(name, w.get("model"), w, frame=w.get("frame"), source=source, persist=False)
                result["added" if existing is None else "updated"].append(name)
            for name, info in list(self.objects.items()):
                if info["source"] == source and name not in names:
                    if info.get("attached_link"):
                        result["kept_attached"].append(name)
                        continue
                    self.remove_object(name, persist=False)
                    result["removed"].append(name)
            self.save()
        return result
