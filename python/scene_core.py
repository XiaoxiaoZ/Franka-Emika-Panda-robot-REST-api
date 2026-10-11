"""Namespaced, versioned collision scene for MoveIt (API v2, /scene/batch).

Objects have ids like ``pipefit.<scene_id>.<object>``; the namespace is
everything before the last ``.`` and is hierarchical (``pipefit`` covers
``pipefit.*``). Each object is a list of shapes (box / sphere / cylinder /
mesh), each with its own pose relative to the object pose.

A batch is validated completely (ids, frames, poses, shapes, mesh files,
limits) before anything is sent. It is then committed as ONE PlanningScene
diff through ``/apply_planning_scene``; the response's ``success`` is
checked and the world object names are read back. If either check fails
the previous state of every touched object is re-applied; if that fails
too the scene is marked ``dirty`` and motion endpoints refuse to run until
a later batch or namespace delete succeeds.

Scene hash (clients must compute the same value), per object, sorted by id:
    {"id", "frame_id",
     "pose": {"position": [%.6f x3], "orientation": [%.9f x4]},   # quaternion normalised, w >= 0
     "attached_link": str | null,
     "shapes": [{"type", "dims": [%.6f ...], "pose": {...}, "mesh_sha256": str | null}]}
dims: box [x, y, z], sphere [radius], cylinder [radius, length], mesh [scale].
Numbers are formatted as STRINGS with the given printf format, and "-0.000..."
is written without the sign. Serialised with sort_keys and separators
(",", ":"), sha256 of the UTF-8 bytes, prefixed "sha256:".
"""
import copy
import hashlib
import json
import math
import os
import re
import threading
import time

from scene_manager import SceneError, BUILTIN_OBJECTS, MESH_EXTENSIONS, _parse_mesh_isolated

API_VERSION = "2.0"
ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$")
NS_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*(\.[A-Za-z0-9][A-Za-z0-9_-]*)*$")
RESERVED_NS = ("builtin", "manual")
SHAPE_TYPES = ("box", "sphere", "cylinder", "mesh")
MODES = ("replace", "upsert", "remove")
MAX_DIM_M = 10.0

DEFAULT_LIMITS = {
    "max_body_bytes": 8 * 1024 * 1024,
    "max_objects_per_batch": 500,
    "max_shapes_per_object": 1024,
    "max_primitives_total": 20000,
    "max_triangles_per_mesh": 20000,
    "max_triangles_total": 300000,
    "max_id_len": 64,
    "max_batch_meta_bytes": 16 * 1024,
    "max_object_meta_bytes": 4 * 1024,
}


def _err(status, code, msg, path=None, **extra):
    if path is not None:
        extra["path"] = path
    return SceneError(status, code, msg, **extra)


# ------------------------------------------------------------ numbers ----

def _num(v, path, positive=False, upper=None):
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise _err(400, "bad_shape" if positive else "bad_pose", "must be a number", path)
    v = float(v)
    if math.isnan(v) or math.isinf(v):
        raise _err(400, "bad_shape" if positive else "bad_pose", "must be finite", path)
    if positive and v <= 0:
        raise _err(400, "bad_shape", "must be > 0", path)
    if upper is not None and v > upper:
        raise _err(400, "bad_shape", f"must be <= {upper}", path)
    return v


def _fmt(v, digits):
    s = "%.*f" % (digits, v)
    return s[1:] if s.startswith("-") and float(s) == 0.0 else s


# -------------------------------------------------------------- poses ----

IDENTITY_POSE = {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}


def canon_pose(d, path):
    """{"position":[3], "orientation":[qx,qy,qz,qw]} -> normalised, w >= 0."""
    if d is None:
        return copy.deepcopy(IDENTITY_POSE)
    if not isinstance(d, dict):
        raise _err(400, "bad_pose", "pose must be an object {position, orientation}", path)
    unknown = set(d) - {"position", "orientation"}
    if unknown:
        raise _err(400, "bad_pose", f"unknown pose fields {sorted(unknown)} (rpy is not accepted; use a quaternion)", path)
    pos = d.get("position", [0, 0, 0])
    ori = d.get("orientation", [0, 0, 0, 1])
    if not isinstance(pos, list) or len(pos) != 3:
        raise _err(400, "bad_pose", "position must be [x, y, z]", f"{path}.position")
    if not isinstance(ori, list) or len(ori) != 4:
        raise _err(400, "bad_pose", "orientation must be [qx, qy, qz, qw]", f"{path}.orientation")
    p = [_num(v, f"{path}.position[{i}]") for i, v in enumerate(pos)]
    q = [_num(v, f"{path}.orientation[{i}]") for i, v in enumerate(ori)]
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-6:
        raise _err(400, "bad_pose", "quaternion norm < 1e-6", f"{path}.orientation")
    q = [v / n for v in q]
    if q[3] < 0:
        q = [-v for v in q]
    return {"position": p, "orientation": q}


def _hash_pose(p):
    return {"position": [_fmt(v, 6) for v in p["position"]],
            "orientation": [_fmt(v, 9) for v in p["orientation"]]}


# ------------------------------------------------------------- shapes ----

def shape_dims(s):
    t = s["type"]
    if t == "box":
        return list(s["size"])
    if t == "sphere":
        return [s["radius"]]
    if t == "cylinder":
        return [s["radius"], s["length"]]
    return [s["scale"]]


def hash_object(o):
    return {
        "id": o["id"],
        "frame_id": o["frame_id"],
        "pose": _hash_pose(o["pose"]),
        "attached_link": o.get("attached_link") or None,
        "shapes": [{"type": s["type"], "dims": [_fmt(v, 6) for v in shape_dims(s)],
                    "pose": _hash_pose(s["pose"]),
                    "mesh_sha256": s.get("sha256") if s["type"] == "mesh" else None}
                   for s in o["shapes"]],
    }


def hash_objects(objs):
    payload = [hash_object(o) for o in sorted(objs, key=lambda o: o["id"])]
    raw = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return "sha256:" + hashlib.sha256(raw).hexdigest()


def namespace_of(obj_id):
    return obj_id.rsplit(".", 1)[0] if "." in obj_id else "manual"


def in_namespace(obj_id, ns):
    return obj_id.startswith(ns + ".")


def _json_size(v):
    return len(json.dumps(v, separators=(",", ":")).encode("utf-8"))


# ---------------------------------------------------------------- core ----

class SceneCore:
    """State + validation + commit for /scene/batch. ROS access goes through
    ``io`` (see MoveItSceneIO) so tests can stub it."""

    def __init__(self, io, store, state_path, planning_frame="panda_link0",
                 limits=None, logger=None, legacy=None):
        self.io = io
        self.store = store
        self.state_path = state_path
        self.planning_frame = planning_frame
        self.limits = dict(DEFAULT_LIMITS, **(limits or {}))
        self.log = logger or (lambda *a: None)
        self.legacy = legacy                  # scene_manager.SceneManager or None
        self._lock = threading.RLock()
        self._mesh_cache = {}                 # sha256 -> (verts, faces); unscaled
        self._mesh_cache_tris = 0
        self.version = 0
        self.dirty_namespaces = set()         # namespaces whose move_group state is unknown
        self.dirty_reason = None
        self.updated_at = None
        self.objects = {}                     # id -> canonical object (incl. meta)
        self.batch_meta = {}                  # namespace -> last batch meta
        self.test_hooks = {"fail_next_apply": 0, "delay_next_apply_s": 0.0}
        self.builtin_attached = None          # wrist camera box, see restful.py

    @property
    def dirty(self):
        return bool(self.dirty_namespaces)

    def _covers(self, ns, d):
        """A batch / delete on ``ns`` resyncs namespace ``d``."""
        return d == ns or d.startswith(ns + ".")

    def _is_dirty(self, ns):
        return any(self._covers(ns, d) for d in self.dirty_namespaces)

    def _clear_dirty(self, ns):
        self.dirty_namespaces = {d for d in self.dirty_namespaces if not self._covers(ns, d)}
        if not self.dirty_namespaces:
            self.dirty_reason = None

    # ---- persistence ----

    def _snapshot(self):
        return {"api_version": API_VERSION, "version": self.version,
                "dirty_namespaces": sorted(self.dirty_namespaces), "dirty_reason": self.dirty_reason, "updated_at": self.updated_at,
                "objects": self.objects, "batch_meta": self.batch_meta}

    def save(self):
        os.makedirs(os.path.dirname(self.state_path), exist_ok=True)
        tmp = self.state_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self._snapshot(), f, indent=1, sort_keys=True)
        os.replace(tmp, self.state_path)

    def load_and_restore(self):
        """Read the state file and push every object back into move_group in
        one diff (move_group forgets its world on restart). Any failure marks
        the scene dirty. Returns a summary dict."""
        if not os.path.isfile(self.state_path):
            return {"objects": 0, "restored": True}
        with open(self.state_path) as f:
            data = json.load(f)
        with self._lock:
            self.version = int(data.get("version", 0))
            self.updated_at = data.get("updated_at")
            self.objects = data.get("objects", {})
            self.batch_meta = data.get("batch_meta", {})
            self.dirty_namespaces = set(data.get("dirty_namespaces", []))
            self.dirty_reason = data.get("dirty_reason")
            if not self.objects:
                return {"objects": 0, "restored": True, "dirty": self.dirty}
            try:
                for o in self.objects.values():
                    for i, s in enumerate(o["shapes"]):
                        if s["type"] == "mesh":
                            self._resolve_mesh(s["file"], s.get("sha256"), f"{o['id']}.shapes[{i}]")
                world = set(self.io.world_names())
                ops = [("remove", oid, None) for oid in self.objects if oid in world]
                ops += [("add", oid, o) for oid, o in self.objects.items()]
                self._apply_and_confirm(ops, expect_present=set(self.objects), expect_absent=set())
            except SceneError as e:
                self._mark_dirty(f"restore failed: {e.msg}",
                                 {oid.rsplit(".", 1)[0] for oid in self.objects})
                self.save()
                return {"objects": len(self.objects), "restored": False, "error": e.msg}
            return {"objects": len(self.objects), "restored": True, "dirty": self.dirty}

    def _mark_dirty(self, reason, namespaces):
        self.dirty_namespaces |= set(namespaces)
        self.dirty_reason = reason
        self.log("scene marked DIRTY (%s): %s", sorted(namespaces), reason)

    # ---- meshes ----

    def _resolve_mesh(self, file, sha_expected, path):
        """Validate a mesh reference and make sure it is parsed + cached.
        Returns (relpath, sha256, triangles)."""
        from file_store import FileStoreError
        if not isinstance(file, str) or not file:
            raise _err(400, "bad_shape", "mesh needs a file path in the /files store", f"{path}.file")
        try:
            abs_path = self.store.resolve(file)
        except FileStoreError as e:
            raise _err(400, "bad_shape", e.msg, f"{path}.file")
        if not os.path.isfile(abs_path):
            raise _err(404, "unknown_mesh", f"no file {file!r} in the /files store", f"{path}.file")
        if not abs_path.lower().endswith(MESH_EXTENSIONS):
            raise _err(415, "unsupported_mesh", f"mesh must be one of {MESH_EXTENSIONS}", f"{path}.file")
        sha = self.store.sha256(abs_path)
        if sha_expected is not None and sha_expected != sha:
            raise _err(409, "mesh_sha_mismatch", f"file sha256 is {sha}, request says {sha_expected}",
                       f"{path}.sha256")
        if sha not in self._mesh_cache:
            if os.path.getsize(abs_path) == 0:
                raise _err(422, "mesh_unreadable", "the file is empty (0 bytes)", f"{path}.file")
            try:
                verts, faces = _parse_mesh_isolated(abs_path)
            except SceneError as e:
                raise _err(422, "mesh_unreadable", e.msg, f"{path}.file")
            self._mesh_cache[sha] = (verts, faces)
            self._mesh_cache_tris += len(faces)
            # keep the cache bounded (insertion order = age)
            while self._mesh_cache_tris > 3_000_000 and len(self._mesh_cache) > 1:
                old = next(iter(self._mesh_cache))
                self._mesh_cache_tris -= len(self._mesh_cache.pop(old)[1])
        tris = len(self._mesh_cache[sha][1])
        if tris > self.limits["max_triangles_per_mesh"]:
            raise _err(422, "limits_exceeded",
                       f"mesh has {tris} triangles, limit is {self.limits['max_triangles_per_mesh']} "
                       "(the server does not simplify meshes)", f"{path}.file")
        return self.store.relpath(abs_path), sha, tris

    # ---- validation ----

    def _canon_shape(self, s, path):
        if not isinstance(s, dict):
            raise _err(400, "bad_shape", "shape must be an object", path)
        t = s.get("type")
        if t not in SHAPE_TYPES:
            raise _err(400, "bad_shape", f"type must be one of {SHAPE_TYPES}", f"{path}.type")
        out = {"type": t, "pose": canon_pose(s.get("pose"), f"{path}.pose")}
        if t == "box":
            size = s.get("size")
            if not isinstance(size, list) or len(size) != 3:
                raise _err(400, "bad_shape", "box needs size [x, y, z]", f"{path}.size")
            out["size"] = [_num(v, f"{path}.size[{i}]", positive=True, upper=MAX_DIM_M) for i, v in enumerate(size)]
        elif t == "sphere":
            out["radius"] = _num(s.get("radius"), f"{path}.radius", positive=True, upper=MAX_DIM_M)
        elif t == "cylinder":
            out["radius"] = _num(s.get("radius"), f"{path}.radius", positive=True, upper=MAX_DIM_M)
            out["length"] = _num(s.get("length"), f"{path}.length", positive=True, upper=MAX_DIM_M)
        else:
            scale = s.get("scale", 1.0)
            out["scale"] = _num(scale, f"{path}.scale", positive=True, upper=1000.0)
            sha_exp = s.get("sha256")
            if sha_exp is not None and not isinstance(sha_exp, str):
                raise _err(400, "bad_shape", "sha256 must be a hex string", f"{path}.sha256")
            rel, sha, tris = self._resolve_mesh(s.get("file"), sha_exp, path)
            out.update(file=rel, sha256=sha, triangles=tris)
        return out

    def _check_id(self, oid, ns, path):
        if not isinstance(oid, str) or not oid:
            raise _err(400, "bad_id", "id must be a non-empty string", path)
        if not ID_RE.match(oid):
            raise _err(400, "bad_id", "id must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}", path)
        if oid.split(".", 1)[0] in RESERVED_NS:
            raise _err(400, "bad_id", f"namespaces {RESERVED_NS} are reserved", path)
        if not in_namespace(oid, ns):
            raise _err(400, "bad_id", f"id must start with '{ns}.'", path)

    def check_namespace(self, ns, path="namespace"):
        if not isinstance(ns, str) or not ns or not NS_RE.match(ns):
            raise _err(400, "bad_request", "namespace must be dot-separated [A-Za-z0-9_-] parts", path)
        if ns.split(".", 1)[0] in RESERVED_NS:
            raise _err(400, "bad_request", f"namespaces {RESERVED_NS} are reserved", path)
        return ns

    def _check_frame(self, frame, path):
        frame = frame or "panda_link0"
        if frame not in ("panda_link0", self.planning_frame):
            raise _err(400, "bad_frame", "frame_id must be panda_link0 (the planning frame)", path)
        return frame

    def validate(self, body):
        """Full validation of a /scene/batch body. Parses/caches meshes.
        Returns a request dict; raises SceneError (scene untouched)."""
        if not isinstance(body, dict):
            raise _err(400, "bad_request", "body must be a JSON object")
        ns = self.check_namespace(body.get("namespace"))
        mode = body.get("mode", "replace")
        if mode not in MODES:
            raise _err(400, "bad_request", f"mode must be one of {MODES}", "mode")
        if_version = body.get("if_version")
        if if_version is not None and (isinstance(if_version, bool) or not isinstance(if_version, int)):
            raise _err(400, "bad_request", "if_version must be an integer", "if_version")
        meta = body.get("meta")
        if meta is not None and _json_size(meta) > self.limits["max_batch_meta_bytes"]:
            raise _err(413, "too_large", f"meta exceeds {self.limits['max_batch_meta_bytes']} bytes", "meta")
        req = {"namespace": ns, "mode": mode, "if_version": if_version, "meta": meta,
               "validate_only": bool(body.get("validate_only", False)),
               "keep_attached": bool(body.get("keep_attached", True)),
               "objects": {}, "ids": []}
        if mode == "remove":
            ids = body.get("ids")
            if not isinstance(ids, list) or not ids:
                raise _err(400, "bad_request", "mode=remove needs a non-empty ids list", "ids")
            for i, oid in enumerate(ids):
                self._check_id(oid, ns, f"ids[{i}]")
            req["ids"] = list(dict.fromkeys(ids))
            return req
        objs = body.get("objects")
        if not isinstance(objs, list):
            raise _err(400, "bad_request", "objects must be a list", "objects")
        if len(objs) > self.limits["max_objects_per_batch"]:
            raise _err(422, "limits_exceeded", f"more than {self.limits['max_objects_per_batch']} objects", "objects")
        for i, o in enumerate(objs):
            p = f"objects[{i}]"
            if not isinstance(o, dict):
                raise _err(400, "bad_request", "object must be a JSON object", p)
            oid = o.get("id")
            self._check_id(oid, ns, f"{p}.id")
            if oid in req["objects"]:
                raise _err(400, "bad_id", f"duplicate id {oid!r}", f"{p}.id")
            shapes = o.get("shapes")
            if not isinstance(shapes, list) or not shapes:
                raise _err(400, "bad_shape", "object needs a non-empty shapes list", f"{p}.shapes")
            if len(shapes) > self.limits["max_shapes_per_object"]:
                raise _err(422, "limits_exceeded", f"more than {self.limits['max_shapes_per_object']} shapes", f"{p}.shapes")
            ometa = o.get("meta")
            if ometa is not None and _json_size(ometa) > self.limits["max_object_meta_bytes"]:
                raise _err(413, "too_large", f"meta exceeds {self.limits['max_object_meta_bytes']} bytes", f"{p}.meta")
            req["objects"][oid] = {
                "id": oid,
                "frame_id": self._check_frame(o.get("frame_id"), f"{p}.frame_id"),
                "pose": canon_pose(o.get("pose"), f"{p}.pose"),
                "shapes": [self._canon_shape(s, f"{p}.shapes[{j}]") for j, s in enumerate(shapes)],
                "attached_link": None,
                "meta": ometa,
            }
        return req

    # ---- views ----

    def legacy_objects(self):
        """Legacy (/scene/objects) instances in the v2 object format."""
        out = {}
        if self.legacy is None:
            return out
        for name, info in self.legacy.objects.items():
            model = self.legacy.models.get(info["model"], {})
            p = info["pose"]
            out[name] = {
                "id": name, "frame_id": info.get("frame") or "panda_link0",
                "pose": canon_pose({"position": [p["x"], p["y"], p["z"]],
                                    "orientation": [p["qx"], p["qy"], p["qz"], p["qw"]]}, name),
                "shapes": [{"type": "mesh", "file": model.get("file"), "sha256": model.get("sha256"),
                            "scale": model.get("scale", 1.0), "pose": copy.deepcopy(IDENTITY_POSE)}],
                "attached_link": info.get("attached_link"),
                "meta": {"legacy_model": info["model"], "source": info.get("source")},
                "legacy": True,
            }
        return out

    def all_objects(self):
        objs = dict(self.legacy_objects())
        objs.update(self.objects)
        return objs

    def namespace_hash(self, ns):
        return hash_objects([o for oid, o in self.all_objects().items() if in_namespace(oid, ns)])

    def scene_hash(self):
        return hash_objects(list(self.all_objects().values()))

    def namespaces(self):
        tops = sorted({oid.split(".", 1)[0] for oid in self.all_objects() if "." in oid})
        return {ns: self.namespace_hash(ns) for ns in tops}

    @staticmethod
    def counts(objs):
        prim = sum(1 for o in objs for s in o["shapes"] if s["type"] != "mesh")
        meshes = [s for o in objs for s in o["shapes"] if s["type"] == "mesh"]
        return {"objects": len(objs), "primitives": prim, "meshes": len(meshes),
                "triangles": sum(s.get("triangles", 0) for s in meshes)}

    # ---- commit ----

    def _apply_and_confirm(self, ops, expect_present, expect_absent):
        """Send ops [("add"|"remove", id, obj)] as one diff, check success and
        read back world names. Raises SceneError(504, apply_unconfirmed)."""
        delay = self.test_hooks.get("delay_next_apply_s") or 0.0
        if delay:
            self.test_hooks["delay_next_apply_s"] = 0.0
            time.sleep(delay)
        if self.test_hooks.get("fail_next_apply"):
            self.test_hooks["fail_next_apply"] -= 1
            raise _err(504, "apply_unconfirmed", "apply_planning_scene returned success=false (test hook)")
        ok = self.io.apply(ops, self._mesh_cache)
        if not ok:
            raise _err(504, "apply_unconfirmed", "apply_planning_scene returned success=false")
        world = set(self.io.world_names())
        missing = sorted(expect_present - world)
        lingering = sorted(expect_absent & world)
        if missing or lingering:
            raise _err(504, "apply_unconfirmed", "planning scene does not match after apply",
                       missing=missing[:20], lingering=lingering[:20])

    def _rollback(self, before, touched):
        """Re-apply the pre-batch state of every touched id."""
        world = set(self.io.world_names())
        ops = [("remove", oid, None) for oid in touched if oid in world]
        ops += [("add", oid, before[oid]) for oid in touched if oid in before]
        present = {oid for oid in touched if oid in before}
        self._apply_and_confirm(ops, expect_present=present, expect_absent=set(touched) - present)

    def batch(self, req, motion_lock, lock_timeout_s=5.0):
        """Commit a validated request. Holds ``motion_lock`` only for the
        commit itself (unless validate_only)."""
        t0 = time.time()
        if req["validate_only"]:
            with self._lock:
                return self._plan_batch(req, dry=True, t0=t0)
        if not motion_lock.acquire(timeout=lock_timeout_s):
            raise _err(409, "busy", "robot is busy (motion in progress); scene not changed")
        try:
            with self._lock:
                return self._plan_batch(req, dry=False, t0=t0)
        finally:
            motion_lock.release()

    def _plan_batch(self, req, dry, t0):
        ns, mode = req["namespace"], req["mode"]
        if req["if_version"] is not None and req["if_version"] != self.version:
            raise _err(409, "version_conflict", f"scene_version is {self.version}, request expects {req['if_version']}",
                       scene_version=self.version)
        legacy_in_ns = sorted(n for n in self.legacy_objects() if in_namespace(n, ns))
        if legacy_in_ns:
            raise _err(409, "namespace_owned",
                       "namespace contains objects created through the legacy /scene/objects API; "
                       "DELETE /scene/namespaces/<ns> first", objects=legacy_in_ns[:20])
        current = {oid: o for oid, o in self.objects.items() if in_namespace(oid, ns)}
        target = dict(current)
        kept_attached, warnings = [], []
        if mode == "remove":
            unknown = [oid for oid in req["ids"] if oid not in current]
            if unknown:
                raise _err(404, "unknown_object", f"not in namespace {ns!r}: {unknown[:20]}", objects=unknown[:20])
            for oid in req["ids"]:
                if current[oid].get("attached_link"):
                    raise _err(409, "object_attached", f"{oid!r} is attached; detach it first")
                target.pop(oid)
        else:
            if mode == "replace":
                for oid, o in current.items():
                    if oid not in req["objects"]:
                        if o.get("attached_link"):
                            if not req["keep_attached"]:
                                raise _err(409, "object_attached", f"{oid!r} is attached (keep_attached=false)")
                            kept_attached.append(oid)
                        else:
                            target.pop(oid)
            for oid, o in req["objects"].items():
                if current.get(oid, {}).get("attached_link"):
                    if mode == "replace" and req["keep_attached"]:
                        kept_attached.append(oid)
                        continue
                    raise _err(409, "object_attached", f"{oid!r} is attached; detach it first")
                target[oid] = o
        # limits on the resulting scene
        all_after = [o for oid, o in self.objects.items() if not in_namespace(oid, ns)] + list(target.values())
        c_all = self.counts(all_after)
        if c_all["primitives"] > self.limits["max_primitives_total"]:
            raise _err(422, "limits_exceeded", f"scene would have {c_all['primitives']} primitives "
                       f"(limit {self.limits['max_primitives_total']})")
        if c_all["triangles"] > self.limits["max_triangles_total"]:
            raise _err(422, "limits_exceeded", f"scene would have {c_all['triangles']} mesh triangles "
                       f"(limit {self.limits['max_triangles_total']})")
        added = sorted(oid for oid in target if oid not in current)
        removed = sorted(oid for oid in current if oid not in target)
        updated = sorted(oid for oid in target if oid in current
                         and hash_object(target[oid]) != hash_object(current[oid]))
        meta_only = [oid for oid in target if oid in current and oid not in updated
                     and target[oid].get("meta") != current[oid].get("meta")]
        world = set() if dry else set(self.io.world_names())
        resync = self._is_dirty(ns)
        if resync:
            # move_group's content for this namespace is unknown: drop whatever
            # carries the prefix and re-send the whole target.
            stray = sorted(oid for oid in world if in_namespace(oid, ns) and oid not in target)
            missing = sorted(oid for oid in target if oid not in added and not target[oid].get("attached_link"))
            removed = sorted(set(removed) | set(stray))
            warnings.append("namespace was dirty: fully re-synchronised")
        else:
            missing = [] if dry else sorted(oid for oid in target if oid not in added and oid not in world
                                            and not target[oid].get("attached_link"))
        if missing and not resync:
            warnings.append(f"{len(missing)} object(s) were missing from move_group (restarted?) and are re-sent")
        ns_hash_after = hash_objects(list(target.values()))
        result = {"ok": True, "namespace": ns, "mode": mode, "added": added, "updated": updated,
                  "removed": removed, "kept_attached": sorted(set(kept_attached)),
                  "counts": self.counts(list(target.values())), "namespace_hash": ns_hash_after,
                  "warnings": warnings}
        changed = bool(added or updated or removed or missing or resync)
        if dry:
            result.update(validate_only=True, would_change=changed, scene_version=self.version,
                          apply_ms=int((time.time() - t0) * 1000))
            return result
        if not changed:
            for oid in meta_only:
                self.objects[oid]["meta"] = target[oid].get("meta")
            if req["meta"] is not None:
                self.batch_meta[ns] = req["meta"]
            if meta_only or req["meta"] is not None:
                self.save()
            result.update(unchanged=True, confirmed=True, scene_version=self.version,
                          scene_hash=self.scene_hash(), apply_ms=int((time.time() - t0) * 1000))
            return result
        touched = sorted(set(added) | set(updated) | set(removed) | set(missing))
        ops = [("remove", oid, None) for oid in sorted(set(updated) | set(removed) | set(missing)) if oid in world]
        ops += [("add", oid, target[oid]) for oid in sorted(set(added) | set(updated) | set(missing))]
        before = copy.deepcopy(current)
        try:
            self._apply_and_confirm(ops, expect_present=set(added) | set(updated) | set(missing),
                                    expect_absent=set(removed))
        except SceneError as e:
            try:
                self._rollback(before, touched)
                e.extra["rolled_back"] = True
            except SceneError as e2:
                self._mark_dirty(f"batch on {ns!r} failed ({e.msg}) and rollback failed ({e2.msg})", {ns})
                e.extra["rolled_back"] = False
            e.extra["dirty"] = self.dirty
            self.save()
            raise
        for oid in removed:
            self.objects.pop(oid, None)
        for oid in target:
            self.objects[oid] = target[oid]
        if req["meta"] is not None:
            self.batch_meta[ns] = req["meta"]
        self.version += 1
        self._clear_dirty(ns)
        self.updated_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        self.save()
        result.update(unchanged=False, confirmed=True, scene_version=self.version,
                      scene_hash=self.scene_hash(), apply_ms=int((time.time() - t0) * 1000))
        return result

    def delete_namespace(self, ns, motion_lock, force_detach=False, if_version=None, lock_timeout_s=5.0):
        ns = self.check_namespace(ns)
        if not motion_lock.acquire(timeout=lock_timeout_s):
            raise _err(409, "busy", "robot is busy (motion in progress); scene not changed")
        try:
            with self._lock:
                if if_version is not None and if_version != self.version:
                    raise _err(409, "version_conflict", f"scene_version is {self.version}",
                               scene_version=self.version)
                ours = sorted(oid for oid in self.objects if in_namespace(oid, ns))
                legacy = sorted(n for n in self.legacy_objects() if in_namespace(n, ns))
                attached = [oid for oid in ours if self.objects[oid].get("attached_link")]
                attached += [n for n in legacy if self.legacy.objects[n].get("attached_link")]
                if attached and not force_detach:
                    raise _err(409, "object_attached", f"attached objects in namespace: {attached}",
                               objects=attached)
                world = set(self.io.world_names())
                legacy_names = set(self.legacy_objects())
                stray = sorted(oid for oid in world if in_namespace(oid, ns)
                               and oid not in self.objects and oid not in legacy_names)
                if ours or stray:
                    before = {oid: self.objects[oid] for oid in ours}
                    ops = [("remove", oid, None) for oid in ours + stray if oid in world]
                    try:
                        self._apply_and_confirm(ops, expect_present=set(), expect_absent=set(ours))
                    except SceneError as e:
                        try:
                            self._rollback(before, ours)
                        except SceneError as e2:
                            self._mark_dirty(f"namespace delete failed ({e.msg}), rollback failed ({e2.msg})", {ns})
                        self.save()
                        raise
                    for oid in ours:
                        self.objects.pop(oid)
                hook, self.legacy.on_change = self.legacy.on_change, None   # we bump once below
                try:
                    for n in legacy:
                        if self.legacy.objects[n].get("attached_link"):
                            self.legacy.detach(n, persist=False)
                        self.legacy.remove_object(n, persist=True)
                finally:
                    self.legacy.on_change = hook
                for k in [k for k in self.batch_meta if k == ns or k.startswith(ns + ".")]:
                    self.batch_meta.pop(k)
                removed = ours + legacy + stray
                if removed or self._is_dirty(ns):
                    self.version += 1
                    self._clear_dirty(ns)
                    self.updated_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
                    self.save()
                return {"removed": removed, "scene_version": self.version, "scene_hash": self.scene_hash()}
        finally:
            motion_lock.release()

    def bump_for_legacy_change(self):
        """Legacy /scene/objects mutations change the scene hash too."""
        with self._lock:
            self.version += 1
            self.updated_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
            self.save()

    # ---- listing ----

    def list_objects(self, namespace=None, geometry=False, world_names=None, attached=None):
        objs = self.all_objects()
        out = []
        for oid in sorted(objs):
            o = objs[oid]
            ns = namespace_of(oid)
            if namespace and not (ns == namespace or ns.startswith(namespace + ".")):
                continue
            item = {"id": oid, "namespace": ns, "frame_id": o["frame_id"], "pose": o["pose"],
                    "attached_link": o.get("attached_link"), "meta": o.get("meta")}
            if geometry:
                item["shapes"] = [{k: v for k, v in s.items() if k != "triangles"} for s in o["shapes"]]
            out.append(item)
        if namespace in (None, "builtin") and world_names is not None:
            for n in sorted(world_names):
                if n in BUILTIN_OBJECTS:
                    out.append({"id": n, "namespace": "builtin", "frame_id": self.planning_frame,
                                "pose": None, "attached_link": None, "meta": None})
            if self.builtin_attached:
                b = self.builtin_attached
                out.append({"id": b["id"], "namespace": "builtin", "frame_id": b["frame"],
                            "pose": {"position": b["center"], "orientation": [0, 0, 0, 1]},
                            "attached_link": b["link"], "meta": {"placeholder": b["placeholder"]},
                            **({"shapes": [{"type": "box", "size": b["size"], "pose": IDENTITY_POSE}]} if geometry else {})})
        return out


class MoveItSceneIO:
    """The only place that talks to move_group for the v2 scene."""

    def __init__(self, timeout_s=10.0):
        import rospy
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
        rospy.wait_for_service("/apply_planning_scene", timeout=timeout_s)
        rospy.wait_for_service("/get_planning_scene", timeout=timeout_s)
        self._apply = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)
        self._get = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
        self._mesh_msgs = {}      # (sha, scale) -> shape_msgs/Mesh

    def world_names(self):
        from moveit_msgs.msg import PlanningSceneComponents
        res = self._get(PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_NAMES))
        return [co.id for co in res.scene.world.collision_objects]

    def attached_names(self):
        from moveit_msgs.msg import PlanningSceneComponents
        res = self._get(PlanningSceneComponents(components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS))
        return [a.object.id for a in res.scene.robot_state.attached_collision_objects]

    def _mesh_msg(self, sha, scale, mesh_cache):
        key = (sha, scale)
        if key not in self._mesh_msgs:
            from shape_msgs.msg import Mesh, MeshTriangle
            from geometry_msgs.msg import Point
            verts, faces = mesh_cache[sha]
            m = Mesh()
            m.vertices = [Point(x=float(x) * scale, y=float(y) * scale, z=float(z) * scale) for x, y, z in verts]
            m.triangles = [MeshTriangle(vertex_indices=[int(a), int(b), int(c)]) for a, b, c in faces]
            if len(self._mesh_msgs) > 256:
                self._mesh_msgs.clear()
            self._mesh_msgs[key] = m
        return self._mesh_msgs[key]

    @staticmethod
    def _pose_msg(p):
        from geometry_msgs.msg import Pose
        m = Pose()
        m.position.x, m.position.y, m.position.z = p["position"]
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = p["orientation"]
        return m

    def collision_object(self, obj, mesh_cache):
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        co = CollisionObject()
        co.id = obj["id"]
        co.header.frame_id = obj["frame_id"]
        co.pose = self._pose_msg(obj["pose"])
        for s in obj["shapes"]:
            t = s["type"]
            if t == "mesh":
                co.meshes.append(self._mesh_msg(s["sha256"], s["scale"], mesh_cache))
                co.mesh_poses.append(self._pose_msg(s["pose"]))
                continue
            prim = SolidPrimitive()
            if t == "box":
                prim.type, prim.dimensions = SolidPrimitive.BOX, list(s["size"])
            elif t == "sphere":
                prim.type, prim.dimensions = SolidPrimitive.SPHERE, [s["radius"]]
            else:   # MoveIt order is [height, radius] -- opposite of the API field order
                prim.type, prim.dimensions = SolidPrimitive.CYLINDER, [s["length"], s["radius"]]
            co.primitives.append(prim)
            co.primitive_poses.append(self._pose_msg(s["pose"]))
        co.operation = CollisionObject.ADD
        return co

    def attach_box(self, obj_id, link, frame, size, center, touch_links):
        """Attach an axis-aligned box (pose given in ``frame``) to ``link`` in
        one confirmed diff. Used for built-in robot accessories (wrist camera)."""
        from moveit_msgs.msg import PlanningScene, AttachedCollisionObject, CollisionObject
        from shape_msgs.msg import SolidPrimitive
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.touch_links = list(touch_links)
        co = aco.object
        co.id = obj_id
        co.header.frame_id = frame
        co.pose = self._pose_msg({"position": list(center), "orientation": [0, 0, 0, 1]})
        co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))]
        co.primitive_poses = [self._pose_msg({"position": [0, 0, 0], "orientation": [0, 0, 0, 1]})]
        co.operation = CollisionObject.ADD
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = [aco]
        ok = bool(self._apply(ps).success)
        return ok and obj_id in self.attached_names()

    def apply(self, ops, mesh_cache):
        from moveit_msgs.msg import PlanningScene, CollisionObject
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        for op, oid, obj in ops:
            if op == "remove":
                if not oid:      # an empty id REMOVE wipes the whole world
                    raise ValueError("refusing REMOVE with empty id")
                co = CollisionObject()
                co.id = oid
                co.operation = CollisionObject.REMOVE
            else:
                co = self.collision_object(obj, mesh_cache)
            ps.world.collision_objects.append(co)
        return bool(self._apply(ps).success)
