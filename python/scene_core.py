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
        self.acm_entries = {}                 # "a|b" -> {a, b, allowed, original}  (written by API clients)
        self.padding = None                   # {"default_m": x, "links": {link: m}}
        self.robot_links = lambda: []         # set by restful.py

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
                "dirty_namespaces": sorted(self.dirty_namespaces), "dirty_reason": self.dirty_reason,
                "acm_entries": self.acm_entries, "padding": self.padding, "updated_at": self.updated_at,
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
            self.acm_entries = data.get("acm_entries", {}) or {}
            self.padding = data.get("padding")
            if not self.objects and not self.acm_entries and not self.padding:
                return {"objects": 0, "restored": True, "dirty": self.dirty}
            try:
                for o in self.objects.values():
                    for i, s in enumerate(o["shapes"]):
                        if s["type"] == "mesh":
                            self._resolve_mesh(s["file"], s.get("sha256"), f"{o['id']}.shapes[{i}]")
                world = set(self.io.world_names())
                ops = [("remove", oid, None) for oid in self.objects if oid in world]
                ops += [("add", oid, o) for oid, o in self.objects.items()]
                # attachments do not survive a move_group restart: put them back
                ops += [("attach", oid, {"link": o["attached_link"], "touch_links": o.get("touch_links")})
                        for oid, o in self.objects.items() if o.get("attached_link")]
                acm = self._acm_msg_with(self.acm_entries) if self.acm_entries else None
                pad = self._padding_map(self.padding) if self.padding else None
                attached = {oid for oid, o in self.objects.items() if o.get("attached_link")}
                self._apply_and_confirm(ops, expect_present=set(self.objects) - attached, expect_absent=set(),
                                        acm=acm, padding=pad, expect_attached=attached)
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
        """sha256 over {objects (hash_object list), acm: [[a, b, allowed]...]
        sorted, padding: {default_m, links} as %.6f strings or null}."""
        objs = [hash_object(o) for o in sorted(self.all_objects().values(), key=lambda o: o["id"])]
        acm = sorted([e["a"], e["b"], bool(e["allowed"])] for e in self.acm_entries.values())
        pad = None
        if self.padding:
            pad = {"default_m": _fmt(self.padding["default_m"], 6),
                   "links": {k: _fmt(v, 6) for k, v in sorted(self.padding.get("links", {}).items())}}
        raw = json.dumps({"objects": objs, "acm": acm, "padding": pad}, sort_keys=True, separators=(",", ":"))
        return "sha256:" + hashlib.sha256(raw.encode("utf-8")).hexdigest()

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

    def _apply_and_confirm(self, ops, expect_present, expect_absent, acm=None, padding=None,
                           expect_attached=frozenset(), expect_detached=frozenset()):
        """Send ops as one diff, check success and read back world names (and
        attached names). Raises SceneError(504, apply_unconfirmed)."""
        delay = self.test_hooks.get("delay_next_apply_s") or 0.0
        if delay:
            self.test_hooks["delay_next_apply_s"] = 0.0
            time.sleep(delay)
        if self.test_hooks.get("fail_next_apply"):
            self.test_hooks["fail_next_apply"] -= 1
            raise _err(504, "apply_unconfirmed", "apply_planning_scene returned success=false (test hook)")
        ok = self.io.apply(ops, self._mesh_cache, acm=acm, padding=padding)
        if not ok:
            raise _err(504, "apply_unconfirmed", "apply_planning_scene returned success=false")
        world = set(self.io.world_names())
        missing = sorted(expect_present - world)
        lingering = sorted(expect_absent & world)
        if expect_attached or expect_detached:
            att = set(self.io.attached_names())
            missing += sorted(set(expect_attached) - att)
            lingering += sorted(set(expect_detached) & att)
        if missing or lingering:
            raise _err(504, "apply_unconfirmed", "planning scene does not match after apply",
                       missing=missing[:20], lingering=lingering[:20])

    def _rollback(self, before, touched, acm_before=None):
        """Re-apply the pre-batch state of every touched id (and the ACM,
        which MoveIt applies before the world part of a failed diff)."""
        world = set(self.io.world_names())
        attached = set(self.io.attached_names())
        ops = [("detach", oid, {"link": before[oid]["attached_link"]}) for oid in touched
               if oid in attached and oid in before and before[oid].get("attached_link")]
        ops += [("remove", oid, None) for oid in touched if oid in world or oid in attached]
        ops += [("add", oid, before[oid]) for oid in touched if oid in before]
        ops += [("attach", oid, {"link": before[oid]["attached_link"], "touch_links": before[oid].get("touch_links")})
                for oid in touched if oid in before and before[oid].get("attached_link")]
        present = {oid for oid in touched if oid in before and not before[oid].get("attached_link")}
        self._apply_and_confirm(ops, expect_present=present, expect_absent=set(touched) - present -
                                {oid for oid in touched if oid in before}, acm=acm_before)

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
        acm_msg, acm_drop = self._acm_for_removed(set(removed))
        acm_before = self.io.get_acm() if acm_msg is not None else None
        try:
            self._apply_and_confirm(ops, expect_present=set(added) | set(updated) | set(missing),
                                    expect_absent=set(removed), acm=acm_msg)
        except SceneError as e:
            try:
                self._rollback(before, touched, acm_before)
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
        for k in acm_drop:
            self.acm_entries.pop(k, None)
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
                acm_msg, acm_drop = self._acm_for_removed(set(ours) | set(legacy) | set(stray))
                if ours or stray or acm_msg is not None:
                    before = {oid: self.objects[oid] for oid in ours}
                    acm_before = self.io.get_acm() if acm_msg is not None else None
                    ops = [("detach", oid, {"link": self.objects[oid]["attached_link"]})
                           for oid in ours if self.objects[oid].get("attached_link")]
                    ops += [("remove", oid, None) for oid in ours + stray
                            if oid in world or self.objects.get(oid, {}).get("attached_link")]
                    try:
                        self._apply_and_confirm(ops, expect_present=set(), expect_absent=set(ours), acm=acm_msg)
                    except SceneError as e:
                        try:
                            self._rollback(before, ours, acm_before)
                        except SceneError as e2:
                            self._mark_dirty(f"namespace delete failed ({e.msg}), rollback failed ({e2.msg})", {ns})
                        self.save()
                        raise
                    for oid in ours:
                        self.objects.pop(oid)
                    for k in acm_drop:
                        self.acm_entries.pop(k, None)
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

    # ---- ACM ----

    @staticmethod
    def _pair_key(a, b):
        a, b = sorted((a, b))
        return f"{a}|{b}"

    def _acm_msg_with(self, entries, drop_names=()):
        """Current scene ACM with ``entries`` ({key: {a, b, allowed}}) written
        over it and ``drop_names`` removed (their rows/columns)."""
        from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
        cur = self.io.get_acm()
        names = list(cur.entry_names)
        vals = {}
        for i, a in enumerate(names):
            for j, b in enumerate(names):
                vals[(a, b)] = bool(cur.entry_values[i].enabled[j])
        for e in entries.values():
            for n in (e["a"], e["b"]):
                if n not in names:
                    names.append(n)
            vals[(e["a"], e["b"])] = vals[(e["b"], e["a"])] = bool(e["allowed"])
        names = [n for n in names if n not in set(drop_names)]
        msg = AllowedCollisionMatrix()
        msg.entry_names = names
        # The message is dense: every pair of listed names becomes an explicit
        # entry. A pair that was NOT explicit before must get the value MoveIt
        # derives from the default entries (e.g. the *_sc capsules are
        # default-allowed with everything) -- writing False there would make
        # every new object collide with them.
        msg.entry_values = [AllowedCollisionEntry(enabled=[vals[(a, b)] if (a, b) in vals
                                                           else self._default_allowed(cur, a, b)
                                                           for b in names]) for a in names]
        msg.default_entry_names = list(cur.default_entry_names)
        msg.default_entry_values = list(cur.default_entry_values)
        return msg

    @staticmethod
    def _default_allowed(acm, a, b):
        """MoveIt's fallback for a pair without an explicit entry: allowed if
        either name has a default entry that allows everything."""
        d = dict(zip(acm.default_entry_names, acm.default_entry_values))
        return bool(d.get(a, False) or d.get(b, False))

    def _acm_value(self, acm, a, b):
        names = list(acm.entry_names)
        if a in names and b in names:
            return bool(acm.entry_values[names.index(a)].enabled[names.index(b)])
        return None

    def _known_names(self):
        names = set(self.robot_links()) | set(self.all_objects())
        try:
            names |= set(self.io.world_names()) | set(self.io.attached_names())
        except Exception:
            pass
        return names

    def acm_get(self, names=None):
        acm = self.io.get_acm()
        pick = set(names) if names else None
        entries = []
        all_names = list(acm.entry_names)
        for i, a in enumerate(all_names):
            for j in range(i + 1, len(all_names)):
                b = all_names[j]
                if pick is not None and not (a in pick or b in pick):
                    continue
                if acm.entry_values[i].enabled[j]:
                    key = self._pair_key(a, b)
                    entries.append({"a": a, "b": b, "allowed": True, "client": key in self.acm_entries})
        return {"entries": entries, "client_entries": list(self.acm_entries.values()),
                "default_entries": dict(zip(acm.default_entry_names, acm.default_entry_values)),
                "scene_version": self.version}

    def acm_set(self, entries, motion_lock, if_version=None, lock_timeout_s=5.0):
        if not isinstance(entries, list) or not entries:
            raise _err(400, "bad_request", "entries must be a non-empty list", "entries")
        known = self._known_names()
        parsed = []
        for i, e in enumerate(entries):
            if not isinstance(e, dict) or not isinstance(e.get("a"), str) or not isinstance(e.get("b"), str):
                raise _err(400, "bad_request", "entry needs string a, b and bool allowed", f"entries[{i}]")
            if e["a"] == e["b"]:
                raise _err(400, "bad_request", "a and b must differ", f"entries[{i}]")
            for k in ("a", "b"):
                if e[k] not in known:
                    raise _err(404, "unknown_object", f"{e[k]!r} is neither a robot link nor a scene object", f"entries[{i}].{k}")
            if not isinstance(e.get("allowed"), bool):
                raise _err(400, "bad_request", "allowed must be true/false", f"entries[{i}].allowed")
            parsed.append(e)
        if not motion_lock.acquire(timeout=lock_timeout_s):
            raise _err(409, "busy", "robot is busy (motion in progress); scene not changed")
        try:
            with self._lock:
                if if_version is not None and if_version != self.version:
                    raise _err(409, "version_conflict", f"scene_version is {self.version}", scene_version=self.version)
                cur = self.io.get_acm()
                new_entries = dict(self.acm_entries)
                for e in parsed:
                    key = self._pair_key(e["a"], e["b"])
                    a, b = sorted((e["a"], e["b"]))
                    original = new_entries[key]["original"] if key in new_entries else self._acm_value(cur, a, b)
                    new_entries[key] = {"a": a, "b": b, "allowed": e["allowed"], "original": original}
                msg = self._acm_msg_with(new_entries)
                self._apply_and_confirm([], set(), set(), acm=msg)
                after = self.io.get_acm()
                wrong = [k for k, e in new_entries.items() if self._acm_value(after, e["a"], e["b"]) != e["allowed"]]
                if wrong:
                    raise _err(504, "apply_unconfirmed", "ACM read-back does not match", pairs=wrong[:20])
                self.acm_entries = new_entries
                self.version += 1
                self.updated_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
                self.save()
                return {"entries": list(new_entries.values()), "scene_version": self.version,
                        "scene_hash": self.scene_hash()}
        finally:
            motion_lock.release()

    def _acm_restore_map(self, keys):
        """Entries that put the given client entries back to their original
        value (no explicit entry before -> the default-entry value)."""
        cur = self.io.get_acm()
        out = {}
        for k in keys:
            e = self.acm_entries[k]
            orig = e["original"]
            out[k] = dict(e, allowed=bool(orig) if orig is not None else self._default_allowed(cur, e["a"], e["b"]))
        return out

    def acm_delete(self, motion_lock, namespace=None, if_version=None, lock_timeout_s=5.0):
        """Restore the original value of client-written entries (all, or those
        touching an object in ``namespace``)."""
        if namespace is not None:
            self.check_namespace(namespace)
        if not motion_lock.acquire(timeout=lock_timeout_s):
            raise _err(409, "busy", "robot is busy (motion in progress); scene not changed")
        try:
            with self._lock:
                if if_version is not None and if_version != self.version:
                    raise _err(409, "version_conflict", f"scene_version is {self.version}", scene_version=self.version)
                keys = [k for k, e in self.acm_entries.items()
                        if namespace is None or in_namespace(e["a"], namespace) or in_namespace(e["b"], namespace)]
                if keys:
                    msg = self._acm_msg_with(self._acm_restore_map(keys))
                    self._apply_and_confirm([], set(), set(), acm=msg)
                    for k in keys:
                        self.acm_entries.pop(k)
                    self.version += 1
                    self.updated_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
                    self.save()
                return {"removed": keys, "scene_version": self.version, "scene_hash": self.scene_hash()}
        finally:
            motion_lock.release()

    def _acm_for_removed(self, removed_ids):
        """(acm msg or None, keys to forget) when objects disappear: drop their
        client entries and their rows in the ACM (MoveIt does not clean up)."""
        removed = set(removed_ids)
        if not removed:
            return None, []
        keys = [k for k, e in self.acm_entries.items() if e["a"] in removed or e["b"] in removed]
        names = set(self.io.get_acm().entry_names)
        if not keys and not (names & removed):
            return None, []
        keep = {k: e for k, e in self.acm_entries.items() if k not in keys}
        return self._acm_msg_with(keep, drop_names=removed), keys

    # ---- padding ----

    def _padding_map(self, pad):
        links = self.robot_links()
        return {l: float(pad.get("links", {}).get(l, pad["default_m"])) for l in links}

    def padding_set(self, default_m, links, motion_lock, if_version=None, lock_timeout_s=5.0):
        robot_links = set(self.robot_links())
        dv = _num(default_m, "default_m")
        if not (0.0 <= dv <= 0.1):
            raise _err(400, "bad_request", "default_m must be in 0..0.1", "default_m")
        links = links or {}
        if not isinstance(links, dict):
            raise _err(400, "bad_request", "links must be {link: metres}", "links")
        clean = {}
        for k, v in links.items():
            if k not in robot_links:
                raise _err(404, "unknown_object", f"{k!r} is not a robot link", f"links.{k}")
            v = _num(v, f"links.{k}")
            if not (0.0 <= v <= 0.1):
                raise _err(400, "bad_request", "padding must be in 0..0.1 m", f"links.{k}")
            clean[k] = v
        pad = {"default_m": dv, "links": clean}
        if not motion_lock.acquire(timeout=lock_timeout_s):
            raise _err(409, "busy", "robot is busy (motion in progress); scene not changed")
        try:
            with self._lock:
                if if_version is not None and if_version != self.version:
                    raise _err(409, "version_conflict", f"scene_version is {self.version}", scene_version=self.version)
                want = self._padding_map(pad)
                self._apply_and_confirm([], set(), set(), padding=want)
                got = self.io.get_padding()
                wrong = [l for l, v in want.items() if abs(got.get(l, -1) - v) > 1e-9]
                if wrong:
                    raise _err(504, "apply_unconfirmed", "link padding read-back does not match", links=wrong[:20])
                self.padding = None if (dv == 0 and not any(clean.values())) else pad
                self.version += 1
                self.updated_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
                self.save()
                return {"padding": pad, "applied": want, "scene_version": self.version, "scene_hash": self.scene_hash()}
        finally:
            motion_lock.release()

    # ---- attach / detach (v2 objects; legacy objects go through the same diff) ----

    def attach(self, oid, link, touch_links, motion_lock, lock_timeout_s=5.0):
        if link not in set(self.robot_links()):
            raise _err(400, "bad_request", f"{link!r} is not a robot link", "link")
        touch = list(touch_links) if touch_links else [link, "panda_leftfinger", "panda_rightfinger", "panda_hand_tcp"]
        bad = [t for t in touch if t not in set(self.robot_links())]
        if bad:
            raise _err(400, "bad_request", f"touch_links not robot links: {bad}", "touch_links")
        if not motion_lock.acquire(timeout=lock_timeout_s):
            raise _err(409, "busy", "robot is busy (motion in progress); scene not changed")
        try:
            with self._lock:
                o = self.objects.get(oid)
                if o is None:
                    raise _err(404, "unknown_object", f"no /scene/batch object {oid!r}")
                if o.get("attached_link"):
                    raise _err(409, "object_attached", f"{oid!r} is already attached to {o['attached_link']}")
                self._apply_and_confirm([("attach", oid, {"link": link, "touch_links": touch})],
                                        set(), {oid}, expect_attached={oid})
                o["attached_link"], o["touch_links"] = link, touch     # only after MoveIt confirmed it
                self.version += 1
                self.updated_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
                self.save()
                return {"id": oid, "attached_link": link, "touch_links": touch, "confirmed": True,
                        "scene_version": self.version, "scene_hash": self.scene_hash()}
        finally:
            motion_lock.release()

    def detach(self, oid, motion_lock, lock_timeout_s=5.0):
        if not motion_lock.acquire(timeout=lock_timeout_s):
            raise _err(409, "busy", "robot is busy (motion in progress); scene not changed")
        try:
            with self._lock:
                o = self.objects.get(oid)
                if o is None:
                    raise _err(404, "unknown_object", f"no /scene/batch object {oid!r}")
                if not o.get("attached_link"):
                    raise _err(409, "bad_request", f"{oid!r} is not attached")
                self._apply_and_confirm([("detach", oid, {"link": o["attached_link"]})],
                                        {oid}, set(), expect_detached={oid})
                pose = self.io.world_poses({oid}).get(oid)      # MoveIt left it where the hand was
                if pose is not None:
                    o["pose"] = canon_pose(pose, oid)
                o["frame_id"] = self.planning_frame
                o["attached_link"] = None
                o.pop("touch_links", None)
                self.version += 1
                self.updated_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
                self.save()
                return {"id": oid, "attached_link": None, "pose": o["pose"], "confirmed": True,
                        "scene_version": self.version, "scene_hash": self.scene_hash()}
        finally:
            motion_lock.release()

    def io_attach_existing(self, oid, link, touch_links):
        """For the legacy SceneManager: version-independent attach via one diff."""
        self._apply_and_confirm([("attach", oid, {"link": link, "touch_links": touch_links})],
                                set(), {oid}, expect_attached={oid})

    def io_detach_existing(self, oid, link):
        self._apply_and_confirm([("detach", oid, {"link": link})], {oid}, set(), expect_detached={oid})

    # ---- snapshot ----

    def snapshot(self, namespace=None):
        objs = self.all_objects()
        if namespace:
            objs = {k: v for k, v in objs.items() if in_namespace(k, namespace)}
        acm = list(self.acm_entries.values())
        if namespace:
            acm = [e for e in acm if in_namespace(e["a"], namespace) or in_namespace(e["b"], namespace)]
        out = {"api_version": API_VERSION, "scene_version": self.version, "scene_hash": self.scene_hash(),
               "dirty": self.dirty, "dirty_namespaces": sorted(self.dirty_namespaces), "updated_at": self.updated_at,
               "objects": [{k: v for k, v in o.items() if k != "legacy"} for _, o in sorted(objs.items())],
               "acm": acm, "padding": self.padding, "wrist_camera": self.builtin_attached}
        if namespace:
            out["namespace"] = namespace
            out["namespace_hash"] = self.namespace_hash(namespace)
        return out

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

    def get_acm(self):
        from moveit_msgs.msg import PlanningSceneComponents
        res = self._get(PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX))
        return res.scene.allowed_collision_matrix

    def get_padding(self):
        from moveit_msgs.msg import PlanningSceneComponents
        res = self._get(PlanningSceneComponents(components=PlanningSceneComponents.LINK_PADDING_AND_SCALING))
        return {p.link_name: p.padding for p in res.scene.link_padding}

    def world_poses(self, ids):
        """{id: pose dict} for world objects (object-level pose)."""
        from moveit_msgs.msg import PlanningSceneComponents
        res = self._get(PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY))
        out = {}
        for co in res.scene.world.collision_objects:
            if co.id in ids:
                p = co.pose
                out[co.id] = {"position": [p.position.x, p.position.y, p.position.z],
                              "orientation": [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]}
        return out

    def apply(self, ops, mesh_cache, acm=None, padding=None):
        """One PlanningScene diff. ops: ("remove", id, None), ("add", id, obj),
        ("attach", id, {"link", "touch_links"}) -- attaches the EXISTING world
        object --, ("detach", id, {"link"}) -- puts it back into the world.
        acm: a full AllowedCollisionMatrix msg (replaces the scene's ACM);
        padding: {link: metres}. MoveIt processes robot_state before world,
        so detach + remove of the same id work in one diff."""
        from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject, LinkPadding
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        if acm is not None:
            ps.allowed_collision_matrix = acm
        if padding:
            ps.link_padding = [LinkPadding(link_name=k, padding=float(v)) for k, v in sorted(padding.items())]
        for op, oid, obj in ops:
            if not oid:          # an empty id would address ALL objects
                raise ValueError("refusing scene op with empty id")
            if op in ("attach", "detach"):
                aco = AttachedCollisionObject()
                aco.link_name = obj["link"]
                aco.object.id = oid
                if op == "attach":
                    aco.touch_links = list(obj.get("touch_links") or [obj["link"]])
                    aco.object.operation = CollisionObject.ADD
                else:
                    aco.object.operation = CollisionObject.REMOVE
                ps.robot_state.attached_collision_objects.append(aco)
                continue
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
