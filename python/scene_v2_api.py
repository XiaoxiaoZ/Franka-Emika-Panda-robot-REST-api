"""Scene API v2: GET /scene/info, POST /scene/batch, DELETE /scene/namespaces/<ns>.

None of these endpoints move the robot or the gripper (safety rule S1):
this module only talks to move_group's planning-scene services through
scene_core. See scene_core.py for the batch semantics and hash definition.
"""
import json
import os

from flask import Blueprint, jsonify, request

from scene_core import API_VERSION, SHAPE_TYPES, SceneError

bp = Blueprint("scene_v2", __name__)
_core = None
_lock = None
_robot_info = None        # callable -> dict (planning frame, groups, versions ...)

MESH_FORMATS = [".stl", ".obj", ".dae", ".ply"]


def init(core, motion_lock, robot_info):
    global _core, _lock, _robot_info
    _core, _lock, _robot_info = core, motion_lock, robot_info


def core():
    return _core


@bp.errorhandler(SceneError)
def _on_scene_error(e):
    return jsonify({"error": e.code, "msg": e.msg, **e.extra}), e.status


def _json_body():
    """Parse the JSON body with our own size cap (MAX_CONTENT_LENGTH would
    also cap the 100 MB /files uploads)."""
    limit = _core.limits["max_body_bytes"]
    if request.content_length is not None and request.content_length > limit:
        raise SceneError(413, "too_large", f"body exceeds {limit} bytes")
    raw = request.stream.read(limit + 1)
    if len(raw) > limit:
        raise SceneError(413, "too_large", f"body exceeds {limit} bytes")
    try:
        return json.loads(raw.decode("utf-8") or "null")
    except (UnicodeDecodeError, ValueError) as e:
        raise SceneError(400, "bad_request", f"body is not valid JSON: {e}")


@bp.route("/scene/info", methods=["GET"])
def scene_info():
    """Scene API v2 capabilities and state: planning frame (and its
    transform to panda_link0), robot description hashes, groups, scene
    version + hash + per-namespace hashes, dirty flag, limits, shape types,
    mesh formats, and the exact hash definition. Call before pushing."""
    info = {"api_version": API_VERSION}
    info.update(_robot_info())
    with _core._lock:
        info.update({
            "scene_version": _core.version,
            "scene_hash": _core.scene_hash(),
            "namespace_hashes": _core.namespaces(),
            "dirty": _core.dirty,
            "dirty_namespaces": sorted(_core.dirty_namespaces),
            "dirty_reason": _core.dirty_reason,
            "updated_at": _core.updated_at,
        })
    info.update({
        "motion_enabled": True,
        "wrist_camera": _core.builtin_attached,
        "builtin_attached": [b["id"] for b in ([_core.builtin_attached] if _core.builtin_attached else []) + _core.builtin_extra],
        "flange_to_tcp_m": __import__("panda_ik").flange_to_tcp(),
        "acm_client_entries": len(_core.acm_entries),
        "padding": _core.padding,
        "limits": {k: v for k, v in _core.limits.items()},
        "shape_types": list(SHAPE_TYPES),
        "mesh_formats": MESH_FORMATS,
        "hash_spec": "scene_hash = sha256 of {objects: [per-object dicts], acm: [[a,b,allowed]...] sorted "
                     "(client entries), padding: {default_m, links} (%.6f strings) | null}; namespace_hash = "
                     "sha256 of the per-object list only. Per object, sorted by id: {id, frame_id, pose:{position:[%.6f x3], "
                     "orientation:[%.9f x4, normalised, w>=0]}, attached_link, shapes:[{type, "
                     "dims:[%.6f...] (box [x,y,z], sphere [r], cylinder [r,length], mesh [scale]), "
                     "pose, mesh_sha256|null}]}; numbers as strings, '-0.0..' without sign; "
                     "json sort_keys, separators (',',':'); 'sha256:'+hex",
    })
    return jsonify(info), 200


@bp.route("/scene/batch", methods=["POST"])
def scene_batch():
    """Atomic scene update for one namespace: validated completely first
    (any error -> 4xx, scene unchanged), then committed as ONE
    PlanningScene diff, confirmed by reading the scene back; on failure the
    previous state is re-applied (or the scene is marked dirty -> 504).
    Body: {namespace, mode: replace|upsert|remove, objects:[{id, frame_id,
    pose:{position:[3], orientation:[qx,qy,qz,qw]}, shapes:[{type:
    box(size)|sphere(radius)|cylinder(radius,length)|mesh(file,scale,sha256?),
    pose}], meta}], ids (remove), if_version, validate_only, keep_attached,
    meta}. Identical content -> unchanged:true, version unchanged.
    Never moves the robot."""
    body = _json_body()
    req = _core.validate(body)                      # meshes parsed here, outside the lock
    result = _core.batch(req, _lock)
    return jsonify(result), 200


@bp.route("/scene/namespaces/<ns>", methods=["DELETE"])
def scene_delete_namespace(ns):
    """Remove every object in a namespace (and its sub-namespaces), batch and
    legacy alike, in one confirmed diff. if_version=, force_detach=1 to also
    drop attached objects. builtin/manual and empty namespaces are refused.
    A successful delete clears the dirty flag."""
    if_version = request.args.get("if_version")
    try:
        if_version = None if if_version is None else int(if_version)
    except ValueError:
        raise SceneError(400, "bad_request", "if_version must be an integer")
    force = request.args.get("force_detach", "0") not in ("0", "false", "")
    result = _core.delete_namespace(ns, _lock, force_detach=force, if_version=if_version)
    return jsonify(result), 200


@bp.route("/scene/namespaces/", methods=["DELETE"])
def scene_delete_namespace_empty():
    raise SceneError(400, "bad_request", "namespace must not be empty")


def _int_arg(name):
    v = request.args.get(name)
    if v is None:
        return None
    try:
        return int(v)
    except ValueError:
        raise SceneError(400, "bad_request", f"{name} must be an integer")


@bp.route("/scene/acm", methods=["GET"])
def scene_acm_get():
    """Allowed-collision matrix: allowed pairs (names=a,b,... to filter to
    pairs touching those names), which of them API clients wrote, and the
    default entries."""
    names = [n for n in request.args.get("names", "").split(",") if n]
    with _core._lock:
        return jsonify(_core.acm_get(names or None)), 200


@bp.route("/scene/acm", methods=["POST"])
def scene_acm_post():
    """Write ACM entries: {entries:[{a, b, allowed}], if_version}. a/b are
    robot links or scene objects. The scene's ACM is read, merged and
    written back whole (a diff with an ACM replaces it) in one confirmed
    diff; the original value of every pair is remembered so DELETE can
    restore it. Entries of removed objects are dropped automatically."""
    d = _json_body()
    if not isinstance(d, dict):
        raise SceneError(400, "bad_request", "body must be a JSON object")
    return jsonify(_core.acm_set(d.get("entries"), _lock, if_version=d.get("if_version"))), 200


@bp.route("/scene/acm", methods=["DELETE"])
def scene_acm_delete():
    """Restore the original value of client-written ACM entries: all of them,
    or (namespace=<ns>) only those touching objects of that namespace.
    SRDF/default entries are never touched."""
    ns = request.args.get("namespace") or None
    return jsonify(_core.acm_delete(_lock, namespace=ns, if_version=_int_arg("if_version"))), 200


@bp.route("/scene/padding", methods=["GET"])
def scene_padding_get():
    """Robot link padding: the configured {default_m, links} and the values
    move_group currently uses."""
    return jsonify({"padding": _core.padding, "applied": _core.io.get_padding(),
                    "scene_version": _core.version}), 200


@bp.route("/scene/padding", methods=["POST"])
def scene_padding_post():
    """Set robot link padding (robot vs world only; self-collision is not
    padded): {default_m: 0.01, links: {"panda_leftfinger": 0.003, ...},
    if_version}. 0..0.1 m. Persisted and re-applied after a restart.
    Objects are NOT padded here -- inflate them in the geometry you push."""
    d = _json_body()
    if not isinstance(d, dict) or "default_m" not in d:
        raise SceneError(400, "bad_request", "body needs default_m")
    return jsonify(_core.padding_set(d["default_m"], d.get("links"), _lock,
                                     if_version=d.get("if_version"))), 200


@bp.route("/scene/unreferenced", methods=["GET"])
def scene_unreferenced():
    """Files under prefix= (default pipefit/meshes) in the /files store that no
    scene object or model references, with size and age; files used or
    modified within older_than_s= (default 600) are marked protected."""
    try:
        older = float(request.args.get("older_than_s", "600"))
    except ValueError:
        raise SceneError(400, "bad_request", "older_than_s must be a number")
    files = _core.unreferenced(request.args.get("prefix", "pipefit/meshes"), older)
    return jsonify({"files": files, "count": len(files), "bytes": sum(f["size"] for f in files)}), 200


@bp.route("/scene/cleanup", methods=["POST"])
def scene_cleanup():
    """Delete unreferenced mesh files (and optionally unused legacy models).
    JSON: prefix (pipefit/meshes), older_than_s (600: recently uploaded or
    used files are kept), dry_run (TRUE by default -- set false to delete),
    models (false). Never touches referenced files or the scene itself."""
    d = _json_body() or {}
    if not isinstance(d, dict):
        raise SceneError(400, "bad_request", "body must be a JSON object")
    try:
        older = float(d.get("older_than_s", 600))
    except (TypeError, ValueError):
        raise SceneError(400, "bad_request", "older_than_s must be a number")
    return jsonify(_core.cleanup(d.get("prefix", "pipefit/meshes"), older,
                                 dry_run=d.get("dry_run", True) is not False,
                                 models=bool(d.get("models", False)))), 200


@bp.route("/scene/snapshot", methods=["GET"])
def scene_snapshot():
    """Canonical, complete scene for reproduction: every object with full
    geometry and meta, client ACM entries, padding, versions and hashes
    (namespace=<ns> to restrict, adds namespace_hash)."""
    ns = request.args.get("namespace") or None
    if ns:
        _core.check_namespace(ns)
    with _core._lock:
        return jsonify(_core.snapshot(ns)), 200


# ---- test hooks (only with FRANKA_TEST_HOOKS=1; never in production) ----

if os.environ.get("FRANKA_TEST_HOOKS") == "1":
    @bp.route("/scene/_test/fail_next_apply", methods=["POST"])
    def _test_fail_next_apply():
        n = int(request.args.get("n", "1"))
        _core.test_hooks["fail_next_apply"] = n
        return jsonify({"fail_next_apply": n}), 200

    @bp.route("/scene/_test/delay_next_apply", methods=["POST"])
    def _test_delay_next_apply():
        s = float(request.args.get("s", "3"))
        _core.test_hooks["delay_next_apply_s"] = s
        return jsonify({"delay_next_apply_s": s}), 200
