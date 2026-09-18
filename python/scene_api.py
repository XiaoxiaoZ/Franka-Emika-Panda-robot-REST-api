"""/scene -- CAD models and objects in the MoveIt planning scene.

    GET    /scene/models                  registered models
    POST   /scene/models                  {name, file, scale=1, collision=mesh|convex_hull, overwrite=0}
    GET    /scene/models/<name>
    DELETE /scene/models/<name>?force=1   (force also removes objects using it)

    GET    /scene/objects                 everything in the scene (ours + built-ins)
    POST   /scene/objects                 {name, model, x,y,z, roll,pitch,yaw | qx,qy,qz,qw, frame, source}
    PUT    /scene/objects/sync            {source, objects:[{name, model, x,y,z,...}]} -- replace that source's set
    GET    /scene/objects/<name>
    PATCH  /scene/objects/<name>          {x,y,z, roll,pitch,yaw | qx..qw, frame}  (partial)
    DELETE /scene/objects/<name>?force=1  (force needed for built-ins)
    POST   /scene/objects/<name>/attach   {link=panda_hand, touch_links=[...]}
    POST   /scene/objects/<name>/detach

Bodies are JSON; form fields / query args work too for simple calls.
Mutations take the robot motion lock (wait up to 5 s, else 409 busy) so
the world never changes under a running plan.
"""
from flask import Blueprint, jsonify, request

from scene_manager import SceneError

bp = Blueprint("scene", __name__)
_manager = None
_lock = None


def init(manager, lock):
    global _manager, _lock
    _manager = manager
    _lock = lock


def _body():
    data = request.get_json(silent=True)
    if data is None:
        data = request.values.to_dict()
    if not isinstance(data, dict):
        raise SceneError(400, "bad_request", "JSON body must be an object")
    return data


def _flag(data, key, default=False):
    v = data.get(key, request.args.get(key))
    if v is None:
        return default
    if isinstance(v, bool):
        return v
    return str(v) not in ("0", "false", "no", "")


class _Locked:
    def __enter__(self):
        if _lock is not None and not _lock.acquire(timeout=5.0):
            raise SceneError(409, "busy", "Robot is busy; scene change refused (retry shortly)")
        return self

    def __exit__(self, *exc):
        if _lock is not None:
            _lock.release()


@bp.errorhandler(SceneError)
def _on_scene_error(e):
    return jsonify({"error": e.code, "msg": e.msg, **e.extra}), e.status


# ---- models ----

@bp.route("/scene/models", methods=["GET"])
def models_list():
    """Registered CAD models with triangle counts, bounding boxes and which
    objects use them."""
    return jsonify({"models": _manager.list_models()}), 200


@bp.route("/scene/models", methods=["POST"])
def models_register():
    """Register a mesh from the /files store as a named model. JSON: name,
    file (path in /files, e.g. cad/part.stl), scale=1 (0.001 for mm STLs),
    collision=mesh|convex_hull, overwrite=0. Re-registering with overwrite=1
    re-parses the file and refreshes every object placed with the model."""
    d = _body()
    if not d.get("name") or not d.get("file"):
        raise SceneError(400, "bad_request", "name and file are required")
    with _Locked():
        info = _manager.register_model(d["name"], d["file"], d.get("scale", 1.0),
                                       d.get("collision", "mesh"), overwrite=_flag(d, "overwrite"))
    return jsonify({"status": "registered", **info}), 201


@bp.route("/scene/models/<name>", methods=["GET"])
def models_get(name):
    return jsonify(_manager.get_model(name)), 200


@bp.route("/scene/models/<name>", methods=["DELETE"])
def models_delete(name):
    """Delete a model; 409 if objects use it unless force=1 (removes them too)."""
    with _Locked():
        _manager.delete_model(name, force=_flag({}, "force"))
    return jsonify({"status": "deleted", "name": name}), 200


# ---- objects ----

@bp.route("/scene/objects", methods=["GET"])
def objects_list():
    """Everything in the planning scene: our model instances (with model,
    pose, source, attached_link) plus built-ins (walls, ceiling, floor)."""
    return jsonify({"objects": _manager.list_objects()}), 200


@bp.route("/scene/objects", methods=["POST"])
def objects_add():
    """Place a model in the scene. JSON: name, model, x,y,z (m) and either
    roll,pitch,yaw (deg, base frame) or qx,qy,qz,qw; frame=panda_link0,
    source=manual. Same name replaces the existing object."""
    d = _body()
    if not d.get("name") or not d.get("model"):
        raise SceneError(400, "bad_request", "name and model are required")
    with _Locked():
        info = _manager.add_object(d["name"], d["model"], d, frame=d.get("frame"),
                                   source=d.get("source", "manual"))
    return jsonify({"status": "added", **info}), 201


@bp.route("/scene/objects/sync", methods=["PUT", "POST"])
def objects_sync():
    """Replace the whole set of objects belonging to one source in one call
    (for the detector: call after every detection run). JSON: source,
    objects:[{name, model, x,y,z, roll,pitch,yaw | qx..qw, frame}]. Objects
    of that source not in the list are removed; other sources, built-ins
    and attached objects are untouched. 404 unknown_model lists models that
    still need POST /scene/models."""
    d = _body()
    objs = d.get("objects")
    if not isinstance(objs, list):
        raise SceneError(400, "bad_request", "objects must be a list")
    with _Locked():
        result = _manager.sync(d.get("source"), objs)
    return jsonify({"status": "synced", "source": d.get("source"), **result}), 200


@bp.route("/scene/objects/<name>", methods=["GET"])
def objects_get(name):
    return jsonify(_manager.get_object(name)), 200


@bp.route("/scene/objects/<name>", methods=["PATCH", "PUT"])
def objects_update(name):
    """Move an object: any of x,y,z, roll,pitch,yaw | qx..qw, frame (partial
    update; unspecified fields keep their value)."""
    d = _body()
    with _Locked():
        info = _manager.update_object(name, d, frame=d.get("frame"))
    return jsonify({"status": "updated", **info}), 200


@bp.route("/scene/objects/<name>", methods=["DELETE"])
def objects_delete(name):
    """Remove an object from the scene (force=1 for built-in safety geometry)."""
    with _Locked():
        _manager.remove_object(name, force=_flag({}, "force"))
    return jsonify({"status": "removed", "name": name}), 200


@bp.route("/scene/objects/<name>/attach", methods=["POST"])
def objects_attach(name):
    """Attach the object to a robot link (default panda_hand) so it moves with
    the gripper and is collision-checked as part of it. JSON: link,
    touch_links (links allowed to touch it; default hand + fingers)."""
    d = _body()
    with _Locked():
        info = _manager.attach(name, link=d.get("link", "panda_hand"), touch_links=d.get("touch_links"))
    return jsonify({"status": "attached", **info}), 200


@bp.route("/scene/objects/<name>/detach", methods=["POST"])
def objects_detach(name):
    """Detach the object from the robot; it stays in the world where it is."""
    with _Locked():
        info = _manager.detach(name)
    return jsonify({"status": "detached", **info}), 200
