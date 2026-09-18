"""/files -- HTTP file store so CAD models (and anything else) can be pushed
to the robot server by other machines (the object detector, a laptop) and
then referenced by path from /scene/models.

    PUT    /files/<path>            upload / overwrite (raw body, or multipart field "file")
    POST   /files/<path>            same as PUT (for HTML forms); ?mkdir=1 creates a directory
    GET    /files/<path>            download a file, or list a directory (JSON)
    HEAD   /files/<path>            metadata only (Content-Length, ETag = sha256; served by GET)
    DELETE /files/<path>            delete a file (or directory with ?recursive=1)
    GET    /files?q=*.stl           search by glob (recursive=1 default)

Root: $FRANKA_FILES_ROOT (default <repo>/data). Paths are sandboxed to it.
"""
import os

from flask import Blueprint, jsonify, request, send_file

from file_store import FileStore, FileStoreError

_DEFAULT_ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")

store = FileStore(
    os.environ.get("FRANKA_FILES_ROOT", _DEFAULT_ROOT),
    max_file_bytes=int(os.environ.get("FRANKA_FILES_MAX_BYTES", 100 * 1024 * 1024)),
    quota_bytes=int(os.environ.get("FRANKA_FILES_QUOTA_BYTES", 2 * 1024 ** 3)),
)

bp = Blueprint("files", __name__)


def _flag(name, default=False):
    v = request.args.get(name)
    if v is None:
        return default
    return v not in ("0", "false", "no", "")


@bp.errorhandler(FileStoreError)
def _on_store_error(e):
    return jsonify({"error": e.code, "msg": e.msg}), e.status


def _upload(rel):
    abs_path = store.resolve(rel)
    overwrite = _flag("overwrite", True)
    if request.files and "file" in request.files:
        f = request.files["file"]
        info = store.write_stream(abs_path, f.stream, overwrite=overwrite)
    else:
        info = store.write_stream(abs_path, request.stream,
                                  declared_length=request.content_length, overwrite=overwrite)
    return jsonify({"status": "stored", **info}), 201


@bp.route("/files", methods=["GET"], strict_slashes=False)
def files_root():
    """List the file-store root, or search it: ?q=<glob> (e.g. *.stl),
    recursive=1, hash=1 to include sha256 per file. Also reports usage/quota."""
    q = request.args.get("q")
    body = {
        "root": "",
        "usage_bytes": store.usage_bytes(),
        "quota_bytes": store.quota_bytes,
        "max_file_bytes": store.max_file_bytes,
    }
    if q:
        body["query"] = q
        body["entries"] = store.search(q, recursive=_flag("recursive", True), with_hash=_flag("hash"))
    else:
        body["entries"] = store.listdir(store.root, with_hash=_flag("hash"))
    return jsonify(body), 200


@bp.route("/files/<path:rel>", methods=["GET"])
def files_get(rel):
    """Download the file at <path> (as an attachment), or list it if it is a
    directory (?hash=1 adds sha256 per file)."""
    abs_path = store.resolve(rel)
    if os.path.isdir(abs_path):
        return jsonify({"root": store.relpath(abs_path),
                        "entries": store.listdir(abs_path, with_hash=_flag("hash"))}), 200
    if not os.path.isfile(abs_path):
        raise FileStoreError(404, "not_found", "no such file")
    resp = send_file(abs_path, as_attachment=_flag("download", True),
                     download_name=os.path.basename(abs_path), conditional=True)
    resp.headers["ETag"] = store.sha256(abs_path)
    resp.headers["X-File-Type"] = "file"
    return resp


@bp.route("/files/<path:rel>", methods=["PUT"])
def files_put(rel):
    """Upload (create/overwrite) the file at <path>. Body is the raw file
    (curl -T model.stl http://host:5000/files/cad/model.stl) or a multipart
    form with field "file". overwrite=0 refuses to replace an existing file.
    Atomic: readers never see a partial file. 413 over the per-file limit,
    507 over the store quota."""
    return _upload(rel)


@bp.route("/files/<path:rel>", methods=["POST"])
def files_post(rel):
    """Same as PUT (for HTML forms / clients that cannot PUT); with ?mkdir=1
    creates the directory <path> instead."""
    if _flag("mkdir"):
        return jsonify({"status": "created", **store.mkdir(store.resolve(rel))}), 201
    return _upload(rel)


@bp.route("/files/<path:rel>", methods=["DELETE"])
def files_delete(rel):
    """Delete the file at <path>; a directory needs ?recursive=1 unless empty."""
    store.delete(store.resolve(rel), recursive=_flag("recursive"))
    return jsonify({"status": "deleted", "path": rel.strip("/")}), 200
