"""Sandboxed file store backing the /files API.

Every path the API accepts is resolved with :meth:`FileStore.resolve`, which
rejects anything that escapes ``root`` (``..``, absolute paths, symlinks
pointing outside) or uses characters outside ``[A-Za-z0-9._-]`` per
component. Writes go to a temp file in the same directory and are renamed
into place, so a reader (e.g. /scene/models loading a mesh) never sees a
half-written file. sha256 digests are cached per (size, mtime) so listing a
directory of large CAD files stays cheap after the first pass.
"""
import fnmatch
import hashlib
import os
import re
import shutil
import tempfile
import time

_COMPONENT_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


class FileStoreError(Exception):
    """Raised for client errors; ``status`` is the HTTP status to return."""

    def __init__(self, status, code, msg):
        super().__init__(msg)
        self.status = status
        self.code = code
        self.msg = msg


class FileStore:
    def __init__(self, root, max_file_bytes=100 * 1024 * 1024, quota_bytes=2 * 1024 ** 3):
        self.root = os.path.realpath(root)
        os.makedirs(self.root, exist_ok=True)
        self.max_file_bytes = int(max_file_bytes)
        self.quota_bytes = int(quota_bytes)
        self._sha_cache = {}   # abs path -> (size, mtime_ns, sha256)

    # ---- paths -----------------------------------------------------------

    def resolve(self, rel):
        """Absolute path for a client-supplied relative path, or raise 400.
        '' / '/' means the root."""
        rel = (rel or "").strip().strip("/")
        if rel == "":
            return self.root
        parts = rel.split("/")
        for p in parts:
            if not _COMPONENT_RE.match(p) or p in (".", ".."):
                raise FileStoreError(400, "bad_path",
                                     "path components must match [A-Za-z0-9][A-Za-z0-9._-]* (no '..', no hidden files)")
        abs_path = os.path.realpath(os.path.join(self.root, *parts))
        if abs_path != self.root and not abs_path.startswith(self.root + os.sep):
            raise FileStoreError(400, "bad_path", "path escapes the file store root")
        return abs_path

    def relpath(self, abs_path):
        return os.path.relpath(abs_path, self.root).replace(os.sep, "/")

    # ---- metadata --------------------------------------------------------

    def sha256(self, abs_path):
        st = os.stat(abs_path)
        key = (st.st_size, st.st_mtime_ns)
        cached = self._sha_cache.get(abs_path)
        if cached and cached[:2] == key:
            return cached[2]
        h = hashlib.sha256()
        with open(abs_path, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        digest = h.hexdigest()
        self._sha_cache[abs_path] = (st.st_size, st.st_mtime_ns, digest)
        return digest

    def stat(self, abs_path, with_hash=True):
        st = os.stat(abs_path)
        is_dir = os.path.isdir(abs_path)
        info = {
            "path": self.relpath(abs_path) if abs_path != self.root else "",
            "name": os.path.basename(abs_path) if abs_path != self.root else "",
            "type": "dir" if is_dir else "file",
            "size": None if is_dir else st.st_size,
            "mtime": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(st.st_mtime)),
            "mtime_epoch": st.st_mtime,
        }
        if not is_dir and with_hash:
            info["sha256"] = self.sha256(abs_path)
        return info

    def listdir(self, abs_path, with_hash=False):
        if not os.path.isdir(abs_path):
            raise FileStoreError(404, "not_found", "no such directory")
        entries = []
        for name in sorted(os.listdir(abs_path)):
            if name.startswith("."):
                continue
            entries.append(self.stat(os.path.join(abs_path, name), with_hash=with_hash))
        return entries

    def search(self, pattern="*", recursive=True, with_hash=False):
        out = []
        for dirpath, dirnames, filenames in os.walk(self.root):
            dirnames[:] = sorted(d for d in dirnames if not d.startswith("."))
            for name in sorted(filenames):
                if name.startswith(".") or not fnmatch.fnmatch(name, pattern):
                    continue
                out.append(self.stat(os.path.join(dirpath, name), with_hash=with_hash))
            if not recursive:
                break
        return out

    def usage_bytes(self):
        total = 0
        for dirpath, _dirs, files in os.walk(self.root):
            for name in files:
                try:
                    total += os.path.getsize(os.path.join(dirpath, name))
                except OSError:
                    pass
        return total

    # ---- mutations -------------------------------------------------------

    def mkdir(self, abs_path):
        if abs_path == self.root:
            raise FileStoreError(400, "bad_path", "root already exists")
        if os.path.isfile(abs_path):
            raise FileStoreError(409, "conflict", "a file with that name exists")
        os.makedirs(abs_path, exist_ok=True)
        return self.stat(abs_path)

    def write_stream(self, abs_path, stream, declared_length=None, overwrite=True):
        """Stream ``stream`` (file-like with .read) into ``abs_path``
        atomically. Enforces the per-file limit and the quota."""
        if abs_path == self.root or os.path.isdir(abs_path):
            raise FileStoreError(400, "bad_path", "target is a directory")
        if os.path.exists(abs_path) and not overwrite:
            raise FileStoreError(409, "conflict", "file exists (pass overwrite=1)")
        if declared_length is not None and declared_length > self.max_file_bytes:
            raise FileStoreError(413, "too_large", f"file exceeds the {self.max_file_bytes} byte limit")
        existing = os.path.getsize(abs_path) if os.path.isfile(abs_path) else 0
        budget = self.quota_bytes - (self.usage_bytes() - existing)
        parent = os.path.dirname(abs_path)
        os.makedirs(parent, exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix=".upload-", dir=parent)
        written = 0
        h = hashlib.sha256()
        try:
            with os.fdopen(fd, "wb") as out:
                while True:
                    chunk = stream.read(1 << 20)
                    if not chunk:
                        break
                    written += len(chunk)
                    if written > self.max_file_bytes:
                        raise FileStoreError(413, "too_large", f"file exceeds the {self.max_file_bytes} byte limit")
                    if written > budget:
                        raise FileStoreError(507, "quota_exceeded", f"store quota of {self.quota_bytes} bytes exceeded")
                    out.write(chunk)
                    h.update(chunk)
            os.replace(tmp, abs_path)
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
        st = os.stat(abs_path)
        self._sha_cache[abs_path] = (st.st_size, st.st_mtime_ns, h.hexdigest())
        return self.stat(abs_path)

    def delete(self, abs_path, recursive=False):
        if abs_path == self.root:
            raise FileStoreError(400, "bad_path", "refusing to delete the root")
        if os.path.isdir(abs_path):
            if os.listdir(abs_path) and not recursive:
                raise FileStoreError(409, "not_empty", "directory not empty (pass recursive=1)")
            shutil.rmtree(abs_path)
        elif os.path.isfile(abs_path):
            os.unlink(abs_path)
            self._sha_cache.pop(abs_path, None)
        else:
            raise FileStoreError(404, "not_found", "no such file")
