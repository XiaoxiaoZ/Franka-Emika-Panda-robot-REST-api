"""Parse a mesh file with pyassimp in a throwaway process.

pyassimp (the C library under it) can segfault on malformed input, and the
REST server must survive arbitrary uploaded CAD files, so scene_manager
runs this as a subprocess:

    python3 mesh_parse.py <mesh file> <out.npz>

Writes vertices (N x 3 float64) and faces (M x 3 int64) with every node's
transform applied (all meshes in the file merged). Exit code 0 on success,
2 with a one-line reason on stderr for a readable-but-unusable file.
"""
import sys

import numpy as np


def _walk(node, parent_tf, out):
    tf = parent_tf @ np.asarray(node.transformation, dtype=float).reshape(4, 4)
    for m in node.meshes:
        verts = np.asarray(m.vertices, dtype=float)
        if verts.size == 0 or len(m.faces) == 0:
            continue
        faces = [f for f in np.asarray(m.faces) if len(f) == 3] if hasattr(m.faces[0], "__len__") \
            else [f.indices for f in m.faces if len(f.indices) == 3]
        if not faces:
            continue
        hom = np.hstack([verts, np.ones((len(verts), 1))])
        out.append(((hom @ tf.T)[:, :3], np.asarray(faces, dtype=np.int64)))
    for child in node.children:
        _walk(child, tf, out)


def main(path, out_path):
    import pyassimp
    from pyassimp.errors import AssimpError
    try:
        scene = pyassimp.load(path)
    except (Exception, AssimpError) as e:   # AssimpError derives from BaseException!
        print(f"not a readable mesh: {e}", file=sys.stderr)
        return 2
    try:
        parts = []
        _walk(scene.rootnode, np.eye(4), parts)
    finally:
        pyassimp.release(scene)
    if not parts:
        print("no triangles found in the file", file=sys.stderr)
        return 2
    verts, faces, offset = [], [], 0
    for v, f in parts:
        verts.append(v)
        faces.append(f + offset)
        offset += len(v)
    np.savez(out_path, vertices=np.vstack(verts), faces=np.vstack(faces))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
