"""Intel RealSense D435 access for the REST API.

Grabs color + depth (aligned to color) on a background thread and hands the
latest frames to the Flask endpoints in restful.py, so network clients can
fetch images, depth values, and 3D points with plain HTTP GETs.

Design notes
------------
* LAZY START: constructing RealsenseCamera does not touch the device. The
  pipeline opens on the first camera request (or GET /camera/start). This
  keeps the module import cheap and avoids the Flask debug reloader's
  supervisor process claiming the USB device.
* The grab thread owns the pipeline; endpoints only read the latest frames
  under a lock (same pattern as ForceVisualizer).
* Depth frames are ALIGNED TO COLOR, so pixel (u, v) indexes the same point
  in both images, and deprojection uses the color intrinsics.
"""

import glob
import sys
import threading
import time

import numpy as np


def _import_rs():
    """Import pyrealsense2, falling back to user-level site-packages.
    The wheel is installed with `pip install --user` under the franka user,
    but the server usually runs as root (sudo) whose Python doesn't search
    user sites -- so extend sys.path on demand."""
    try:
        import pyrealsense2 as rs
        return rs
    except ModuleNotFoundError:
        for p in glob.glob("/home/*/.local/lib/python3*/site-packages"):
            if p not in sys.path:
                sys.path.append(p)
        import pyrealsense2 as rs
        return rs


class RealsenseCamera:
    def __init__(self, width=848, height=480, fps=15):
        self.width, self.height, self.fps = width, height, fps
        self._lock = threading.Lock()
        self._frames = None          # {"color": bgr u8, "depth": raw u16, "ts": float}
        self._pipeline = None
        self._align = None
        self._depth_scale = None     # meters per depth unit (D435: 0.001)
        self._intrinsics = None      # color-stream intrinsics (fx, fy, ppx, ppy, ...)
        self._serial = None
        self._depth_res = None   # actual depth-sensor stream resolution
        self._running = False
        self._seq = 0            # frame counter (id for consistency checks)
        self._last_error = None

    # ------------------------------------------------------------------ #
    # lifecycle
    # ------------------------------------------------------------------ #
    def start(self):
        """Open the device and start grabbing. Idempotent.
        The depth stream tries the color resolution first, then falls back to
        common D435 depth modes (align upscales depth to color size anyway).
        Returns (ok, error_message)."""
        with self._lock:
            if self._running:
                return True, None
        pipeline = None
        try:
            rs = _import_rs()
            pipeline = rs.pipeline()
            depth_candidates = [(self.width, self.height),
                                (848, 480), (640, 480), (1280, 720)]
            profile = None
            last_exc = None
            for dw, dh in dict.fromkeys(depth_candidates):
                cfg = rs.config()
                cfg.enable_stream(rs.stream.color, self.width, self.height,
                                  rs.format.bgr8, self.fps)
                cfg.enable_stream(rs.stream.depth, dw, dh,
                                  rs.format.z16, self.fps)
                try:
                    profile = pipeline.start(cfg)
                    self._depth_res = (dw, dh)
                    break
                except Exception as e:
                    last_exc = e
            if profile is None:
                raise last_exc or RuntimeError("no supported stream combination")
            dev = profile.get_device()
            self._serial = dev.get_info(rs.camera_info.serial_number)
            self._depth_scale = float(dev.first_depth_sensor().get_depth_scale())
            self._intrinsics = profile.get_stream(
                rs.stream.color).as_video_stream_profile().get_intrinsics()
            self._align = rs.align(rs.stream.color)
            self._pipeline = pipeline
            self._running = True
            self._last_error = None
            t = threading.Thread(target=self._loop, daemon=True,
                                 name="realsense-grab")
            t.start()
            return True, None
        except Exception as e:
            self._last_error = str(e)
            if pipeline is not None:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            return False, str(e)

    def stop(self):
        """Stop grabbing and release the USB device."""
        self._running = False
        time.sleep(0.1)  # let the grab loop leave wait_for_frames
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None
        with self._lock:
            self._frames = None

    def _loop(self):
        while self._running:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=5000)
                frames = self._align.process(frames)
                c = frames.get_color_frame()
                d = frames.get_depth_frame()
                if not c or not d:
                    continue
                color = np.asanyarray(c.get_data()).copy()
                depth = np.asanyarray(d.get_data()).copy()
                with self._lock:
                    self._seq += 1
                    self._frames = {"color": color, "depth": depth,
                                    "ts": time.time(), "id": self._seq}
            except Exception as e:
                if self._running:
                    self._last_error = str(e)
                    time.sleep(0.2)

    def ensure_started(self):
        """(ok, err) -- start if needed and wait briefly for a first frame."""
        ok, err = self.start()
        if not ok:
            return False, err
        for _ in range(40):          # up to ~2 s for the first frame
            if self.latest() is not None:
                return True, None
            time.sleep(0.05)
        return False, self._last_error or "no frames from camera yet"

    # ------------------------------------------------------------------ #
    # data access (lock-protected snapshots)
    # ------------------------------------------------------------------ #
    def latest(self):
        with self._lock:
            return self._frames

    # ------------------------------------------------------------------ #
    # configuration
    # ------------------------------------------------------------------ #
    def supported_modes(self):
        """Common D435 (width, height, fps) modes for the color and depth
        streams. STATIC table on purpose: live device enumeration via
        rs.context() can segfault librealsense when the USB device is in a
        stale state (e.g. right after an unclean worker restart), taking the
        whole server down. set_config() validates the requested mode against
        the real device anyway (with rollback), so this list is guidance."""
        return {
            "color": [[424, 240, f] for f in (6, 15, 30, 60)] +
                     [[640, 360, f] for f in (6, 15, 30, 60)] +
                     [[640, 480, f] for f in (6, 15, 30, 60)] +
                     [[848, 480, f] for f in (6, 15, 30, 60)] +
                     [[1280, 720, f] for f in (6, 15, 30)] +
                     [[1920, 1080, f] for f in (6, 15, 30)],
            "depth": [[424, 240, f] for f in (6, 15, 30, 60, 90)] +
                     [[480, 270, f] for f in (6, 15, 30, 60, 90)] +
                     [[640, 360, f] for f in (6, 15, 30, 60, 90)] +
                     [[640, 480, f] for f in (6, 15, 30, 60, 90)] +
                     [[848, 480, f] for f in (6, 15, 30, 60, 90)] +
                     [[1280, 720, f] for f in (6, 15, 30)],
            "note": "curated common D435 modes; the actual gate is the device "
                    "itself -- /camera/config validates and rolls back on failure",
        }

    def set_config(self, width=None, height=None, fps=None):
        """Apply a new color-stream configuration, restarting the stream to
        validate it against the real device. On failure the previous config is
        restored (and restarted if it was running). Returns (ok, err)."""
        old = (self.width, self.height, self.fps)
        was_running = self._running
        new = (int(width) if width else old[0],
               int(height) if height else old[1],
               int(fps) if fps else old[2])
        if new == old and was_running:
            return True, None
        if was_running:
            self.stop()
        self.width, self.height, self.fps = new
        ok, err = self.ensure_started()
        if not ok:
            # roll back so a bad request can't leave the camera dead
            self.width, self.height, self.fps = old
            if was_running:
                self.ensure_started()
            return False, err
        return True, None

    def info(self):
        """Status + intrinsics dict for /camera/info."""
        fr = self.latest()
        out = {
            "streaming": self._running,
            "has_frame": fr is not None,
            "width": self.width, "height": self.height, "fps": self.fps,
            "serial": self._serial,
            "depth_sensor_res": self._depth_res,  # aligned output is color-sized
            "depth_scale_m": self._depth_scale,
            "last_error": self._last_error,
        }
        if fr is not None:
            out["frame_age_s"] = round(time.time() - fr["ts"], 3)
        i = self._intrinsics
        if i is not None:
            out["intrinsics"] = {
                "fx": i.fx, "fy": i.fy, "ppx": i.ppx, "ppy": i.ppy,
                "model": str(i.model), "coeffs": list(i.coeffs),
                "note": "color-stream intrinsics; depth is aligned to color, "
                        "so (u,v) indexes both images",
            }
        return out

    def depth_at(self, u, v, win=5):
        """Median depth (meters) in a win x win window around pixel (u, v) of
        the aligned depth image, ignoring zero (no-data) pixels. Also returns
        the deprojected 3D point in the CAMERA frame (meters, +z forward).
        Returns dict or None if no frame / no valid depth."""
        fr = self.latest()
        if fr is None or self._depth_scale is None:
            return None
        u, v = int(u), int(v)
        h, w = fr["depth"].shape
        if not (0 <= u < w and 0 <= v < h):
            return {"error": f"pixel out of range (image is {w}x{h})"}
        r = max(1, int(win) // 2)
        patch = fr["depth"][max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
        valid = patch[patch > 0]
        if valid.size == 0:
            return {"u": u, "v": v, "depth_m": None,
                    "msg": "no valid depth at this pixel (hole/too close/too far)"}
        depth_m = float(np.median(valid)) * self._depth_scale
        out = {"u": u, "v": v, "depth_m": round(depth_m, 4),
               "valid_samples": int(valid.size)}
        if self._intrinsics is not None:
            rs = _import_rs()
            x, y, z = rs.rs2_deproject_pixel_to_point(
                self._intrinsics, [float(u), float(v)], depth_m)
            out["point_camera_m"] = {"x": round(x, 4), "y": round(y, 4),
                                     "z": round(z, 4)}
        return out

    def frame_bundle(self):
        """One CONSISTENT snapshot: color + depth (converted to uint16 mm)
        taken from the same frameset, plus frame id/timestamp and intrinsics.
        Guarantees pixel-level color/depth pairing (single latest() read)."""
        fr = self.latest()
        if fr is None or self._depth_scale is None:
            return None
        mm = np.clip(fr["depth"].astype(np.float32) * (self._depth_scale * 1000.0),
                     0, 65535).astype(np.uint16)
        i = self._intrinsics
        return {"color": fr["color"], "depth_mm": mm,
                "ts": fr["ts"], "id": fr.get("id", 0),
                "fx": i.fx if i else 0.0, "fy": i.fy if i else 0.0,
                "ppx": i.ppx if i else 0.0, "ppy": i.ppy if i else 0.0,
                "depth_scale_m": self._depth_scale}

    def depth_mm(self):
        """Aligned depth converted to uint16 MILLIMETERS (0 = no data)."""
        fr = self.latest()
        if fr is None or self._depth_scale is None:
            return None
        mm = fr["depth"].astype(np.float32) * (self._depth_scale * 1000.0)
        return np.clip(mm, 0, 65535).astype(np.uint16)

    def depth_preview_bgr(self, max_m=4.0):
        """Colorized depth (JET colormap, 0..max_m meters) for human viewing."""
        import cv2
        fr = self.latest()
        if fr is None or self._depth_scale is None:
            return None
        d_m = fr["depth"].astype(np.float32) * self._depth_scale
        norm = np.clip(d_m / max_m, 0, 1)
        img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
        img[fr["depth"] == 0] = 0  # no-data -> black
        return img
