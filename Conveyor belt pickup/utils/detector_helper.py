# detection_helper.py
from __future__ import annotations

from typing import List, Dict, Optional, Tuple
import threading
import time

# Hailo SDK
import hailo

# GStreamer / PyGObject
import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst

# Hailo app pieces (same ones you use in run_hailo_with_display.py)
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import GStreamerDetectionApp

# Buffer helpers (caps + frame extraction)
from hailo_apps.hailo_app_python.core.common.buffer_utils import get_caps_from_pad


# ======================================================================================
# Core parser: returns per-detection (label, u, v) in pixel coords
# ======================================================================================
from typing import Optional, List, Dict
import hailo  # assuming already available

class DetectionHelper:
    """
    Robust parser that returns per-detection dicts:
      { 'label': str, 'u': int, 'v': int, 'track_id': int, 'score': float, 'area': Optional[float] }
    - Handles normalized (0..1) or absolute boxes.
    - Tries multiple attribute/method names to stay compatible across SDK versions.
    """
    def __init__(self, require_label: Optional[str] = None):
        self.require_label = require_label  # e.g., "apple" to filter; None = all

    def parse(self, pad, buffer) -> List[Dict]:
        fmt, width, height = self._safe_caps(pad)
        if width is None or height is None:
            return []

        try:
            roi = hailo.get_roi_from_buffer(buffer)
            dets = roi.get_objects_typed(hailo.HAILO_DETECTION)
        except Exception:
            return []

        out: List[Dict] = []
        for det in dets:
            label = self._safe_label(det)
            if self.require_label and label != self.require_label:
                continue

            bbox = det.get_bbox()
            cx, cy, is_norm = self._bbox_center(bbox)
            if cx is None or cy is None:
                continue

            # pixel coords
            if is_norm:
                u = int(round(cx * width))
                v = int(round(cy * height))
            else:
                u = int(round(cx))
                v = int(round(cy))

            track_id = self._safe_track_id(det)
            score = self._safe_score(det)

            # optional area (best-effort)
            area = None
            try:
                w = self._first_valid([
                    self._maybe_call(bbox, "w"),
                    self._maybe_call(bbox, "width"),
                    self._maybe_call(bbox, "get_width"),
                ])
                h = self._first_valid([
                    self._maybe_call(bbox, "h"),
                    self._maybe_call(bbox, "height"),
                    self._maybe_call(bbox, "get_height"),
                ])
                if w is not None and h is not None:
                    area = float(w) * float(h)
            except Exception:
                pass

            out.append({
                "label": label,
                "u": u,
                "v": v,
                "track_id": track_id,
                "score": float(score) if score is not None else 0.0,
                "area": area,
            })

        return out
    # ------------------ internals ------------------

    def _safe_caps(self, pad) -> Tuple[Optional[str], Optional[int], Optional[int]]:
        try:
            return get_caps_from_pad(pad)  # (format, width, height)
        except Exception:
            return None, None, None

    def _safe_label(self, det) -> str:
        try:
            return det.get_label()
        except Exception:
            try:
                return getattr(det, "label", "unknown")
            except Exception:
                return "unknown"

    def _safe_track_id(self, det) -> int:
        try:
            uniq = det.get_objects_typed(hailo.HAILO_UNIQUE_ID)
            if len(uniq) == 1:
                return int(uniq[0].get_id())
        except Exception:
            pass
        return -1

    def _bbox_center(self, bbox) -> Tuple[Optional[float], Optional[float], bool]:
        """
        Returns (cx, cy, is_normalized)
        Tries multiple attribute/method patterns:
          - xmin/xmax/ymin/ymax (normalized 0..1 or absolute)
          - x/y/width/height
          - left/top/right/bottom
        """
        try:
            # Pattern A: xmin/xmax/ymin/ymax
            xs = [self._maybe_call(bbox, "xmin"), self._maybe_call(bbox, "x_min"), self._maybe_call(bbox, "left")]
            ys = [self._maybe_call(bbox, "ymin"), self._maybe_call(bbox, "y_min"), self._maybe_call(bbox, "top")]
            xe = [self._maybe_call(bbox, "xmax"), self._maybe_call(bbox, "x_max"), self._maybe_call(bbox, "right")]
            ye = [self._maybe_call(bbox, "ymax"), self._maybe_call(bbox, "y_max"), self._maybe_call(bbox, "bottom")]

            xmin = self._first_valid(xs)
            ymin = self._first_valid(ys)
            xmax = self._first_valid(xe)
            ymax = self._first_valid(ye)

            if xmin is not None and ymin is not None and xmax is not None and ymax is not None:
                cx = (xmin + xmax) * 0.5
                cy = (ymin + ymax) * 0.5
                # Heuristic: if coords mostly within [0,1], treat as normalized
                is_norm = 0.0 <= min(xmin, ymin, xmax, ymax) <= 1.0 and 0.0 <= max(xmin, ymin, xmax, ymax) <= 1.0
                return cx, cy, is_norm

            # Pattern B: x/y/width/height
            x  = self._first_valid([self._maybe_call(bbox, "x"), self._maybe_call(bbox, "get_x"), self._maybe_call(bbox, "left")])
            y  = self._first_valid([self._maybe_call(bbox, "y"), self._maybe_call(bbox, "get_y"), self._maybe_call(bbox, "top")])
            w  = self._first_valid([self._maybe_call(bbox, "w"), self._maybe_call(bbox, "width"), self._maybe_call(bbox, "get_width")])
            h  = self._first_valid([self._maybe_call(bbox, "h"), self._maybe_call(bbox, "height"), self._maybe_call(bbox, "get_height")])

            if x is not None and y is not None and w is not None and h is not None:
                cx = x + w * 0.5
                cy = y + h * 0.5
                # Heuristic normalization check
                is_norm = (0 <= x <= 1) and (0 <= y <= 1) and (0 <= x + w <= 1.01) and (0 <= y + h <= 1.01)
                return cx, cy, is_norm

            # Pattern C: direct tuple/struct methods
            get_tl = getattr(bbox, "get_top_left", None)
            get_br = getattr(bbox, "get_bottom_right", None)
            if callable(get_tl) and callable(get_br):
                tl = get_tl(); br = get_br()
                xt, yt = getattr(tl, "x", None), getattr(tl, "y", None)
                xb, yb = getattr(br, "x", None), getattr(br, "y", None)
                if None not in (xt, yt, xb, yb):
                    cx = (xt + xb) * 0.5
                    cy = (yt + yb) * 0.5
                    is_norm = (0 <= min(xt, yt, xb, yb) <= 1) and (0 <= max(xt, yt, xb, yb) <= 1)
                    return cx, cy, is_norm
        except Exception:
            pass

        return None, None, True  # fallback

    def _maybe_call(self, obj, name):
        """Return obj.name() if callable, else obj.name, else None."""
        if not hasattr(obj, name):
            return None
        attr = getattr(obj, name)
        try:
            return attr() if callable(attr) else attr
        except Exception:
            return None

    def _first_valid(self, vals):
        for v in vals:
            if v is not None:
                return v
        return None

    def _safe_score(self, det) -> float:
        """
        Try common names for detection confidence. Returns float in [0,1] or 0.0 if absent.
        """
        try:
            return float(det.get_confidence())
        except Exception:
            pass
        for name in ("score", "get_score", "confidence", "get_probability", "probability"):
            try:
                v = getattr(det, name)
                v = v() if callable(v) else v
                if v is not None:
                    return float(v)
            except Exception:
                continue
        # Some post-processors attach class probs; use max if present
        try:
            cls = det.get_objects_typed(hailo.HAILO_CLASSIFICATION)
            if cls:
                probs = getattr(cls[0], "get_probabilities", None)
                if callable(probs):
                    p = probs()
                    if p is not None and len(p):
                        return float(max(p))
        except Exception:
            pass
        return 0.0
# ======================================================================================
# One-hit detection: returns a SINGLE (label, u, v) then stops
# ======================================================================================
# detection_helper.py

from typing import Optional, Tuple
import threading

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

from utils.detector_helper import DetectionHelper
from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import GStreamerDetectionApp


# ---- Minimal callback state (no multiprocessing, no frames) -----------------
class _MinimalCallback:
    def __init__(self):
        self.frame_count = 0
        self.use_frame = False  # we don't pull frames in one-hit mode

    def increment(self):
        self.frame_count += 1

    def get_count(self):
        return self.frame_count


class _OneFruitUserData(_MinimalCallback):
    """State holder for 'first detection wins'."""
    def __init__(self, require_label: Optional[str] = None):
        super().__init__()
        self.det_evt = threading.Event()
        self.result: Optional[Tuple[str, int, int]] = None
        self._helper = DetectionHelper(require_label=require_label)
        self._app = None  # set by detect_one_fruit_uv


def _one_fruit_pad_probe(pad, info, user_data: _OneFruitUserData):
    """
    On first detection, capture (label,u,v), signal event,
    and schedule EOS on the GLib main loop to exit cleanly.
    """
    buffer = info.get_buffer()
    if buffer is None or user_data.det_evt.is_set():
        return Gst.PadProbeReturn.OK

    dets = user_data._helper.parse(pad, buffer)  # [{'label','u','v','track_id'}...]
    if dets:
        d0 = dets[0]
        user_data.result = (d0["label"], int(d0["u"]), int(d0["v"]))
        user_data.det_evt.set()

        # Tell the pipeline to end from the GLib thread (clean teardown).
        if user_data._app and getattr(user_data._app, "pipeline", None):
            GLib.idle_add(user_data._app.pipeline.send_event, Gst.Event.new_eos())

    return Gst.PadProbeReturn.OK


def detect_one_fruit_uv(
    hef_path: str,
    input_source: str,
    labels_json: Optional[str] = None,
    require_label: Optional[str] = None,
    timeout: float = 2.5,
) -> Optional[Tuple[str, int, int]]:
    """
    Return ONE detection as (label, u, v) from the live pipeline, or None on timeout.
    """
    user = _OneFruitUserData(require_label=require_label)
    app = GStreamerDetectionApp(_one_fruit_pad_probe, user)
    user._app = app  # so the probe can post EOS

    # Make sure app uses YOUR paths (don’t let it fall back to example video)
    if hasattr(app, "options_menu"):
        app.options_menu.input = input_source
        app.options_menu.hef_path = hef_path
        app.options_menu.labels_json = labels_json
    if hasattr(app, "video_source"):
        app.video_source = input_source
    if hasattr(app, "hef_path"):
        app.hef_path = hef_path

    t = threading.Thread(target=app.run, daemon=True)
    t.start()

    try:
        ok = user.det_evt.wait(timeout=timeout)
        return user.result if ok else None
    finally:
        # EOS was posted from the probe; give the mainloop a moment to unwind.
        t.join(timeout=1.5)
        if t.is_alive():
            # Fallback: if the app exposes stop(), call it.
            try:
                if hasattr(app, "stop"):
                    app.stop()
            except Exception:
                pass
# --- add this helper (place near _safe_track_id) ---

