"""An annotated crop around one detection, for the operator to judge it by (bv_core's detection_crop.py)."""

import cv2

from bv_autonomy.types import Detection

_BOX_BGR = (84, 220, 64)
_TEXT_BGR = (12, 24, 12)


def annotated_crop(detection: Detection, margin: float = 2.5, min_px: int = 320, max_px: int = 1024,
                   quality: int = 85) -> bytes:
    """JPEG of the frame around the detection's box, with the box and "label confidence" drawn on.

    The crop is margin times the box, at least min_px, and is shrunk so its longer side is at most
    max_px. A whole-frame downscale would make a distant person a few unreadable pixels.
    """
    image = detection.frame.image
    height, width = image.shape[:2]
    left, top, right, bottom = _window(detection.box, width, height, margin, min_px)
    crop = image[top:bottom, left:right].copy()
    x1, y1, x2, y2 = detection.box
    box = [x1 - left, y1 - top, x2 - left, y2 - top]

    scale = min(1.0, max_px / max(crop.shape[:2]))
    if scale < 1.0:
        size = (max(1, round(crop.shape[1] * scale)), max(1, round(crop.shape[0] * scale)))
        crop = cv2.resize(crop, size, interpolation=cv2.INTER_AREA)
        box = [v * scale for v in box]
    _annotate(crop, box, f"{detection.label} {detection.confidence:.2f}")

    ok, jpeg = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("could not encode the crop")
    return jpeg.tobytes()


def _window(box, width: int, height: int, margin: float, min_px: int) -> tuple[int, int, int, int]:
    """The crop's left, top, right, bottom. Near an edge it's shifted, not shrunk, so it keeps its context."""
    x1, y1, x2, y2 = box
    w = min(max(max(x2 - x1, 1.0) * margin, min_px), width)
    h = min(max(max(y2 - y1, 1.0) * margin, min_px), height)
    left = min(max((x1 + x2) / 2 - w / 2, 0.0), width - w)
    top = min(max((y1 + y2) / 2 - h / 2, 0.0), height - h)
    return round(left), round(top), round(left + w), round(top + h)


def _annotate(image, box, label: str) -> None:
    h, w = image.shape[:2]
    x1, y1, x2, y2 = (round(max(0, min(v, limit - 1))) for v, limit in zip(box, (w, h, w, h)))
    cv2.rectangle(image, (x1, y1), (x2, y2), _BOX_BGR, 2)

    (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    bottom = y1 if y1 - text_h - 8 >= 0 else min(h, y2 + text_h + 8)
    top = max(0, bottom - text_h - 8)
    left = max(0, min(x1, w - text_w - 8))
    cv2.rectangle(image, (left, top), (left + text_w + 8, bottom), _BOX_BGR, -1)
    cv2.putText(image, label, (left + 4, bottom - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, _TEXT_BGR, 2, cv2.LINE_AA)
