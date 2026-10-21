"""Feature-matching stitcher, ported from bv_core's stitching.py.

Each row's frames are chained together by ORB feature matches and homographies into a strip,
then the strips are chained top to bottom. A frame that won't match is skipped; if the next
one won't either, the row continues as a new segment after a red gap bar.
"""

import asyncio
import logging
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from bv_autonomy.components.stitcher.base import Stitcher
from bv_autonomy.components.stitcher.frame_recorder import FrameRecorder
from bv_autonomy.types import Frame

log = logging.getLogger(__name__)


class OpencvStitcher(Stitcher):
    def __init__(
        self,
        output_dir: str | Path,
        overlap: float = 0.35,
        max_width: int = 1500,
        max_images_per_row: int = 50,
        feature_count: int = 5000,
        match_confidence: float = 0.7,     # Lowe's ratio threshold
        gap_width: int = 50,
    ):
        """
        Args:
            output_dir: Where frames (in frames/) and the mosaic are written.
            overlap: Fraction of each frame shared with the next one along a row.
            max_width: Frames are downscaled to this width before stitching.
        Defaults are bv_core's.
        """
        self._output_dir = Path(output_dir)
        self._recorder = FrameRecorder(self._output_dir / "frames", overlap)
        self._max_width = max(100, max_width)
        self._max_per_row = max(2, max_images_per_row)
        self._feature_count = feature_count
        self._match_confidence = match_confidence
        self._gap_width = gap_width

    def add(self, frame: Frame) -> None:
        self._recorder.add(frame)

    async def stitch(self) -> Path:
        return await asyncio.to_thread(self._stitch)

    def _stitch(self) -> Path:
        self._recorder.flush()
        rows = self._recorder.rows()
        if not rows:
            raise RuntimeError("no frames to stitch")
        mosaic = stitch_rows(rows, self._max_width, self._max_per_row, self._feature_count,
                             self._match_confidence, self._gap_width)
        path = self._output_dir / f"mosaic_{datetime.now():%m%d_%H%M}.jpg"
        if not cv2.imwrite(str(path), mosaic):
            raise OSError(f"could not write {path}")
        log.info("mosaic saved: %s (%dx%d px)", path, mosaic.shape[1], mosaic.shape[0])
        return path


def stitch_rows(rows: dict[int, list[Path]], max_width: int = 1500, max_per_row: int = 50,
                feature_count: int = 5000, match_confidence: float = 0.7, gap_width: int = 50) -> np.ndarray:
    """bv_core's two phases: stitch each row into a strip, then the strips top to bottom."""
    strips = {}
    failed = []
    for row in sorted(rows):
        paths = rows[row][:max_per_row]
        images = _load_and_resize(paths, max_width)
        if not images:
            failed.append((row, "no valid images loaded"))
            continue
        images = _normalize_row_orientation(images, row)
        ok, result, info = _stitch_row_with_fallback(images, feature_count, match_confidence, gap_width, f"row {row}")
        if ok:
            strips[row] = _crop_black_borders(result)
            if info["gap_count"]:
                log.warning("row %d: %d gap(s), failed frames %s", row, info["gap_count"], info["failed_frames"])
        else:
            failed.append((row, result))
    if not strips:
        raise RuntimeError("all rows failed: " + "; ".join(f"row {r}: {m}" for r, m in failed))

    if len(strips) == 1:
        mosaic = next(iter(strips.values()))
    else:
        ok, result = _stitch_sequential([strips[r] for r in sorted(strips)], feature_count, match_confidence)
        if not ok:
            raise RuntimeError(f"rows stitched but merging them failed: {result}")
        mosaic = result
    if failed:
        log.warning("%d row(s) failed: %s", len(failed), [r for r, _ in failed])
    return _crop_black_borders(mosaic)


def _normalize_row_orientation(images: list[np.ndarray], row: int) -> list[np.ndarray]:
    """Rotate the rows flown in reverse (even rows of the S) to match the first row."""
    if row % 2 == 0:
        return [cv2.rotate(image, cv2.ROTATE_180) for image in images]
    return images


def _load_and_resize(paths: list[Path], max_width: int) -> list[np.ndarray]:
    images = []
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            log.warning("failed to load %s", path)
            continue
        h, w = image.shape[:2]
        if w > max_width:
            scale = max_width / w
            image = cv2.resize(image, (max_width, int(h * scale)), interpolation=cv2.INTER_AREA)
        images.append(image)
    return images


def _find_homography(img1, img2, feature_count, match_confidence):
    """Homography mapping img2 onto img1, or (False, reason)."""
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(nfeatures=feature_count)
    kp1, des1 = orb.detectAndCompute(gray1, None)
    kp2, des2 = orb.detectAndCompute(gray2, None)
    if des1 is None or des2 is None:
        return False, "no features detected in one or both images"
    if len(kp1) < 10 or len(kp2) < 10:
        return False, f"too few features: {len(kp1)}, {len(kp2)}"

    try:
        matches = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(des1, des2, k=2)
    except cv2.error as exc:
        return False, f"feature matching failed: {exc}"
    # Keep a match only if it's clearly better than the runner-up.
    pairs = (pair for pair in matches if len(pair) == 2)
    good = [m for m, n in pairs if m.distance < match_confidence * n.distance]
    if len(good) < 10:
        return False, f"not enough good matches: {len(good)} (need at least 10)"

    pts1 = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    H, mask = cv2.findHomography(pts2, pts1, cv2.RANSAC, 5.0)
    if H is None:
        return False, "homography estimation failed (RANSAC)"
    det = np.linalg.det(H[:2, :2])
    if det < 0.5 or det > 2.0:
        # Aerial frames at one altitude should be close to a pure translation.
        log.warning("suspicious homography determinant %.3f (expected ~1.0)", det)
    return True, H


def _stitch_pair(img1, img2, H):
    """Warp img2 onto img1's canvas with H, averaging where they overlap."""
    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]
    corners2 = cv2.perspectiveTransform(np.float32([[0, 0], [w2, 0], [w2, h2], [0, h2]]).reshape(-1, 1, 2), H)
    corners1 = np.float32([[0, 0], [w1, 0], [w1, h1], [0, h1]]).reshape(-1, 1, 2)
    corners = np.concatenate([corners1, corners2], axis=0)
    x_min = int(np.floor(corners[:, 0, 0].min()))
    y_min = int(np.floor(corners[:, 0, 1].min()))
    x_max = int(np.ceil(corners[:, 0, 0].max()))
    y_max = int(np.ceil(corners[:, 0, 1].max()))
    translation = np.array([[1, 0, -x_min], [0, 1, -y_min], [0, 0, 1]], dtype=np.float64)

    canvas_w, canvas_h = x_max - x_min, y_max - y_min
    # A bad homography mustn't allocate a giant canvas.
    max_canvas = max(w1, w2) * 6
    if canvas_w > max_canvas or canvas_h > max_canvas:
        log.warning("canvas too large (%dx%d), clamping to %d", canvas_w, canvas_h, max_canvas)
        canvas_w, canvas_h = min(canvas_w, max_canvas), min(canvas_h, max_canvas)

    result = cv2.warpPerspective(img2, translation @ H, (canvas_w, canvas_h))
    tx, ty = -x_min, -y_min
    y_start, y_end = max(0, ty), min(canvas_h, ty + h1)
    x_start, x_end = max(0, tx), min(canvas_w, tx + w1)
    i1_y = y_start - ty
    i1_x = x_start - tx
    roi = img1[i1_y:i1_y + (y_end - y_start), i1_x:i1_x + (x_end - x_start)]
    canvas_roi = result[y_start:y_end, x_start:x_end]

    img1_mask = np.any(roi > 0, axis=2)
    img2_mask = np.any(canvas_roi > 0, axis=2)
    overlap = img1_mask & img2_mask
    canvas_roi[img1_mask & ~overlap] = roi[img1_mask & ~overlap]
    if np.any(overlap):
        canvas_roi[overlap] = (roi[overlap].astype(np.float32) * 0.5
                               + canvas_roi[overlap].astype(np.float32) * 0.5).astype(np.uint8)
    result[y_start:y_end, x_start:x_end] = canvas_roi
    return result


def _concatenate_segments(segments, gap_width):
    """Join row segments side by side with red bars marking the gaps."""
    max_h = max(seg.shape[0] for seg in segments)
    total_w = sum(seg.shape[1] for seg in segments) + (len(segments) - 1) * gap_width
    canvas = np.zeros((max_h, total_w, 3), dtype=np.uint8)
    x = 0
    for i, seg in enumerate(segments):
        h, w = seg.shape[:2]
        y = (max_h - h) // 2
        canvas[y:y + h, x:x + w] = seg
        x += w
        if i < len(segments) - 1:
            canvas[:, x:x + gap_width] = (0, 0, 255)
            x += gap_width
    return canvas


def _stitch_row_with_fallback(images, feature_count, match_confidence, gap_width, label=""):
    """Chain a row's frames, skipping one bad frame, or starting a new segment after two."""
    if not images:
        return False, f"no images for {label}", {}
    if len(images) == 1:
        return True, images[0], {"segments": [(0, 0)], "failed_frames": [], "gap_count": 0}

    segments, failed = [], []
    indices, mosaic = [], None
    i = 0
    while i < len(images):
        if not indices:
            indices, mosaic = [i], images[i]
            i += 1
            continue
        ok, result = _find_homography(mosaic, images[i], feature_count, match_confidence)
        if ok:
            mosaic = _stitch_pair(mosaic, images[i], result)
            indices.append(i)
            i += 1
            continue
        log.warning("%s: frame %d failed: %s", label, i + 1, result)
        if i + 1 < len(images):
            skip_ok, skip_result = _find_homography(mosaic, images[i + 1], feature_count, match_confidence)
            if skip_ok:
                mosaic = _stitch_pair(mosaic, images[i + 1], skip_result)
                failed.append(i)
                indices.append(i + 1)
                i += 2
                continue
        segments.append((mosaic, list(indices)))
        failed.append(i)
        indices, mosaic = [], None
        i += 1
    if indices:
        segments.append((mosaic, list(indices)))
    if not segments:
        return False, f"{label}: all frames failed to stitch", {}

    info = {"segments": [(idx[0], idx[-1]) for _, idx in segments], "failed_frames": failed,
            "gap_count": len(segments) - 1}
    if len(segments) == 1:
        return True, segments[0][0], info
    return True, _concatenate_segments([seg for seg, _ in segments], gap_width), info


def _stitch_sequential(images, feature_count, match_confidence):
    """Chain images in order, failing on the first one that won't match."""
    if not images:
        return False, "no images"
    mosaic = images[0]
    for i in range(1, len(images)):
        ok, result = _find_homography(mosaic, images[i], feature_count, match_confidence)
        if not ok:
            return False, f"failed at image {i + 1}/{len(images)}: {result}"
        mosaic = _stitch_pair(mosaic, images[i], result)
    return True, mosaic


def _crop_black_borders(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 5, 255, cv2.THRESH_BINARY)
    coords = cv2.findNonZero(thresh)
    if coords is None:
        return img
    x, y, w, h = cv2.boundingRect(coords)
    margin = 5
    x, y = max(0, x - margin), max(0, y - margin)
    w = min(img.shape[1] - x, w + 2 * margin)
    h = min(img.shape[0] - y, h + 2 * margin)
    return img[y:y + h, x:x + w]
