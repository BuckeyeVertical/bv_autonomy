import cv2
import numpy as np
import pytest

from bv_autonomy.crop import _window, annotated_crop
from bv_autonomy.types import Detection


def detection(frame, box, label="person", confidence=0.94):
    frame.image = np.full((1000, 2000, 3), 128, dtype=np.uint8)
    return Detection(label, confidence, box, frame)


def decode(jpeg: bytes) -> np.ndarray:
    return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)


def test_small_box_gets_at_least_min_px_of_context(frame):
    image = decode(annotated_crop(detection(frame, (995, 495, 1005, 505))))
    assert image.shape[:2] == (320, 320)


def test_large_crop_is_shrunk_to_max_px_on_its_longer_side(frame):
    image = decode(annotated_crop(detection(frame, (600, 200, 1400, 800))))
    assert max(image.shape[:2]) == 1024
    assert image.shape[:2] == (512, 1024)          # 2000 x 1000 after clipping to the frame, halved


@pytest.mark.parametrize("box,window", [
    ((0, 0, 10, 10), (0, 0, 320, 320)),                  # shifted in from the top-left corner
    ((1990, 990, 2000, 1000), (1680, 680, 2000, 1000)),  # and from the bottom-right
])
def test_window_shifts_inside_the_frame_instead_of_shrinking(box, window):
    assert _window(box, 2000, 1000, 2.5, 320) == window


def test_box_and_label_are_drawn(frame):
    image = decode(annotated_crop(detection(frame, (980, 480, 1020, 520))))
    green = (image[:, :, 1] > 180) & (image[:, :, 0] < 140) & (image[:, :, 2] < 140)
    assert green.sum() > 200                       # the box outline and the label chip
    assert not green[150:170, 150:170].any()       # inside the box is left alone
