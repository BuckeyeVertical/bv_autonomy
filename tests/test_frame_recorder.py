import logging
import math
from dataclasses import replace
from datetime import datetime

import cv2
import piexif
import pytest

from bv_autonomy.components.stitcher import frame_recorder as recording
from bv_autonomy.components.stitcher.exif import CameraExif, FrameGeotag, build_exif_bytes
from bv_autonomy.components.terrain import Terrain
from bv_autonomy.types import Waypoint


@pytest.fixture
def make_recorder(tmp_path):
    recorders = []

    def make(**kwargs):
        recorder = recording.FrameRecorder(tmp_path / "images", overlap=0.5, **kwargs)
        recorders.append(recorder)
        return recorder

    yield make
    for recorder in recorders:
        recorder._writer.shutdown(wait=True)


def moved(frame, east=0, altitude=20, heading=0):
    return replace(frame, pose=replace(
        frame.pose,
        position=Waypoint(0, math.degrees(east / 6378137), altitude),
        yaw=math.radians(heading),
    ))


def test_capture_spacing_uses_last_kept_frame_and_current_altitude(make_recorder, frame):
    recorder = make_recorder()
    assert recorder.add(moved(frame, east=0))
    assert not recorder.add(moved(frame, east=6))
    assert recorder.add(moved(frame, east=11))
    assert not recorder.add(moved(frame, east=25, altitude=40))
    assert recorder.add(moved(frame, east=32, altitude=40))
    recorder.flush()
    rows = recorder.rows()
    assert [path.name for path in rows[1]] == ["row1_1.jpg", "row1_2.jpg", "row1_3.jpg"]
    assert all(cv2.imread(str(path)) is not None for path in rows[1])


def test_new_row_uses_original_row_heading_and_handles_north_wrap(make_recorder, frame):
    recorder = make_recorder()
    assert recorder.add(moved(frame, heading=350))
    assert not recorder.add(moved(frame, heading=10))
    assert recorder.add(moved(frame, heading=85))
    recorder.flush()
    assert {row: [path.name for path in paths] for row, paths in recorder.rows().items()} == {
        1: ["row1_1.jpg"], 2: ["row2_1.jpg"],
    }


def test_flush_propagates_background_image_write_failure(make_recorder, frame, monkeypatch):
    recorder = make_recorder()
    monkeypatch.setattr(recording.cv2, "imwrite", lambda *args: False)
    recorder.add(frame)
    with pytest.raises(OSError, match="could not write"):
        recorder.flush()


@pytest.fixture
def camera_exif():
    return CameraExif.from_calibration("Sony", "RX0", 800, 1000, 1600, 900, 12)


def ratio(value):
    return value[0] / value[1]


def test_written_jpeg_preserves_calibration_terrain_height_and_capture_metadata(
    make_recorder, camera_exif, frame, monkeypatch,
):
    class FieldTerrain(Terrain):
        def ground_msl(self, lat, lon):
            return 110

    captured = datetime(2026, 10, 23, 12, 34, 56, 123456)

    class CaptureClock:
        @staticmethod
        def now():
            return captured

    monkeypatch.setattr(recording, "datetime", CaptureClock)
    recorder = make_recorder(camera=camera_exif, terrain=FieldTerrain())
    frame.pose.position = Waypoint(-12.5, 96.25, 20)
    frame.pose.yaw = -math.pi / 2
    recorder.add(frame)
    recorder.flush()
    tags = piexif.load(str(recorder.rows()[1][0]))
    gps, exif = tags["GPS"], tags["Exif"]
    assert gps[piexif.GPSIFD.GPSLatitudeRef] == b"S"
    assert tuple(map(ratio, gps[piexif.GPSIFD.GPSLatitude])) == (12, 30, 0)
    assert gps[piexif.GPSIFD.GPSLongitudeRef] == b"E"
    assert tuple(map(ratio, gps[piexif.GPSIFD.GPSLongitude])) == (96, 15, 0)
    assert ratio(gps[piexif.GPSIFD.GPSAltitude]) == 130
    assert gps[piexif.GPSIFD.GPSAltitudeRef] == 0
    assert ratio(gps[piexif.GPSIFD.GPSImgDirection]) == 270
    assert exif[piexif.ExifIFD.DateTimeOriginal] == b"2026:10:23 12:34:56"
    assert exif[piexif.ExifIFD.SubSecTimeOriginal] == b"123"
    assert ratio(exif[piexif.ExifIFD.FocalLength]) == 6
    assert ratio(exif[piexif.ExifIFD.FocalPlaneXResolution]) == pytest.approx(100 * 25.4 / 12)
    assert ratio(exif[piexif.ExifIFD.FocalPlaneYResolution]) == pytest.approx(100 * 25.4 / 5.4)


def test_geotag_failure_keeps_usable_jpeg_and_logs_error(make_recorder, camera_exif, frame, monkeypatch, caplog):
    recorder = make_recorder(camera=camera_exif)

    def fail_insert(*args):
        raise OSError("EXIF insert failed")

    monkeypatch.setattr(recording.piexif, "insert", fail_insert)
    recorder.add(frame)
    recorder.flush()
    assert cv2.imread(str(recorder.rows()[1][0])) is not None
    assert [r.levelno for r in caplog.records if r.name == recording.__name__] == [logging.ERROR]


@pytest.mark.parametrize("altitude", [None, -12.345])
def test_geotag_distinguishes_missing_altitude_from_below_sea_level(camera_exif, altitude):
    geotag = FrameGeotag(0, 0, altitude, datetime(2026, 10, 23))
    gps = piexif.load(build_exif_bytes(camera_exif, geotag, 800, 450))["GPS"]
    if altitude is None:
        assert piexif.GPSIFD.GPSAltitude not in gps
        assert piexif.GPSIFD.GPSAltitudeRef not in gps
    else:
        assert ratio(gps[piexif.GPSIFD.GPSAltitude]) == abs(altitude)
        assert gps[piexif.GPSIFD.GPSAltitudeRef] == 1
    assert piexif.GPSIFD.GPSImgDirection not in gps


def test_geotag_rounding_carries_seconds_without_invalid_dms(camera_exif):
    geotag = FrameGeotag(
        12 + 59 / 60 + 59.99996 / 3600,
        45 + 12 / 60 + 59.99996 / 3600,
        None, datetime(2026, 10, 23),
    )
    gps = piexif.load(build_exif_bytes(camera_exif, geotag, 800, 450))["GPS"]
    assert tuple(map(ratio, gps[piexif.GPSIFD.GPSLatitude])) == (13, 0, 0)
    assert tuple(map(ratio, gps[piexif.GPSIFD.GPSLongitude])) == (45, 13, 0)
