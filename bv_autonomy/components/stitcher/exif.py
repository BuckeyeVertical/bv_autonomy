"""EXIF geotags for stitch frames, ported from bv_core's frame_metadata.py.

ODM reconstructs far better from frames that carry their own position, time and camera model.
"""

import math
from dataclasses import dataclass
from datetime import datetime

import piexif

MM_PER_INCH = 25.4

_FOCAL_SCALE = 1000000
_RESOLUTION_SCALE = 10000
_METRE_SCALE = 1000
_DEGREE_SCALE = 1000
_ARCSECOND_SCALE = 10000


@dataclass(frozen=True)
class CameraExif:
    """Static EXIF describing one calibrated camera."""

    make: str
    model: str
    focal_length_mm: float
    sensor_width_mm: float
    sensor_height_mm: float

    @classmethod
    def from_calibration(cls, make: str, model: str, fx: float, fy: float,
                         calib_width_px: int, calib_height_px: int, sensor_width_mm: float):
        """Solve the physical camera that produces these intrinsics.

        fx and the sensor's physical width give the focal length; the sensor height is then
        solved from fy rather than assumed square (a 4K RX0 II frame is a 16:9 crop of a 3:2 sensor).
        """
        if min(fx, fy, sensor_width_mm) <= 0.0:
            raise ValueError("focal lengths and sensor width must be positive")
        if min(calib_width_px, calib_height_px) <= 0:
            raise ValueError("calibration dimensions must be positive")
        focal_length_mm = fx * sensor_width_mm / float(calib_width_px)
        sensor_height_mm = focal_length_mm * float(calib_height_px) / fy
        return cls(make, model, focal_length_mm, float(sensor_width_mm), sensor_height_mm)

    def exif_dict(self, image_width_px: int, image_height_px: int) -> dict:
        """piexif IFDs for an image of this pixel size (the focal-plane resolution follows it)."""
        if min(image_width_px, image_height_px) <= 0:
            raise ValueError("image dimensions must be positive")
        x_res = image_width_px * MM_PER_INCH / self.sensor_width_mm
        y_res = image_height_px * MM_PER_INCH / self.sensor_height_mm
        return {
            "0th": {
                piexif.ImageIFD.Make: self.make.encode("ascii", "replace"),
                piexif.ImageIFD.Model: self.model.encode("ascii", "replace"),
            },
            "Exif": {
                piexif.ExifIFD.FocalLength: _rational(self.focal_length_mm, _FOCAL_SCALE),
                piexif.ExifIFD.FocalPlaneXResolution: _rational(x_res, _RESOLUTION_SCALE),
                piexif.ExifIFD.FocalPlaneYResolution: _rational(y_res, _RESOLUTION_SCALE),
                piexif.ExifIFD.FocalPlaneResolutionUnit: 2,
            },
        }


@dataclass(frozen=True)
class FrameGeotag:
    """Where, when and which way the drone was when one frame was taken."""

    latitude: float
    longitude: float
    altitude_m: float | None
    captured_at: datetime
    heading_deg: float | None = None

    def exif_dict(self) -> dict:
        gps = {
            piexif.GPSIFD.GPSLatitude: _dms_rationals(self.latitude),
            piexif.GPSIFD.GPSLatitudeRef: b"N" if self.latitude >= 0.0 else b"S",
            piexif.GPSIFD.GPSLongitude: _dms_rationals(self.longitude),
            piexif.GPSIFD.GPSLongitudeRef: b"E" if self.longitude >= 0.0 else b"W",
        }
        if self.altitude_m is not None:
            gps[piexif.GPSIFD.GPSAltitude] = _rational(abs(self.altitude_m), _METRE_SCALE)
            gps[piexif.GPSIFD.GPSAltitudeRef] = 0 if self.altitude_m >= 0.0 else 1
        if self.heading_deg is not None:
            gps[piexif.GPSIFD.GPSImgDirection] = _rational(self.heading_deg, _DEGREE_SCALE)
            gps[piexif.GPSIFD.GPSImgDirectionRef] = b"T"
        return {
            "Exif": {
                piexif.ExifIFD.DateTimeOriginal: self.captured_at.strftime("%Y:%m:%d %H:%M:%S").encode("ascii"),
                piexif.ExifIFD.SubSecTimeOriginal: f"{self.captured_at.microsecond // 1000:03d}".encode("ascii"),
            },
            "GPS": gps,
        }


def build_exif_bytes(camera: CameraExif, geotag: FrameGeotag, image_width_px: int, image_height_px: int) -> bytes:
    """Merge the camera and frame metadata into one EXIF block."""
    cam = camera.exif_dict(image_width_px, image_height_px)
    frame = geotag.exif_dict()
    return piexif.dump({
        "0th": dict(cam["0th"]),
        "Exif": {**cam["Exif"], **frame["Exif"]},
        "GPS": dict(frame["GPS"]),
        "1st": {},
        "thumbnail": None,
    })


def _rational(value: float, scale: int) -> tuple[int, int]:
    return (int(round(value * scale)), scale)


def _dms_rationals(decimal_degrees: float) -> list[tuple[int, int]]:
    """Split a decimal degree into EXIF's degree/minute/second triple (the sign goes in a ref tag)."""
    remaining = abs(decimal_degrees)
    degrees = int(remaining)
    minutes_total = (remaining - degrees) * 60.0
    minutes = int(minutes_total)
    seconds = (minutes_total - minutes) * 60.0
    ticks = int(round(seconds * _ARCSECOND_SCALE))
    if ticks >= 60 * _ARCSECOND_SCALE:
        ticks -= 60 * _ARCSECOND_SCALE
        minutes += 1
    if minutes >= 60:
        minutes -= 60
        degrees += 1
    return [(degrees, 1), (minutes, 1), (ticks, _ARCSECOND_SCALE)]
