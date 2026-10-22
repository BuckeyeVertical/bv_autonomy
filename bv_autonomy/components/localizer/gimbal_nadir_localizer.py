"""Localizer for a gimbal that holds the camera pointing straight down, as in bv_core's Localizer."""

import math

import cv2
import numpy as np
from geographiclib.geodesic import Geodesic
from scipy.spatial.transform import Rotation

from bv_autonomy.components.localizer.base import Localizer
from bv_autonomy.types import Detection, Waypoint

# bv_core's camera_orientation (config/filtering_params.yaml): turns the camera's
# optical axis to point down, with the image's top edge toward the drone's nose.
DEFAULT_MOUNT_XYZ = (3.1416, 0.0, -1.5708)


class GimbalNadirLocalizer(Localizer):
    """Projects the center of a detection's box onto flat ground.

    The gimbal keeps the camera level, so the drone's roll and pitch never reach it:
    only the drone's yaw and position are used. The ground is treated as flat at the
    drone's height above it, as bv_core did with the altitude above home.
    """

    def __init__(self, mount_xyz: tuple[float, float, float] = DEFAULT_MOUNT_XYZ):
        """
        Args:
            mount_xyz: Camera mount as extrinsic x, y, z Euler angles in radians,
                from the camera frame (z along the optical axis) to the drone's level body frame.
        """
        self._mount = Rotation.from_euler("xyz", mount_xyz)

    def locate(self, detection: Detection) -> Waypoint | None:
        frame = detection.frame
        k = frame.intrinsics
        height = frame.pose.position.alt
        if height <= 0:
            return None

        # 1) Undistort the box center to the normalized image plane.
        x1, y1, x2, y2 = detection.box
        center = np.array([[[(x1 + x2) / 2, (y1 + y2) / 2]]], dtype=np.float32)
        camera_matrix = np.array([[k.fx, 0, k.cx], [0, k.fy, k.cy], [0, 0, 1]], dtype=np.float64)
        u, v = cv2.undistortPoints(center, camera_matrix, np.asarray(k.dist, dtype=np.float64)).reshape(2)

        # 2) Turn it into a ray, rotate it into east-north-up by the mount and the drone's yaw.
        ray_camera = np.array([u, v, 1.0])
        ray_camera /= np.linalg.norm(ray_camera)
        yaw_enu = math.pi / 2 - frame.pose.yaw      # pose yaw is clockwise from north
        east, north, up = (Rotation.from_euler("z", yaw_enu) * self._mount).apply(ray_camera)

        # 3) Where the ray meets the ground.
        if up >= 0:
            return None
        scale = height / -up
        east, north = east * scale, north * scale

        # 4) Walk that far from the drone along the earth's surface.
        start = frame.pose.position
        azimuth = math.degrees(math.atan2(east, north))
        end = Geodesic.WGS84.Direct(start.lat, start.lon, azimuth, math.hypot(east, north))
        return Waypoint(lat=end["lat2"], lon=end["lon2"], alt=0.0)

