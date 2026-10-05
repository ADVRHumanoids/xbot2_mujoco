"""ROS conversion for terrain scans; imported only when ROS is enabled."""

import numpy as np
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32

from xbot2_py_mujoco.terrain_scan import TerrainScan


def pointcloud_from_scan(scan: TerrainScan, frame_id: str):
    """Return an organized XYZ PointCloud2 in the scan body's full local frame.

    Rows are forward samples, columns lateral samples. Invalid rays retain
    their matrix positions as NaN XYZ points. Pose and time come from the scan
    snapshot, so conversion remains correct even after the robot has moved.
    """
    world_points = np.stack((scan.x, scan.y, scan.heights), axis=-1)
    # For row vectors, R_world_body inverse is applied by multiplying by R.
    points = (world_points - scan.center) @ scan.rotation
    points[~scan.valid] = np.nan
    points = points.astype(np.float32)
    seconds, nanoseconds = divmod(round(scan.time * 1e9), 1_000_000_000)
    header = Header(frame_id=frame_id, stamp=Time(sec=seconds, nanosec=nanoseconds))
    cloud = create_cloud_xyz32(header, points.reshape(-1, 3))
    cloud.height, cloud.width = scan.heights.shape
    cloud.row_step = cloud.width * cloud.point_step
    cloud.is_dense = bool(np.isfinite(points).all())
    return cloud
