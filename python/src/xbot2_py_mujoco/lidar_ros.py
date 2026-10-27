"""ROS conversion for lidar scans; imported only when ROS is enabled."""

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from xbot2_py_mujoco.lidar import LidarScan

# Hesai driver layout: x y z intensity (float32), ring (uint16), timestamp (float64, s).
_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'timestamp'],
                   'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f8'],
                   'offsets': [0, 4, 8, 12, 16, 18], 'itemsize': 26})
_FIELDS = [PointField(name=name, offset=offset, datatype=datatype, count=1)
           for name, offset, datatype in (('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32),
                                          ('z', 8, PointField.FLOAT32),
                                          ('intensity', 12, PointField.FLOAT32),
                                          ('ring', 16, PointField.UINT16),
                                          ('timestamp', 18, PointField.FLOAT64))]


def pointcloud_from_scan(scan: LidarScan, frame_id: str) -> PointCloud2:
    """One row, ring fastest, misses as (0, 0, 0), stamped at the sweep start; timestamp: the
    absolute time of each point."""
    cloud = np.zeros(len(scan.points), dtype=_DTYPE)
    cloud['x'], cloud['y'], cloud['z'] = scan.points.T
    cloud['ring'] = scan.rings
    cloud['timestamp'] = scan.times
    seconds, nanoseconds = divmod(round(scan.time * 1e9), 1_000_000_000)
    return PointCloud2(
        header=Header(frame_id=frame_id, stamp=Time(sec=seconds, nanosec=nanoseconds)),
        height=1, width=len(cloud), fields=_FIELDS, is_bigendian=False,
        point_step=_DTYPE.itemsize, row_step=_DTYPE.itemsize * len(cloud),
        data=cloud.tobytes(), is_dense=True,
    )


def site_transform(model, cfg) -> TransformStamped:
    """The lidar frame in its site's parent body, for /tf_static."""
    site = model.site(cfg.site)
    transform = TransformStamped()
    transform.header.frame_id = model.body(int(site.bodyid[0])).name
    transform.child_frame_id = cfg.frame_id
    t = transform.transform.translation
    t.x, t.y, t.z = (float(x) for x in site.pos)
    q = transform.transform.rotation
    q.w, q.x, q.y, q.z = (float(x) for x in site.quat)
    return transform
