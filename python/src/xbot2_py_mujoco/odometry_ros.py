"""ROS messages for odometry samples; imported only when ROS is enabled."""

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from xbot2_py_mujoco.odometry import OdometryCfg, OdometrySample


def odometry_message(sample: OdometrySample, cfg: OdometryCfg) -> Odometry:
    """Pose in frame_id, twist in child_frame_id."""
    seconds, nanoseconds = divmod(round(sample.time * 1e9), 1_000_000_000)
    msg = Odometry(header=Header(frame_id=cfg.frame_id, stamp=Time(sec=seconds, nanosec=nanoseconds)),
                   child_frame_id=cfg.child_frame_id)
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    p.x, p.y, p.z = (float(v) for v in sample.position)
    q.w, q.x, q.y, q.z = (float(v) for v in sample.orientation)
    v, w = msg.twist.twist.linear, msg.twist.twist.angular
    v.x, v.y, v.z = (float(c) for c in sample.linear_velocity)
    w.x, w.y, w.z = (float(c) for c in sample.angular_velocity)
    return msg
