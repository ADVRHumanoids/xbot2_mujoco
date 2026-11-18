"""ROS messages for IMU samples; imported only when ROS is enabled."""

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

from xbot2_py_mujoco.imu import ImuCfg, ImuSample


def imu_message(sample: ImuSample, cfg: ImuCfg) -> Imu:
    seconds, nanoseconds = divmod(round(sample.time * 1e9), 1_000_000_000)
    msg = Imu(header=Header(frame_id=cfg.frame_id, stamp=Time(sec=seconds, nanosec=nanoseconds)))
    q = msg.orientation
    q.w, q.x, q.y, q.z = (float(v) for v in sample.orientation)
    w, a = msg.angular_velocity, msg.linear_acceleration
    w.x, w.y, w.z = (float(v) for v in sample.angular_velocity)
    a.x, a.y, a.z = (float(v) for v in sample.linear_acceleration)
    return msg
