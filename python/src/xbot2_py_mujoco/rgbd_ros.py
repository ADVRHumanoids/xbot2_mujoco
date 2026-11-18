"""ROS 2 messages for RGB-D snapshots; imported only when ROS is enabled."""

import sys
import mujoco
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


def messages_from_frame(frame, frame_id):
    """Convert a single coherent capture to color, depth, calibration and TF."""
    seconds, nanoseconds = divmod(round(frame.time * 1e9), 1_000_000_000)
    header = Header(frame_id=frame_id, stamp=Time(sec=seconds, nanosec=nanoseconds))
    height, width = (frame.depth if frame.depth is not None else frame.rgb).shape[:2]

    def image(array, encoding, step):
        return Image(header=header, height=height, width=width, encoding=encoding,
                     is_bigendian=int(sys.byteorder == 'big'), step=step,
                     data=np.ascontiguousarray(array).tobytes())

    color = image(frame.rgb, 'rgb8', width * 3) if frame.rgb is not None else None
    depth = (image(frame.depth.astype(np.float32, copy=False), '32FC1', width * 4)
             if frame.depth is not None else None)
    projection = np.zeros((3, 4))
    projection[:, :3] = frame.intrinsic
    info = CameraInfo(
        header=header, height=height, width=width, distortion_model='plumb_bob',
        d=[0.] * 5, k=frame.intrinsic.ravel().tolist(),
        r=np.eye(3).ravel().tolist(), p=projection.ravel().tolist(),
    )
    transform = TransformStamped()
    transform.header = Header(frame_id=frame.parent_frame, stamp=header.stamp)
    transform.child_frame_id = frame_id
    t = transform.transform.translation
    t.x, t.y, t.z = (float(x) for x in frame.parent_position)
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, np.ascontiguousarray(frame.parent_rotation).ravel())
    q = transform.transform.rotation
    q.w, q.x, q.y, q.z = (float(x) for x in quat)
    return color, depth, info, transform


class RgbdPublisher:
    def __init__(self, node, configs):
        from rclpy.qos import qos_profile_sensor_data
        qos = lambda cfg: 10 if cfg.reliable else qos_profile_sensor_data
        from tf2_ros import TransformBroadcaster
        self.configs = {cfg.camera: cfg for cfg in configs}
        self.publishers = {}
        self.tf = TransformBroadcaster(node)
        for cfg in configs:
            self.publishers[cfg.camera] = {
                stream: (node.create_publisher(Image, f'{cfg.topic}/{stream}/image_raw', qos(cfg)),
                         node.create_publisher(CameraInfo, f'{cfg.topic}/{stream}/camera_info', qos(cfg)))
                for stream in cfg.streams
            }

    def publish(self, frame):
        color, depth, info, transform = messages_from_frame(frame, self.configs[frame.camera].frame_id)
        for stream, (image_publisher, info_publisher) in self.publishers[frame.camera].items():
            image_publisher.publish(color if stream == 'color' else depth)
            info_publisher.publish(info)
        self.tf.sendTransform(transform)
