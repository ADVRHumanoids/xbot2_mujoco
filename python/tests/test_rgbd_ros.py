from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import TransformStamped

from xbot2_py_mujoco.rgbd import RgbdCameraCfg, RgbdFrame
from xbot2_py_mujoco.rgbd_ros import messages_from_frame, RgbdPublisher


def frame():
    return RgbdFrame('front', 1.9999999996, np.arange(18, dtype=np.uint8).reshape(2, 3, 3),
                     np.array([[1., 2., np.nan], [3., 4., 5.]], dtype=np.float32),
                     np.array([[10., 0, 1], [0, 11., .5], [0, 0, 1.]]),
                     np.array([1., 2., 3.]), np.diag([1., -1., -1.]), 'mount',
                     np.array([.1, .2, .3]), np.diag([1., -1., -1.]))


def test_messages_serialize_calibration_optical_pose_and_shared_stamps():
    original = frame()
    color, depth, info, tf = messages_from_frame(original, 'front_optical')
    for message in (color, depth, info, tf):
        restored = deserialize_message(serialize_message(message), type(message))
        assert restored == message
        assert (message.header.stamp.sec, message.header.stamp.nanosec) == (2, 0)
    assert color.encoding == 'rgb8' and color.step == 9
    assert depth.encoding == '32FC1' and depth.step == 12
    assert color.header.frame_id == depth.header.frame_id == info.header.frame_id == 'front_optical'
    assert (color.height, color.width) == (2, 3)
    np.testing.assert_array_equal(np.frombuffer(color.data, np.uint8).reshape(2, 3, 3), original.rgb)
    np.testing.assert_array_equal(np.frombuffer(depth.data, np.float32).reshape(2, 3), original.depth)
    np.testing.assert_array_equal(np.array(info.k).reshape(3, 3), original.intrinsic)
    assert list(info.d) == [0.] * 5
    np.testing.assert_array_equal(np.array(info.p).reshape(3, 4)[:, :3], original.intrinsic)
    assert tf.header.frame_id == 'mount' and tf.child_frame_id == 'front_optical'
    assert tf.transform.translation.x == .1
    assert abs(tf.transform.rotation.x) == 1
    assert tf.transform.rotation.w == 0


def test_publish_topics_qos_and_synchronized_camera_info(monkeypatch):
    import tf2_ros
    published = {}
    def create(message_type, topic, qos):
        assert qos is qos_profile_sensor_data
        published[topic] = []
        return SimpleNamespace(publish=published[topic].append)
    transforms = []
    monkeypatch.setattr(tf2_ros, 'TransformBroadcaster', lambda node: SimpleNamespace(sendTransform=transforms.append))
    publisher = RgbdPublisher(SimpleNamespace(create_publisher=create), [RgbdCameraCfg('front')])
    publisher.publish(frame())
    assert set(published) == {'/front/color/image_raw', '/front/depth/image_raw', '/front/color/camera_info', '/front/depth/camera_info'}
    assert all(len(messages) == 1 for messages in published.values())
    assert published['/front/color/camera_info'] == published['/front/depth/camera_info']
    assert transforms[0].child_frame_id == 'front_optical_frame'


@pytest.mark.parametrize('ros', [True, False])
def test_simulator_camera_lifecycle_ros_optional_pause_and_respawn(monkeypatch, ros):
    import rclpy
    import rclpy.node
    from xbot2_py_mujoco import simulator_wrapper as wrapper
    frames = []
    made = []
    class Manager:
        def __init__(self, model, configs):
            self.submitted = self.resets = 0
            self.closed = False
            self.latest_frames = {'front': frame()}
            made.append(self)
        def submit(self, data):
            self.submitted += 1
        def poll(self):
            result = list(frames)
            frames.clear()
            return result
        def reset(self):
            self.resets += 1
        def close(self):
            self.closed = True
    monkeypatch.setattr(wrapper, 'RgbdCameraManager', Manager)
    monkeypatch.setattr(wrapper, 'MjXbot2Bridge', lambda **kwargs: SimpleNamespace(
        receive=lambda: None, send_state=lambda: None, close=lambda: None))
    calls = []
    monkeypatch.setattr(rclpy, 'init', lambda: None)
    monkeypatch.setattr(rclpy.node, 'Node', lambda name: SimpleNamespace(create_publisher=lambda *args: SimpleNamespace(publish=lambda msg: None)))
    import xbot2_py_mujoco.rgbd_ros as ros_module
    monkeypatch.setattr(ros_module, 'RgbdPublisher', lambda node, configs: SimpleNamespace(publish=calls.append))
    m = mujoco.MjModel.from_xml_string('''<mujoco><worldbody><body name="mount">
       <freejoint/><geom type="box" size=".1 .1 .1"/><camera name="front"/>
    </body></worldbody></mujoco>''')
    sim = wrapper.SimulatorWrapper(m, viewer=False, ros=ros, target_rtf=1e9, rgbd_cameras=[RgbdCameraCfg('front')])
    try:
        frames.append(frame())
        sim.run()
        assert made[0].submitted == 1
        assert len(calls) == int(ros)
        assert sim.rgbd_frames['front'].camera == 'front'
        sim.set_paused(True)
        sim.run()
        assert made[0].submitted == 1
        sim.set_paused(False)
        sim._respawn()
        assert made[0].resets == 2  # Initial state and respawn.
    finally:
        sim.close()
    assert made[0].closed
