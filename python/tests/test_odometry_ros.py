from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('nav_msgs.msg')
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message, serialize_message

from xbot2_py_mujoco.odometry_ros import OdometryPublisher


def scene():
    model = mujoco.MjModel.from_xml_string('''<mujoco>
      <option gravity="0 0 0"/><worldbody>
        <body name="robot" pos="1 2 3"><freejoint/>
          <geom type="box" size=".1 .2 .3" pos=".4 .5 .6"/>
          <body name="child" pos="0 0 1"><geom size=".1"/></body>
        </body>
        <body name="fixed" pos="4 5 6"><geom size=".1"/></body>
        <body pos="7 8 9"><geom size=".1"/></body>
      </worldbody></mujoco>''')
    return model, mujoco.MjData(model)


def recording_node():
    topics = {}

    def create_publisher(message_type, topic, qos):
        assert message_type is Odometry
        topics[topic] = []
        return SimpleNamespace(publish=topics[topic].append)

    return SimpleNamespace(create_publisher=create_publisher), topics


def xyz(vector):
    return [vector.x, vector.y, vector.z]


def test_root_discovery_pose_and_local_twist_at_body_origin():
    model, data = scene()
    node, topics = recording_node()
    publisher = OdometryPublisher(node, model)
    assert set(topics) == {'/robot/odom'}
    data.qpos[3:7] = [.5, .5, .5, .5]
    data.qvel[:] = [1, 2, 3, .4, .5, .6]
    mujoco.mj_forward(model, data)
    mujoco.mj_step(model, data)
    # The post-step Cartesian cache deliberately remains stale here.
    stale_position = data.xpos.copy()
    data.time = 1.9999999996
    publisher.publish(data)
    np.testing.assert_array_equal(data.xpos, stale_position)
    msg = topics['/robot/odom'][0]
    assert msg.header.frame_id == 'world' and msg.child_frame_id == 'robot'
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (2, 0)
    np.testing.assert_allclose(xyz(msg.pose.pose.position), data.qpos[:3])
    q = msg.pose.pose.orientation
    np.testing.assert_allclose([q.w, q.x, q.y, q.z], data.qpos[3:7])
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, data.qpos[3:7])
    np.testing.assert_allclose(xyz(msg.twist.twist.linear),
                               rotation.reshape(3, 3).T @ data.qvel[:3], atol=1e-12)
    np.testing.assert_allclose(xyz(msg.twist.twist.angular), data.qvel[3:6])
    restored = deserialize_message(serialize_message(msg), Odometry)
    assert restored == msg


@pytest.mark.parametrize('joint', ['free', 'ball', 'slide', 'hinge'])
def test_only_roots_with_degrees_of_freedom_are_published(joint):
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
      <body><joint type="{joint}"/><geom size=".1"/></body>
      <body name="fixed"><geom size=".1"/>
        <body name="moving_child"><joint/><geom size=".1"/></body>
      </body>
    </worldbody></mujoco>''')
    node, topics = recording_node()
    publisher = OdometryPublisher(node, model)
    publisher.publish(mujoco.MjData(model))
    assert set(topics) == {'/body_1/odom'}
    assert len(topics['/body_1/odom']) == 1


@pytest.mark.parametrize('ros', [False, True])
def test_simulator_odometry_follows_state_decimation_and_pause(monkeypatch, ros):
    import rclpy
    import rclpy.node
    from xbot2_py_mujoco import simulator_wrapper

    topics = {}

    def create_publisher(message_type, topic, qos):
        topics[topic] = []
        return SimpleNamespace(publish=topics[topic].append)

    monkeypatch.setattr(rclpy, 'init', lambda: None)
    monkeypatch.setattr(rclpy.node, 'Node', lambda *args:
                        SimpleNamespace(create_publisher=create_publisher))
    monkeypatch.setattr(simulator_wrapper, 'MjXbot2Bridge', lambda **kwargs:
                        SimpleNamespace(receive=lambda: None, send_state=lambda: None,
                                        close=lambda: None))
    model, _ = scene()
    sim = simulator_wrapper.SimulatorWrapper(
        model, viewer=False, ros=ros, send_state_decimation=2, target_rtf=1e9,
    )
    try:
        sim.run()
        if ros:
            assert not topics['/robot/odom']
        sim.run()
        if ros:
            assert len(topics['/robot/odom']) == 1
            assert topics['/robot/odom'][0].header.stamp == topics['/clock'][-1].clock
        else:
            assert not topics and sim.odometry_publisher is None
        sim.set_paused()
        sim.run()
        if ros:
            assert len(topics['/robot/odom']) == 1
    finally:
        sim.close()
