from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('sensor_msgs_py.point_cloud2')
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from xbot2_py_mujoco.terrain_scan import TerrainScanner
from xbot2_py_mujoco.terrain_scan_ros import pointcloud_from_scan


def scene():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <geom type="plane" size="10 10 .1" group="2"/>
      <body name="pelvis" pos="1 -2 1">
        <freejoint/><geom type="box" size=".1 .1 .1" group="1"/>
      </body>
    </worldbody></mujoco>''')
    return model, mujoco.MjData(model)


def points(cloud):
    dtype = '>f4' if cloud.is_bigendian else '<f4'
    return np.frombuffer(bytes(cloud.data), dtype=dtype).reshape(cloud.height, cloud.width, 3)


def test_cloud_full_body_frame_and_scan_pose_snapshot():
    model, data = scene()
    data.qpos[3:7] = np.array([.7, .2, -.3, .4]) / np.linalg.norm([.7, .2, -.3, .4])
    data.time = 2.25
    scan = TerrainScanner(model, 'pelvis', shape=(3, 2), spacing=(.2, .1)).scan(data)
    rotation = data.xmat[model.body('pelvis').id].reshape(3, 3).copy()
    center = data.xpos[model.body('pelvis').id].copy()
    # Move and rotate the robot before converting the old scan.
    data.qpos[:3] += [4, 5, 6]
    data.qpos[3:7] = [1, 0, 0, 0]
    mujoco.mj_kinematics(model, data)
    cloud = pointcloud_from_scan(scan, 'pelvis')
    assert cloud.header.frame_id == 'pelvis'
    assert cloud.header.stamp.sec == 2 and cloud.header.stamp.nanosec == 250_000_000
    assert (cloud.height, cloud.width) == (3, 2)
    assert cloud.point_step == 12 and cloud.row_step == 24
    assert len(cloud.data) == 72 and cloud.is_dense
    assert [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields] == [
        ('x', 0, PointField.FLOAT32, 1),
        ('y', 4, PointField.FLOAT32, 1),
        ('z', 8, PointField.FLOAT32, 1),
    ]
    reconstructed = points(cloud) @ rotation.T + center
    expected = np.stack((scan.x, scan.y, scan.heights), axis=-1)
    np.testing.assert_allclose(reconstructed, expected, atol=1e-6)
    restored = deserialize_message(serialize_message(cloud), PointCloud2)
    np.testing.assert_array_equal(points(restored), points(cloud))


def test_cloud_misses_keep_matrix_positions_and_stamp_rounding():
    model, data = scene()
    data.time = 1.9999999996
    scan = TerrainScanner(model, 'pelvis', shape=(2, 3)).scan(data)
    scan.geom_ids[0, 1] = -1
    scan.heights[0, 1] = np.nan
    cloud = pointcloud_from_scan(scan, 'pelvis')
    xyz = points(cloud)
    assert not cloud.is_dense
    assert (cloud.height, cloud.width) == (2, 3)
    assert np.isnan(xyz[0, 1]).all()
    assert np.isfinite(xyz[1]).all()
    assert cloud.header.stamp.sec == 2 and cloud.header.stamp.nanosec == 0
    # An entirely empty scan still has an organized NaN matrix.
    scan.geom_ids[:] = -1
    cloud = pointcloud_from_scan(scan, 'pelvis')
    assert np.isnan(points(cloud)).all() and not cloud.is_dense


@pytest.mark.parametrize('ros', [False, True])
@pytest.mark.parametrize('enabled', [False, True])
def test_simulator_publishes_new_scans_only_when_ros_enabled(monkeypatch, ros, enabled):
    import rclpy
    import rclpy.node
    from xbot2_py_mujoco import simulator_wrapper

    created = {}
    init_calls = []

    def create_publisher(message_type, topic, qos):
        published = []
        pub = SimpleNamespace(publish=published.append)
        created[topic] = (message_type, qos, published, pub)
        return pub

    monkeypatch.setattr(rclpy, 'init', lambda: init_calls.append(True))
    monkeypatch.setattr(rclpy.node, 'Node', lambda *args:
                        SimpleNamespace(create_publisher=create_publisher))
    monkeypatch.setattr(simulator_wrapper, 'MjXbot2Bridge', lambda **kwargs:
                        SimpleNamespace(send_state=lambda: None))
    model, _ = scene()
    scanner = TerrainScanner(model, 'pelvis', shape=(3, 2)) if enabled else None
    sim = simulator_wrapper.SimulatorWrapper(
        model, viewer=False, ros=ros, target_rtf=1e9,
        terrain_scanner=scanner, terrain_scan_decimation=2,
        terrain_scan_topic='/custom_heightscan',
    )
    try:
        assert bool(init_calls) == ros
        if ros and enabled:
            message_type, qos, published, pub = created['/custom_heightscan']
            assert message_type is PointCloud2
            assert qos is qos_profile_sensor_data
            assert sim.terrain_scan_pub is pub
        else:
            assert '/custom_heightscan' not in created
            assert sim.terrain_scan_pub is None
        sim.step()
        sim.post_step()
        if ros and enabled:
            assert not published
        sim.step()
        sim.post_step()
        if ros and enabled:
            assert len(published) == 1
            assert published[0].header.frame_id == 'pelvis'
            stamp = published[0].header.stamp
            assert stamp.sec * 1_000_000_000 + stamp.nanosec == round(sim.data.time * 1e9)
            assert (published[0].height, published[0].width) == (3, 2)
        sim.step()
        sim.post_step()
        if ros and enabled:
            assert len(published) == 1
    finally:
        sim.close()
