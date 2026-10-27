import importlib.util
from pathlib import Path
from types import SimpleNamespace

from lxml import etree
import mujoco
import numpy as np
import pytest

pytest.importorskip('mujoco_lidar')

from xbot2_py_mujoco.lidar import Lidar, LidarCfg
from xbot2_py_mujoco.mjcf_tools import MjcfGenerator


def scene(wall_group=0):
    """Site at the origin, a wall whose face is at x = 4.9 (y in [-1, 1])."""
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
      <site name="lidar"/>
      <geom name="wall" type="box" pos="5 0 0" size=".1 1 1" group="{wall_group}"/>
    </worldbody></mujoco>''')
    return model, mujoco.MjData(model)


def backends():
    available = ['cpu']
    available += [b for b, m in (('taichi', 'taichi'), ('warp', 'warp')) if importlib.util.find_spec(m)]
    return available


def full_scan(model, data, **kwargs):
    """One sweep traced at once (slices=1) at the current pose."""
    mujoco.mj_forward(model, data)
    data.time = model.opt.timestep
    return Lidar(model, LidarCfg('lidar', **{'slices': 1, **kwargs})).step(data)


@pytest.mark.parametrize('backend', backends())
def test_ranges_and_layout(backend):
    model, data = scene()
    # Columns clockwise from +x: 0, -90, 180, 90 deg; rings at 0 and 5 deg.
    scan = full_scan(model, data, elevations_deg=(0.0, 5.0), horizontal_samples=4, backend=backend)
    assert scan.points.shape == (8, 3)
    np.testing.assert_array_equal(scan.rings, [0, 1] * 4)
    np.testing.assert_allclose(scan.points[0], (4.9, 0, 0), atol=1e-4)
    np.testing.assert_allclose(scan.points[1], (4.9, 0, 4.9 * np.tan(np.radians(5))), atol=1e-4)
    np.testing.assert_array_equal(scan.points[2:], 0)


def test_returns_repeat_each_column():
    model, data = scene()
    # Columns at 0, -90, 180, 90 deg, two returns each: the wall (+x) in the first pair only.
    scan = full_scan(model, data, elevations_deg=(0.0, 5.0), horizontal_samples=4, returns=2)
    points = scan.points.reshape(4, 2, 2, 3)  # columns, returns, rings
    np.testing.assert_array_equal(points[:, 0], points[:, 1])
    np.testing.assert_allclose(points[0, 0, 0], (4.9, 0, 0), atol=1e-4)
    np.testing.assert_array_equal(points[1:], 0)
    np.testing.assert_array_equal(scan.rings, [0, 1] * 8)
    assert len(scan.times) == 16


def test_cloud_is_not_dense():
    pytest.importorskip('sensor_msgs')
    from xbot2_py_mujoco.lidar_ros import pointcloud_from_scan
    model, data = scene()
    msg = pointcloud_from_scan(full_scan(model, data, elevations_deg=(0.0,), horizontal_samples=4,
                                         returns=2), 'lidar')
    assert (msg.width, msg.is_dense) == (8, False)


def test_start_azimuth_rotates_columns():
    model, data = scene()
    points = full_scan(model, data, elevations_deg=(0.0,), horizontal_samples=4,
                       start_azimuth_deg=90).points
    np.testing.assert_array_equal(points[0], 0)
    np.testing.assert_allclose(points[1], (4.9, 0, 0), atol=1e-4)


def test_excluded_groups_and_ranges_are_misses():
    model, data = scene(wall_group=3)
    np.testing.assert_array_equal(
        full_scan(model, data, elevations_deg=(0.0,), horizontal_samples=1).points, 0)
    model, data = scene()
    for kwargs in ({'cutoff': 4.0}, {'min_range': 4.95}):
        np.testing.assert_array_equal(
            full_scan(model, data, elevations_deg=(0.0,), horizontal_samples=1, **kwargs).points, 0)


@pytest.mark.parametrize('backend', backends())
def test_excluded_body_is_transparent(backend):
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="mount"><site name="lidar"/><geom type="sphere" pos="1 0 0" size=".2"/></body>
      <geom type="box" pos="5 0 0" size=".1 1 1"/>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    common = {'elevations_deg': (0.0,), 'horizontal_samples': 1, 'backend': backend}
    np.testing.assert_allclose(full_scan(model, data, **common).points[0], (.8, 0, 0), atol=1e-4)
    np.testing.assert_allclose(full_scan(model, data, exclude_body='mount', **common).points[0],
                               (4.9, 0, 0), atol=1e-4)
    with pytest.raises(ValueError, match='Unknown lidar exclude_body'):
        Lidar(model, LidarCfg('lidar', (0.0,), exclude_body='missing'))


@pytest.mark.parametrize('backend', backends())
def test_sweep_slices_see_the_pose_at_their_time(backend):
    """A floor rising at 1 m/s under a lidar looking 45 deg down: each slice sees it at the
    height of its own time."""
    model = mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0" timestep="0.001"/>
      <worldbody><site name="lidar"/>
        <body name="floor" pos="0 0 -2"><joint name="lift" type="slide" axis="0 0 1"/>
          <geom type="box" size="10 10 .1" mass="1"/></body>
      </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    data.qvel[0] = 1.0
    lidar = Lidar(model, LidarCfg('lidar', (-45.0,), rate=10, horizontal_samples=8, slices=4,
                                  backend=backend))
    scans = []
    while not scans:
        mujoco.mj_step(model, data)
        scans = [s for s in [lidar.step(data)] if s is not None]
    scan = scans[0]
    assert scan.time == pytest.approx(0)
    np.testing.assert_allclose(scan.times, np.repeat([0, .025, .05, .075], 2), atol=1e-9)
    np.testing.assert_allclose(scan.points[:, 2], -(1.9 - scan.times), atol=1e-4)


def test_sweep_restarts_when_time_jumps():
    model, data = scene()
    mujoco.mj_forward(model, data)
    lidar = Lidar(model, LidarCfg('lidar', (0.0,), rate=10, horizontal_samples=2, slices=2))

    def at(pose_time):
        data.time = pose_time + model.opt.timestep
        return lidar.step(data)

    assert at(0.0) is None and at(0.03) is None
    assert at(0.06).time == pytest.approx(0)
    assert at(0.02) is None  # back in time: a new sweep from 0.02
    scan = at(0.07)
    assert scan.time == pytest.approx(0.02)
    np.testing.assert_allclose(scan.times, (0.02, 0.07))
    assert at(0.5) is None  # more than a sweep later: a new sweep from 0.5
    assert at(0.55).time == pytest.approx(0.5)


def test_slices_tolerate_accumulated_sim_time_error():
    model, data = scene()
    mujoco.mj_forward(model, data)
    lidar = Lidar(model, LidarCfg('lidar', (0.0,), rate=10, horizontal_samples=2, slices=2))
    scans = []
    for k in range(100):
        # Sim time drifts below the 1 ms grid, as a long sum of steps does.
        data.time = 80 + k * 0.001 - (2e-10 if k else 0) + model.opt.timestep
        scans += [s for s in [lidar.step(data)] if s is not None]
    np.testing.assert_allclose(scans[0].times, (80, 80.05), atol=1e-6)
    assert scans[0].time == scans[0].times[0]


def test_simulator_scans_at_rate(monkeypatch):
    from xbot2_py_mujoco import simulator_wrapper

    monkeypatch.setattr(simulator_wrapper, 'MjXbot2Bridge', lambda **kwargs:
                        SimpleNamespace(send_state=lambda: None, close=lambda: None))
    model, _ = scene()
    model.opt.timestep = 0.05
    sim = simulator_wrapper.SimulatorWrapper(
        model, viewer=False, ros=False, target_rtf=1e9,
        lidars=[LidarCfg('lidar', (0.0,), rate=10, horizontal_samples=1, slices=1)],
    )
    times = []
    for _ in range(4):
        sim.step()
        sim.post_step()
        times.append(sim.lidar_scans['lidar'].time)
    np.testing.assert_allclose(times, [0, 0, 0.1, 0.1])
    np.testing.assert_allclose(sim.lidar_scans['lidar'].points[0], (4.9, 0, 0), atol=1e-4)
    sim.close()


def test_xml_defaults_and_repeated_finalization(tmp_path):
    generator = MjcfGenerator('lidar', (Path(__file__).parent / 'resources/nice_robot.urdf').read_text(),
                              output_dir=str(tmp_path / 'robot'))
    body = generator.mj_xml_tree.find('worldbody/body').get('name')
    generator.merge_xml(f'''<mujoco>
      <reference element="body" name="{body}"><site name="lidar"/></reference>
      <lidar site="lidar" elevations_deg="-1 0 1" rate="20" topic="/points" backend="cpu"
             horizontal_samples="10" start_azimuth_deg="96" geom_groups="0 2 3" slices="5" returns="2"/>
    </mujoco>''')
    xml = generator.generate_mjcf_string()
    assert '<lidar' not in xml
    assert generator.lidars == [LidarCfg('lidar', (-1.0, 0.0, 1.0), rate=20, topic='/points',
                                         horizontal_samples=10, start_azimuth_deg=96,
                                         geom_groups=(0, 2, 3), slices=5, returns=2)]
    assert generator.lidars[0].frame_id == 'lidar'
    assert generator.generate_mjcf_string() == xml
    assert len(generator.lidars) == 1


def test_xml_unknown_site(tmp_path):
    generator = MjcfGenerator('lidar', (Path(__file__).parent / 'resources/nice_robot.urdf').read_text(),
                              output_dir=str(tmp_path / 'robot'))
    generator.merge_xml('<mujoco><lidar site="missing" elevations_deg="0"/></mujoco>')
    with pytest.raises(ValueError, match='Unknown lidar site'):
        generator.generate_mjcf_string()


@pytest.mark.parametrize('attributes', [
    'elevations_deg="0"', 'site="s"', 'site="s" elevations_deg="91"',
    'site="s" elevations_deg="0" rate="0"', 'site="s" elevations_deg="0" cutoff="inf"',
    'site="s" elevations_deg="0" min_range="60"', 'site="s" elevations_deg="0" horizontal_samples="0"',
    'site="s" elevations_deg="0" backend="jax"', 'site="s" elevations_deg="0" geom_groups="6"',
    'site="s" elevations_deg="0" frame_id="/s"', 'site="s" elevations_deg="0" topic="/"',
    'site="s" elevations_deg="0" extra="1"', 'site="s" elevations_deg="0" slices="0"',
    'site="s" elevations_deg="0" returns="0"',
    'site="s" elevations_deg="0" horizontal_samples="10" slices="11"',
])
def test_invalid_xml_settings(attributes):
    with pytest.raises(ValueError):
        LidarCfg.from_xml(etree.fromstring(f'<lidar {attributes}/>'))
