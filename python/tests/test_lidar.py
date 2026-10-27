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


@pytest.mark.parametrize('backend', backends())
def test_ranges_and_layout(backend):
    model, data = scene()
    mujoco.mj_forward(model, data)
    # Columns clockwise from +x: 0, -90, 180, 90 deg; rings at 0 and 5 deg.
    lidar = Lidar(model, LidarCfg('lidar', (0.0, 5.0), horizontal_samples=4, backend=backend))
    scan = lidar.scan(data)
    assert scan.points.shape == (8, 3)
    np.testing.assert_array_equal(scan.rings, [0, 1] * 4)
    np.testing.assert_allclose(scan.points[0], (4.9, 0, 0), atol=1e-4)
    np.testing.assert_allclose(scan.points[1], (4.9, 0, 4.9 * np.tan(np.radians(5))), atol=1e-4)
    np.testing.assert_array_equal(scan.points[2:], 0)


def test_start_azimuth_rotates_columns():
    model, data = scene()
    mujoco.mj_forward(model, data)
    lidar = Lidar(model, LidarCfg('lidar', (0.0,), horizontal_samples=4, start_azimuth_deg=90))
    points = lidar.scan(data).points
    np.testing.assert_array_equal(points[0], 0)
    np.testing.assert_allclose(points[1], (4.9, 0, 0), atol=1e-4)


def test_excluded_groups_and_ranges_are_misses():
    model, data = scene(wall_group=3)
    mujoco.mj_forward(model, data)
    np.testing.assert_array_equal(
        Lidar(model, LidarCfg('lidar', (0.0,), horizontal_samples=1)).scan(data).points, 0)
    model, data = scene()
    mujoco.mj_forward(model, data)
    for cfg in (LidarCfg('lidar', (0.0,), horizontal_samples=1, cutoff=4.0),
                LidarCfg('lidar', (0.0,), horizontal_samples=1, min_range=4.95)):
        np.testing.assert_array_equal(Lidar(model, cfg).scan(data).points, 0)


@pytest.mark.parametrize('backend', backends())
def test_excluded_body_is_transparent(backend):
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="mount"><site name="lidar"/><geom type="sphere" pos="1 0 0" size=".2"/></body>
      <geom type="box" pos="5 0 0" size=".1 1 1"/>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    cfg = LidarCfg('lidar', (0.0,), horizontal_samples=1, backend=backend)
    np.testing.assert_allclose(Lidar(model, cfg).scan(data).points[0], (.8, 0, 0), atol=1e-4)
    cfg = LidarCfg('lidar', (0.0,), horizontal_samples=1, backend=backend, exclude_body='mount')
    np.testing.assert_allclose(Lidar(model, cfg).scan(data).points[0], (4.9, 0, 0), atol=1e-4)
    with pytest.raises(ValueError, match='Unknown lidar exclude_body'):
        Lidar(model, LidarCfg('lidar', (0.0,), exclude_body='missing'))


def test_due_follows_rate_and_time_reset():
    model, _ = scene()
    lidar = Lidar(model, LidarCfg('lidar', (0.0,), rate=10))
    assert [lidar.due(t) for t in (0.0, 0.05, 0.1, 0.15, 0.35)] == [True, False, True, False, True]
    assert not lidar.due(0.38)
    assert lidar.due(0.4)
    assert lidar.due(0.0)


def test_simulator_scans_at_rate(monkeypatch):
    from xbot2_py_mujoco import simulator_wrapper

    monkeypatch.setattr(simulator_wrapper, 'MjXbot2Bridge', lambda **kwargs:
                        SimpleNamespace(send_state=lambda: None, close=lambda: None))
    model, _ = scene()
    model.opt.timestep = 0.05
    sim = simulator_wrapper.SimulatorWrapper(
        model, viewer=False, ros=False, target_rtf=1e9,
        lidars=[LidarCfg('lidar', (0.0,), rate=10, horizontal_samples=1)],
    )
    times = []
    for _ in range(4):
        sim.step()
        sim.post_step()
        times.append(sim.lidar_scans['lidar'].time)
    np.testing.assert_allclose(times, [0.05, 0.05, 0.15, 0.15])
    np.testing.assert_allclose(sim.lidar_scans['lidar'].points[0], (4.9, 0, 0), atol=1e-4)
    sim.close()


def test_xml_defaults_and_repeated_finalization(tmp_path):
    generator = MjcfGenerator('lidar', (Path(__file__).parent / 'resources/nice_robot.urdf').read_text(),
                              output_dir=str(tmp_path / 'robot'))
    body = generator.mj_xml_tree.find('worldbody/body').get('name')
    generator.merge_xml(f'''<mujoco>
      <reference element="body" name="{body}"><site name="lidar"/></reference>
      <lidar site="lidar" elevations_deg="-1 0 1" rate="20" topic="/points" backend="cpu"
             horizontal_samples="10" start_azimuth_deg="96" geom_groups="0 2 3"/>
    </mujoco>''')
    xml = generator.generate_mjcf_string()
    assert '<lidar' not in xml
    assert generator.lidars == [LidarCfg('lidar', (-1.0, 0.0, 1.0), rate=20, topic='/points',
                                         horizontal_samples=10, start_azimuth_deg=96,
                                         geom_groups=(0, 2, 3))]
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
    'site="s" elevations_deg="0" extra="1"',
])
def test_invalid_xml_settings(attributes):
    with pytest.raises(ValueError):
        LidarCfg.from_xml(etree.fromstring(f'<lidar {attributes}/>'))
