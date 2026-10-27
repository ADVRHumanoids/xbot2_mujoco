from pathlib import Path
from types import SimpleNamespace

from lxml import etree
import mujoco
import numpy as np
import pytest

from xbot2_py_mujoco.mjcf_tools import MjcfGenerator
from xbot2_py_mujoco.odometry import Odometry, OdometryCfg


def scene():
    """A free body moving at 1 m/s along x and spinning at 2 rad/s about z, no gravity."""
    model = mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0" timestep="0.001"/>
      <worldbody><body name="base"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
      </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    data.qvel[:] = (1, 0, 0, 0, 0, 2)
    return model, data


def run(model, data, cfg, steps):
    odometry = Odometry(model, cfg)
    samples = []
    for _ in range(steps):
        mujoco.mj_step(model, data)
        sample = odometry.step(data)
        if sample is not None:
            samples.append(sample)
    return samples


def generator(tmp_path, xml):
    gen = MjcfGenerator('odometry', (Path(__file__).parent / 'resources/nice_robot.urdf').read_text(),
                        output_dir=str(tmp_path / 'robot'))
    body = gen.mj_xml_tree.find('worldbody/body').get('name')
    gen.merge_xml(f'<mujoco>{xml.format(body=body)}</mujoco>')
    return gen, body


def test_pose_at_stamp_and_body_frame_twist():
    model, data = scene()
    samples = run(model, data, OdometryCfg('base', rate=100), 25)
    np.testing.assert_allclose([s.time for s in samples], (0, .01, .02), atol=1e-9)
    for s in samples:
        np.testing.assert_allclose(s.position, (s.time, 0, 0), atol=1e-9)
        yaw = 2 * s.time
        np.testing.assert_allclose(s.orientation, (np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)), atol=1e-9)
        # The world's 1 m/s along x, in the yawed body.
        np.testing.assert_allclose(s.linear_velocity, (np.cos(yaw), -np.sin(yaw), 0), atol=1e-9)
        np.testing.assert_allclose(s.angular_velocity, (0, 0, 2), atol=1e-9)


def test_xml_parse(tmp_path):
    gen, body = generator(tmp_path, '<odometry body="{body}" topic="/sim/ground_truth" frame_id="sim_world"/>')
    assert '<odometry' not in gen.generate_mjcf_string()
    assert gen.odometries == [OdometryCfg(body, '/sim/ground_truth', 'sim_world')]
    assert gen.odometries[0].child_frame_id == body


@pytest.mark.parametrize('xml, error', [
    ('<odometry body="missing"/>', 'Unknown odometry body'),
    ('<odometry body="{body}"/><odometry body="{body}"/>', 'Duplicate odometry topic'),
])
def test_unknown_names_and_duplicates(tmp_path, xml, error):
    with pytest.raises(ValueError, match=error):
        generator(tmp_path, xml)[0].generate_mjcf_string()


@pytest.mark.parametrize('attributes', [
    '', 'body="b" frame_id="/w"', 'body="b" child_frame_id="c d"', 'body="b" rate="nan"',
    'body="b" topic="/"', 'body="b" extra="1"',
])
def test_invalid_xml_settings(attributes):
    with pytest.raises(ValueError):
        OdometryCfg.from_xml(etree.fromstring(f'<odometry {attributes}/>'))


def test_simulator_samples(monkeypatch):
    from xbot2_py_mujoco import simulator_wrapper

    monkeypatch.setattr(simulator_wrapper, 'MjXbot2Bridge', lambda **kwargs:
                        SimpleNamespace(send_state=lambda: None, close=lambda: None))
    model, _ = scene()
    sim = simulator_wrapper.SimulatorWrapper(model, viewer=False, ros=False, target_rtf=1e9,
                                             odometries=[OdometryCfg('base', topic='/gt')])
    for _ in range(6):
        sim.step()
        sim.post_step()
    assert sim.odometry_samples['/gt'].time == pytest.approx(0.005)
    sim.close()


def test_ros_message():
    pytest.importorskip('nav_msgs')
    from xbot2_py_mujoco.odometry_ros import odometry_message
    model, data = scene()
    cfg = OdometryCfg('base', frame_id='sim_world', child_frame_id='pelvis')
    sample = run(model, data, cfg, 6)[-1]
    msg = odometry_message(sample, cfg)
    assert (msg.header.frame_id, msg.child_frame_id) == ('sim_world', 'pelvis')
    assert msg.pose.pose.position.x == pytest.approx(sample.position[0])
    assert msg.twist.twist.angular.z == pytest.approx(2)
