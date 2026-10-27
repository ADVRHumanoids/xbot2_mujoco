from pathlib import Path
from types import SimpleNamespace

from lxml import etree
import mujoco
import numpy as np
import pytest

from xbot2_py_mujoco.imu import Imu, ImuCfg
from xbot2_py_mujoco.mjcf_tools import MjcfGenerator


def scene():
    """A free body moving at 1 m/s along x and spinning at 2 rad/s about z, no gravity; the IMU
    site is upside down."""
    model = mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0" timestep="0.001"/>
      <worldbody><body name="base"><freejoint/><geom type="sphere" size=".1" mass="1"/>
        <site name="imu" quat="0 1 0 0"/></body></worldbody>
      <sensor><accelerometer name="acc" site="imu"/><gyro name="gyro" site="imu"/></sensor>
    </mujoco>''')
    data = mujoco.MjData(model)
    data.qvel[:] = (1, 0, 0, 0, 0, 2)
    return model, data


def run(model, data, cfg, steps):
    imu = Imu(model, cfg)
    samples = []
    for _ in range(steps):
        mujoco.mj_step(model, data)
        sample = imu.step(data)
        if sample is not None:
            samples.append(sample)
    return samples


def generator(tmp_path, xml):
    gen = MjcfGenerator('imu', (Path(__file__).parent / 'resources/nice_robot.urdf').read_text(),
                        output_dir=str(tmp_path / 'robot'))
    body = gen.mj_xml_tree.find('worldbody/body').get('name')
    gen.merge_xml(f'''<mujoco>
      <reference element="body" name="{body}"><site name="imu"/></reference>
      <sensor><accelerometer name="acc" site="imu"/><gyro name="gyro" site="imu"/></sensor>
      {xml}</mujoco>''')
    return gen


def test_rate_stamps_and_site_frame():
    model, data = scene()
    samples = run(model, data, ImuCfg('imu', 'acc', 'gyro', rate=200), 20)
    np.testing.assert_allclose([s.time for s in samples], (0, .005, .01, .015), atol=1e-9)
    # The body's +z spin reads as -z in the upside down site.
    np.testing.assert_allclose(samples[-1].angular_velocity, (0, 0, -2), atol=1e-9)
    # Constant velocity, site at the spin axis: no acceleration.
    np.testing.assert_allclose(samples[-1].linear_acceleration, 0, atol=1e-6)
    yaw = 2 * samples[-1].time
    expected = np.empty(4)
    mujoco.mju_mulQuat(expected, np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]),
                       np.array([0., 1, 0, 0]))
    np.testing.assert_allclose(np.abs(samples[-1].orientation @ expected), 1, atol=1e-9)


def test_schedule_tolerates_accumulated_sim_time_error():
    model, data = scene()
    imu = Imu(model, ImuCfg('imu', 'acc', 'gyro', rate=200))
    due = []
    for k in range(20):
        # Sim time drifts below the 1 ms grid, as a long sum of steps does.
        data.time = 80 + k * 0.001 - (2e-10 if k else 0) + model.opt.timestep
        mujoco.mj_forward(model, data)
        if imu.step(data) is not None:
            due.append(k)
    assert due == [0, 5, 10, 15]


def test_xml_parse(tmp_path):
    gen = generator(tmp_path, '''<imu site="imu" accelerometer="acc" gyro="gyro" topic="/vectornav/imu"
                                       frame_id="imu_link" rate="400"/>''')
    assert '<imu' not in gen.generate_mjcf_string()
    assert gen.imus == [ImuCfg('imu', 'acc', 'gyro', '/vectornav/imu', 'imu_link', 400)]


@pytest.mark.parametrize('xml, error', [
    ('<imu site="missing" accelerometer="acc" gyro="gyro"/>', 'Unknown IMU site'),
    ('<imu site="imu" accelerometer="missing" gyro="gyro"/>', 'IMU sensor missing'),
    ('<imu site="imu" accelerometer="acc" gyro="gyro"/><imu site="imu" accelerometer="acc" gyro="gyro"/>',
     'Duplicate IMU topic'),
])
def test_unknown_names_and_duplicates(tmp_path, xml, error):
    with pytest.raises(ValueError, match=error):
        generator(tmp_path, xml).generate_mjcf_string()


@pytest.mark.parametrize('attributes', [
    'accelerometer="a" gyro="g"', 'site="s" accelerometer="a" gyro="g" rate="0"',
    'site="s" accelerometer="a" gyro="g" topic="/"', 'site="s" accelerometer="a" gyro="g" extra="1"',
    'site="s" accelerometer="a" gyro="g" frame_id="/f"',
])
def test_invalid_xml_settings(attributes):
    with pytest.raises(ValueError):
        ImuCfg.from_xml(etree.fromstring(f'<imu {attributes}/>'))


def test_simulator_samples(monkeypatch):
    from xbot2_py_mujoco import simulator_wrapper

    monkeypatch.setattr(simulator_wrapper, 'MjXbot2Bridge', lambda **kwargs:
                        SimpleNamespace(send_state=lambda: None, close=lambda: None))
    model, _ = scene()
    sim = simulator_wrapper.SimulatorWrapper(model, viewer=False, ros=False, target_rtf=1e9,
                                             imus=[ImuCfg('imu', 'acc', 'gyro', topic='/imu')])
    for _ in range(6):
        sim.step()
        sim.post_step()
    assert sim.imu_samples['/imu'].time == pytest.approx(0.005)
    sim.close()


def test_ros_message():
    pytest.importorskip('sensor_msgs')
    from xbot2_py_mujoco.imu_ros import imu_message
    model, data = scene()
    cfg = ImuCfg('imu', 'acc', 'gyro', frame_id='imu_link')
    msg = imu_message(run(model, data, cfg, 6)[-1], cfg)
    assert msg.header.frame_id == 'imu_link' and msg.header.stamp.nanosec == 5_000_000
    assert msg.angular_velocity.z == pytest.approx(-2)
