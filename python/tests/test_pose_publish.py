import time
from types import SimpleNamespace
from unittest.mock import Mock

import mujoco
import numpy as np
import pytest

pytest.importorskip('simpleposes')
from simpleposes import PoseSubscriber

from xbot2_py_mujoco import simulator_wrapper as wrapper
from xbot2_py_mujoco.pose_publish import BodyPosePublisher
from xbot2_py_mujoco.simulate import _pose_links, parse_args

S = np.sqrt(0.5)
# base: at (1, 2, 3), rotated 90 deg about z; ee: 0.5 along base's x, rotated 90 deg about base's x
CHAIN_XML = f'''<mujoco><worldbody>
  <body name="base" pos="1 2 3" quat="{S} 0 0 {S}"><geom type="box" size=".1 .1 .1"/>
    <body name="ee" pos=".5 0 0" quat="{S} {S} 0 0"><geom type="sphere" size=".05"/></body>
  </body></worldbody></mujoco>'''


def test_poses_are_relative_to_their_reference_body():
    model = mujoco.MjModel.from_xml_string(CHAIN_XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ee_quat = np.empty(4)
    mujoco.mju_mulQuat(ee_quat, [S, 0, 0, S], [S, S, 0, 0])
    inv_quat = np.empty(4)
    mujoco.mju_negQuat(inv_quat, ee_quat)
    inv_pos = np.empty(3)
    mujoco.mju_rotVecQuat(inv_pos, -np.array([1.0, 2.5, 3.0]), inv_quat)
    publisher = BodyPosePublisher(model, {'ee': 'base', 'base': 'world', 'world': 'ee'}, 'tcp://127.0.0.1:*')
    try:
        poses = publisher.poses(data)
    finally:
        publisher.close()
    np.testing.assert_allclose(poses[0], [.5, 0, 0, S, 0, 0, S], atol=1e-12)
    np.testing.assert_allclose(poses[1], [1, 2, 3, 0, 0, S, S], atol=1e-12)
    np.testing.assert_allclose(poses[2], [*inv_pos, *inv_quat[1:], inv_quat[0]], atol=1e-12)


@pytest.mark.parametrize('links', [{'nope': 'world'}, {'ee': 'nope'}])
def test_rejects_unknown_bodies(links):
    model = mujoco.MjModel.from_xml_string(CHAIN_XML)
    with pytest.raises(ValueError, match="'nope' is not a body"):
        BodyPosePublisher(model, links, 'tcp://127.0.0.1:*')


def test_cli():
    args = parse_args(['--pose-link', 'ee', 'base', '--pose-link', 'base', 'world', '--pose-decimation', '5'])
    assert _pose_links(args.pose_links) == {'ee': 'base', 'base': 'world'}
    assert args.pose_endpoint == 'tcp://*:5800'
    assert args.pose_decimation == 5
    defaults = parse_args([])
    assert _pose_links(defaults.pose_links) == {} and defaults.pose_decimation == 1
    with pytest.raises(ValueError, match='more than once'):
        _pose_links([['ee', 'base'], ['ee', 'world']])
    with pytest.raises(SystemExit):
        parse_args(['--pose-decimation', '0'])


def test_rejects_invalid_decimation(monkeypatch):
    monkeypatch.setattr(wrapper, 'MjXbot2Bridge', lambda **kwargs: Mock())
    model = mujoco.MjModel.from_xml_string(CHAIN_XML)
    with pytest.raises(ValueError, match='pose_decimation'):
        wrapper.SimulatorWrapper(model, ros=False, viewer=False, pose_decimation=0)


def test_simulator_stamps_poses_with_the_state_they_describe(monkeypatch):
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="robot" pos="0 0 1"><freejoint/><geom type="box" size=".1 .1 .1"/></body>
    </worldbody></mujoco>''')
    bridge = SimpleNamespace(receive=Mock(), send_state=Mock(), close=Mock())
    monkeypatch.setattr(wrapper, 'MjXbot2Bridge', lambda **kwargs: bridge)
    sim = wrapper.SimulatorWrapper(model, ros=False, viewer=False, target_rtf=1000.0,
                                   pose_links={'robot': 'world'}, pose_endpoint='tcp://127.0.0.1:*')
    heights = {} # height of the falling body after each step, by the simulation time after the step
    def run_step():
        sim.pre_step()
        sim.step()
        sim.post_step()
        heights[sim.data.time] = sim.data.qpos[2]

    try:
        with PoseSubscriber(sim.pose_publisher.endpoint, conflate=False) as sub:
            deadline = time.monotonic() + 5.0
            while sub.recv(timeout_s=0.01) is None: # PUB drops messages until the subscriber has connected
                assert time.monotonic() < deadline, 'subscriber did not connect'
                run_step()
            for _ in range(50):
                run_step()
            received = []
            while (snapshot := sub.recv(timeout_s=0.2)) is not None:
                received.append(snapshot)
    finally:
        sim.close()
    assert len(received) == 50
    for snapshot in received:
        assert snapshot.links == ['robot'] and snapshot.frames == ['world']
        assert snapshot.pose('robot')[2] == pytest.approx(heights[snapshot.t], abs=1e-12)
    assert received[-1].pose('robot')[2] < received[0].pose('robot')[2]
