from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from xbot2_py_mujoco import mj_xbot2_bridge


def test_static_pose_reports_the_torque_holding_it_against_gravity(monkeypatch):
    # A 2 kg point 0.5 m out on a horizontal arm, held by a stiff servo.
    model = mujoco.MjModel.from_xml_string('''<mujoco>
      <worldbody><body><joint name="hinge" type="hinge" axis="0 1 0"
          damping="0.5" armature="0.01"/>
        <geom type="sphere" size=".02" pos=".5 0 0" mass="2"/></body></worldbody>
      <actuator><position name="hinge" joint="hinge" kp="2000" kv="20"/></actuator>
    </mujoco>''')
    data = mujoco.MjData(model)
    commands = SimpleNamespace(q=[0.0], dq=[0.0], tau=[0.0], k=[2000.0], d=[20.0])
    server = SimpleNamespace(command=SimpleNamespace(joints=commands, stamp_rx_feedback=0),
                             fresh=False, states=[])
    server.receive = lambda: server.fresh
    server.send_state = lambda state: server.states.append(state)
    monkeypatch.setattr(mj_xbot2_bridge, 'BridgeServer', lambda *args: server)
    bridge = mj_xbot2_bridge.MjXbot2Bridge('test', model, data)
    for _ in range(4000):
        mujoco.mj_step(model, data)
    assert abs(data.qvel[0]) < 1e-6

    bridge.send_state()
    # Gravity's torque about +y is m g l; the joint torque sensor reads the
    # actuator's opposite torque, not the near-zero net smooth force.
    gravity = 2.0 * 9.81 * 0.5 * np.cos(data.qpos[0])
    assert server.states[-1].joints.tau[0] == pytest.approx(-gravity, rel=1e-3)
