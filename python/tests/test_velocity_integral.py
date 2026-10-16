from types import SimpleNamespace

import mujoco
import pytest

from xbot2_py_mujoco import mj_xbot2_bridge
from xbot2_py_mujoco.simulator_wrapper import SimulatorWrapper


@pytest.fixture
def bridge(monkeypatch):
    model = mujoco.MjModel.from_xml_string('''<mujoco>
      <option timestep="0.01"/>
      <worldbody><body><joint name="hinge" range="-2 2"/>
        <geom type="sphere" size=".1" mass="1"/></body></worldbody>
      <actuator><intvelocity name="hinge" joint="hinge" kp="4" kv="2"
                              inheritrange="1"/></actuator>
    </mujoco>''')
    data = mujoco.MjData(model)
    data.joint('hinge').qpos[0] = 0.4
    joints = SimpleNamespace(q=[0.7], dq=[0.5], tau=[0.3], k=[4.0], d=[2.0])
    server = SimpleNamespace(
        command=SimpleNamespace(joints=joints, stamp_rx_feedback=0),
        receive=lambda: True,
        states=[],
    )
    server.send_state = lambda state: server.states.append(state)
    monkeypatch.setattr(mj_xbot2_bridge, 'BridgeServer', lambda *args: server)
    return mj_xbot2_bridge.MjXbot2Bridge('test', model, data)


def test_velocity_pi_uses_mujoco_integrator(bridge):
    model, data = bridge.model, bridge.data
    actuator = model.actuator('hinge')
    assert data.act[int(actuator.actadr[0])] == pytest.approx(0.4)

    bridge.receive()
    assert data.actuator('hinge').ctrl[0] == pytest.approx(0.5)
    assert bridge.qref == [0.7]

    mujoco.mj_forward(model, data)
    assert data.actuator_force[0] == pytest.approx(2 * 0.5 + 0.3)

    data.act[int(actuator.actadr[0])] = 0.6
    data.joint('hinge').qvel[0] = 0.2
    mujoco.mj_forward(model, data)
    assert data.actuator_force[0] == pytest.approx(4 * (0.6 - 0.4) + 2 * (0.5 - 0.2) + 0.3)


def test_velocity_integral_reset_aligns_target(bridge):
    data = bridge.data
    data.joint('hinge').qpos[0] = -0.3
    data.act[0] = 1.0
    data.actuator('hinge').ctrl = 0.7
    bridge.reset_velocity_integrators()
    assert data.act[0] == pytest.approx(-0.3)
    assert data.actuator('hinge').ctrl[0] == 0.0


def test_simulator_initial_position_resets_integrator(bridge):
    sim = SimulatorWrapper.__new__(SimulatorWrapper)
    sim.model = bridge.model
    sim.data = bridge.data
    sim.xbot2_bridge = bridge
    sim.q_init = {'hinge': -0.2}
    bridge.data.act[0] = 0.9
    sim._set_initial_position(bridge.data)
    assert bridge.data.joint('hinge').qpos[0] == pytest.approx(-0.2)
    assert bridge.data.act[0] == pytest.approx(-0.2)
    bridge.send_state()
    assert bridge.bridge_server.states[-1].joints.qref == [-0.2]


def test_velocity_feedback_noise_only_changes_sent_state(bridge):
    bridge.data.joint('hinge').qvel[0] = 0.3
    bridge._noise_rng = SimpleNamespace(normal=lambda mean, scale: [0.2])
    bridge.set_velocity_feedback_noise(['hinge'], 0.1)
    bridge.send_state()
    assert bridge.bridge_server.states[-1].joints.dq == pytest.approx([0.5])
    assert bridge.data.joint('hinge').qvel[0] == pytest.approx(0.3)
    bridge.reset_velocity_feedback_noise()
    bridge.send_state()
    assert bridge.bridge_server.states[-1].joints.dq == pytest.approx([0.3])
