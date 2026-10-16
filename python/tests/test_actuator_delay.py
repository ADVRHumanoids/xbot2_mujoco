from types import SimpleNamespace

import mujoco
import pytest

from xbot2_py_mujoco import mj_xbot2_bridge


def make_bridge(monkeypatch, delay=0.0):
    model = mujoco.MjModel.from_xml_string('''<mujoco>
      <worldbody><body><joint name="hinge" type="hinge"/>
        <geom type="sphere" size=".1" mass="1"/></body></worldbody>
      <actuator><position name="hinge" joint="hinge" kp="10" kv="2"/></actuator>
    </mujoco>''')
    data = mujoco.MjData(model)
    data.actuator('hinge').ctrl = 0.25
    commands = SimpleNamespace(q=[0.0], dq=[0.0], tau=[0.0], k=[10.0], d=[2.0])
    server = SimpleNamespace(command=SimpleNamespace(joints=commands, stamp_rx_feedback=0),
                             fresh=False, states=[])
    server.receive = lambda: server.fresh
    server.send_state = lambda state: server.states.append(state)
    monkeypatch.setattr(mj_xbot2_bridge, 'BridgeServer', lambda *args: server)
    bridge = mj_xbot2_bridge.MjXbot2Bridge('test', model, data, actuator_delay_s=delay)
    return bridge, server, commands


def set_command(commands, value):
    commands.q[0] = value
    commands.dq[0] = value + 1
    commands.tau[0] = value + 2
    commands.k[0] = value + 3
    commands.d[0] = value + 4


def test_zero_delay_applies_complete_command_on_receive(monkeypatch):
    bridge, server, commands = make_bridge(monkeypatch)
    server.fresh = True
    set_command(commands, 1.0)
    assert bridge.receive()
    assert bridge.data.actuator('hinge').ctrl[0] == 1.0
    assert bridge.model.actuator('hinge').gainprm[0] == 4.0
    assert bridge.model.actuator('hinge').biasprm[:3].tolist() == [13.0, -4.0, -5.0]
    bridge.send_state()
    assert server.states[-1].joints.qref == [1.0]
    assert server.states[-1].joints.vref == (2.0,)
    assert server.states[-1].joints.tauref == (3.0,)


def test_initial_position_target_is_sent_to_connecting_client(monkeypatch):
    bridge, server, commands = make_bridge(monkeypatch)
    # SimulatorWrapper applies q_init after constructing the bridge.
    bridge.data.joint('hinge').qpos[0] = -0.4
    bridge.data.actuator('hinge').ctrl = -0.4
    bridge.send_state()
    assert server.states[-1].joints.qref == [-0.4]

    # The client initializes its outgoing position target from that feedback.
    commands.q[0] = server.states[-1].joints.qref[0]
    server.fresh = True
    bridge.receive()
    assert bridge.data.actuator('hinge').ctrl[0] == -0.4


def test_delay_copies_shared_memory_and_holds_initial_values(monkeypatch):
    bridge, server, commands = make_bridge(monkeypatch, delay=0.02)
    server.fresh = True
    set_command(commands, 1.0)
    bridge.receive()
    set_command(commands, 99.0)  # The shared-memory view changes after receive.
    assert bridge.data.actuator('hinge').ctrl[0] == 0.25
    assert bridge.model.actuator('hinge').gainprm[0] == 10.0
    bridge.send_state()
    assert server.states[-1].joints.qref == [0.25]
    server.fresh = False
    bridge.data.time = 0.019
    bridge.receive()
    assert bridge.data.actuator('hinge').ctrl[0] == 0.25
    bridge.data.time = 0.02
    bridge.receive()
    assert bridge.data.actuator('hinge').ctrl[0] == 1.0
    assert bridge.model.actuator('hinge').gainprm[0] == 4.0


def test_live_change_preserves_deadlines_and_prevents_old_command_replay(monkeypatch):
    bridge, server, commands = make_bridge(monkeypatch, delay=0.1)
    server.fresh = True
    set_command(commands, 1.0)
    bridge.receive()  # Due at 0.1.
    bridge.data.time = 0.01
    bridge.set_actuator_delay(0.01)
    set_command(commands, 2.0)
    bridge.receive()  # Due at 0.02.
    server.fresh = False
    bridge.data.time = 0.02
    bridge.receive()
    assert bridge.data.actuator('hinge').ctrl[0] == 2.0
    bridge.data.time = 0.1
    bridge.receive()
    assert bridge.data.actuator('hinge').ctrl[0] == 2.0
    assert not bridge._pending_commands


def test_latest_due_command_wins_and_live_increase_affects_future_only(monkeypatch):
    bridge, server, commands = make_bridge(monkeypatch, delay=0.02)
    server.fresh = True
    set_command(commands, 1.0)
    bridge.receive()
    bridge.data.time = 0.01
    set_command(commands, 2.0)
    bridge.receive()
    bridge.set_actuator_delay(0.1)
    bridge.data.time = 0.02
    set_command(commands, 3.0)
    bridge.receive()
    assert bridge.data.actuator('hinge').ctrl[0] == 1.0
    server.fresh = False
    bridge.data.time = 0.03
    bridge.receive()
    assert bridge.data.actuator('hinge').ctrl[0] == 2.0
    bridge.data.time = 0.12
    bridge.receive()
    assert bridge.data.actuator('hinge').ctrl[0] == 3.0


def test_multiple_due_commands_apply_only_the_newest(monkeypatch):
    bridge, server, commands = make_bridge(monkeypatch, delay=0.02)
    server.fresh = True
    for value, received_at in ((1.0, 0.0), (2.0, 0.005), (3.0, 0.01)):
        bridge.data.time = received_at
        set_command(commands, value)
        bridge.receive()
    server.fresh = False
    bridge.data.time = 0.04
    bridge.receive()
    assert bridge.data.actuator('hinge').ctrl[0] == 3.0
    assert not bridge._pending_commands


@pytest.mark.parametrize('value', [-1, float('nan'), float('inf'), True])
def test_invalid_bridge_delay(monkeypatch, value):
    with pytest.raises(ValueError, match='finite and non-negative'):
        make_bridge(monkeypatch, delay=value)
