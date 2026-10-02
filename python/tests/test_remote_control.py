import json
import uuid

import mujoco
import numpy as np
import pytest
import zmq

from xbot2_py_mujoco.remote_control import CommandError, RemoteControlServer


@pytest.fixture
def remote_server():
    model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <body name="box">
              <freejoint/>
              <geom name="box_geom_1" type="box" size=".1 .1 .1" friction=".5 .01 .001"/>
              <geom name="box_geom_2" type="sphere" size=".05" friction=".5 .01 .001"/>
              <body name="child" pos="0 0 .2">
                <geom name="child_geom" type="sphere" size=".02" friction=".2 .01 .001"/>
              </body>
            </body>
            <body name="empty"/>
          </worldbody>
        </mujoco>
        """
    )
    data = mujoco.MjData(model)
    endpoint = f"inproc://remote-control-{uuid.uuid4()}"
    server = RemoteControlServer(model, data, endpoint)
    try:
        yield server
    finally:
        server.close()


def test_set_contact_parameters_updates_direct_geoms(remote_server):
    result = remote_server.handle_request({
        "command": "set_contact_parameters",
        "bodies": ["box"],
        "parameters": {
            "friction": [0.9, 0.02, 0.003],
            "solref": [0.01, 1.0],
            "condim": 4,
        },
    })

    assert result["geoms"] == ["box_geom_1", "box_geom_2"]
    for name in result["geoms"]:
        geom_id = remote_server.model.geom(name).id
        assert remote_server.model.geom_friction[geom_id].tolist() == [0.9, 0.02, 0.003]
        assert remote_server.model.geom_solref[geom_id].tolist() == [0.01, 1.0]
        assert remote_server.model.geom_condim[geom_id] == 4

    child_id = remote_server.model.geom("child_geom").id
    assert remote_server.model.geom_friction[child_id, 0] == 0.2


def test_contact_update_is_atomic_on_unknown_body(remote_server):
    geom_id = remote_server.model.geom("box_geom_1").id
    before = remote_server.model.geom_friction[geom_id].copy()

    with pytest.raises(CommandError, match="missing"):
        remote_server.handle_request({
            "command": "set_contact_parameters",
            "bodies": ["box", "missing"],
            "parameters": {"friction": [1.0, 0.0, 0.0]},
        })

    np.testing.assert_array_equal(remote_server.model.geom_friction[geom_id], before)


@pytest.mark.parametrize("parameters", [
    {"friction": [1.0, 2.0]},
    {"friction": [1.0, -1.0, 0.0]},
    {"condim": 2},
    {"unknown": 1},
])
def test_rejects_invalid_contact_parameters(remote_server, parameters):
    with pytest.raises(CommandError):
        remote_server.handle_request({
            "command": "set_contact_parameters",
            "bodies": ["box"],
            "parameters": parameters,
        })


def test_rejects_body_without_direct_geoms(remote_server):
    with pytest.raises(CommandError) as exc_info:
        remote_server.handle_request({
            "command": "set_contact_parameters",
            "bodies": ["empty"],
            "parameters": {"friction": [1.0, 0.0, 0.0]},
        })
    assert exc_info.value.code == "no_geoms"


def test_set_body_mass_and_com(remote_server):
    box_id = remote_server.model.body("box").id

    result = remote_server.handle_request({
        "command": "set_body_properties",
        "bodies": ["box"],
        "properties": {"mass": 12.5, "com": [0.01, -0.02, 0.03]},
    })

    assert result == {
        "bodies": ["box"],
        "properties": {"mass": 12.5, "com": [0.01, -0.02, 0.03]},
    }
    assert remote_server.model.body_mass[box_id] == 12.5
    np.testing.assert_array_equal(
        remote_server.model.body_ipos[box_id], [0.01, -0.02, 0.03]
    )
    assert remote_server.model.body_subtreemass[box_id] >= 12.5


@pytest.mark.parametrize("properties", [
    {"mass": 0.0},
    {"mass": -1.0},
    {"mass": float("nan")},
    {"com": [0.0, 0.0]},
    {"inertia": [1.0, 1.0, 1.0]},
])
def test_rejects_invalid_body_properties(remote_server, properties):
    with pytest.raises(CommandError):
        remote_server.handle_request({
            "command": "set_body_properties",
            "bodies": ["box"],
            "properties": properties,
        })


def test_body_update_is_atomic_on_unknown_body(remote_server):
    box_id = remote_server.model.body("box").id
    mass_before = float(remote_server.model.body_mass[box_id])

    with pytest.raises(CommandError, match="missing"):
        remote_server.handle_request({
            "command": "set_body_properties",
            "bodies": ["box", "missing"],
            "properties": {"mass": 2.0},
        })

    assert remote_server.model.body_mass[box_id] == mass_before


def test_wrenches_add_and_expire_in_simulation_time(remote_server):
    request = {
        "command": "apply_wrench",
        "body": "box",
        "force": [1.0, 2.0, 3.0],
        "torque": [4.0, 5.0, 6.0],
        "duration": 0.2,
    }
    result = remote_server.handle_request(request)
    remote_server.handle_request({**request, "force": [10.0, 0.0, 0.0], "duration": 0.1})

    body_id = remote_server.model.body("box").id
    remote_server.apply_wrenches()
    np.testing.assert_array_equal(
        remote_server.data.xfrc_applied[body_id], [11.0, 2.0, 3.0, 8.0, 10.0, 12.0]
    )
    assert result["expires_at"] == pytest.approx(0.2)

    remote_server.data.time = 0.1
    remote_server.apply_wrenches()
    np.testing.assert_array_equal(
        remote_server.data.xfrc_applied[body_id], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    )

    remote_server.data.time = 0.2
    remote_server.apply_wrenches()
    np.testing.assert_array_equal(remote_server.data.xfrc_applied[body_id], np.zeros(6))


def test_restore_resets_model_properties_and_clears_forces(remote_server):
    box_id = remote_server.model.body("box").id
    geom_id = remote_server.model.geom("box_geom_1").id
    original_mass = float(remote_server.model.body_mass[box_id])
    original_com = remote_server.model.body_ipos[box_id].copy()
    original_friction = remote_server.model.geom_friction[geom_id].copy()

    remote_server.handle_request({
        "command": "set_body_properties",
        "bodies": ["box"],
        "properties": {"mass": original_mass * 2.0, "com": [0.1, 0.2, 0.3]},
    })
    remote_server.handle_request({
        "command": "set_contact_parameters",
        "bodies": ["box"],
        "parameters": {"friction": [1.5, 0.2, 0.1]},
    })
    remote_server.handle_request({
        "command": "apply_wrench",
        "body": "box",
        "force": [1.0, 2.0, 3.0],
        "torque": [4.0, 5.0, 6.0],
        "duration": 1.0,
    })
    remote_server.apply_wrenches()
    remote_server.data.qfrc_applied[:] = 7.0

    result = remote_server.handle_request({"command": "restore"})

    assert result == {"restored": True, "cleared_wrenches": 1}
    assert remote_server.model.body_mass[box_id] == original_mass
    np.testing.assert_array_equal(remote_server.model.body_ipos[box_id], original_com)
    np.testing.assert_array_equal(remote_server.model.geom_friction[geom_id], original_friction)
    np.testing.assert_array_equal(remote_server.data.xfrc_applied, 0.0)
    np.testing.assert_array_equal(remote_server.data.qfrc_applied, 0.0)
    remote_server.apply_wrenches()
    np.testing.assert_array_equal(remote_server.data.xfrc_applied, 0.0)


def test_zmq_json_request_response(remote_server):
    client = zmq.Context.instance().socket(zmq.REQ)
    client.setsockopt(zmq.LINGER, 0)
    client.connect(remote_server.endpoint)
    try:
        client.send_json({"id": "abc", "command": "not-a-command"})
        assert remote_server.process_requests() == 1
        response = client.recv_json()
        assert response["id"] == "abc"
        assert response["ok"] is False
        assert response["error"]["code"] == "unknown_command"

        client.send(b"{invalid json")
        assert remote_server.process_requests() == 1
        response = json.loads(client.recv())
        assert response["ok"] is False
        assert response["error"]["code"] == "invalid_json"
    finally:
        client.close()


def test_process_requests_does_not_block(remote_server):
    assert remote_server.process_requests() == 0
