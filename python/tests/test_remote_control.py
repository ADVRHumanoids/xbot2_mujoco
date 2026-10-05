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


@pytest.mark.parametrize("command", [
    {
        "command": "set_body_properties",
        "bodies": ["box"],
        "properties": {"mass": 12.5},
    },
    {
        "command": "set_body_properties",
        "bodies": ["box"],
        "properties": {"com": [0.01, -0.02, 0.03]},
    },
    {
        "command": "set_contact_parameters",
        "bodies": ["box"],
        "parameters": {"friction": [0.9, 0.02, 0.003]},
    },
    {
        "command": "add_payload",
        "body": "box",
        "mass": 2.0,
        "position": [0.1, -0.2, 0.3],
    },
    {"command": "restore"},
])
def test_parameter_changes_preserve_running_state(remote_server, command):
    model, data = remote_server.model, remote_server.data
    # Move away from the spawn pose and give the simulation nonzero momentum
    # and time, as it would have when a remote command arrives during a run.
    data.qpos[:3] = [1.0, -2.0, 0.15]
    data.qvel[:] = np.arange(model.nv) + 0.5
    data.time = 3.0
    mujoco.mj_forward(model, data)
    data.qacc_warmstart[:] = np.arange(model.nv) + 1.0
    state_type = mujoco.mjtState.mjSTATE_INTEGRATION
    before = np.empty(mujoco.mj_stateSize(model, state_type))
    mujoco.mj_getState(model, data, before, state_type)

    remote_server.handle_request(command)

    after = np.empty_like(before)
    mujoco.mj_getState(model, data, after, state_type)
    np.testing.assert_array_equal(after, before)
    mujoco.mj_step(model, data)
    assert data.time == pytest.approx(3.0 + model.opt.timestep)
    assert data.qpos[0] > 0.9
    assert data.qpos[1] < -1.9


@pytest.mark.parametrize("positions", [
    [[0.1, -0.2, 0.3]],  # At the original COM: inertia should stay unchanged.
    [[0.5, 0.2, -0.1]],
    [[0.5, 0.2, -0.1], [0.5, 0.2, -0.1]],
    [[0.5, 0.2, -0.1], [-0.3, 0.4, 0.6]],
])
def test_payload_combines_mass_com_and_rotated_inertia(remote_server, positions):
    model = remote_server.model
    body_id = model.body("box").id
    model.body_mass[body_id] = 2.0
    model.body_ipos[body_id] = [0.1, -0.2, 0.3]
    model.body_inertia[body_id] = [1.0, 2.0, 3.0]
    # A 90-degree rotation about Z swaps the X/Y principal moments.
    model.body_iquat[body_id] = [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)]
    # Capture this deliberately rotated body as the server's startup baseline.
    for field in ("body_mass", "body_ipos", "body_inertia", "body_iquat"):
        remote_server._original_model_values[field][:] = getattr(model, field)
    original_com = model.body_ipos[body_id].copy()

    def parallel_axis(mass, position):
        x, y, z = position
        return mass * np.array([
            [y*y + z*z, -x*y, -x*z],
            [-x*y, x*x + z*z, -y*z],
            [-x*z, -y*z, x*x + y*y],
        ])

    for position in positions:
        result = remote_server.handle_request({
            "command": "add_payload", "body": "box",
            "mass": 1.5, "position": position,
        })
    expected_mass = 2.0 + 1.5
    first_moment = 2.0 * original_com + 1.5 * np.asarray(positions[-1])
    inertia_at_origin = (
        np.diag([2.0, 1.0, 3.0]) + parallel_axis(2.0, original_com)
        + parallel_axis(1.5, positions[-1])
    )

    expected_com = first_moment / expected_mass
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, model.body_iquat[body_id])
    rotation = rotation.reshape(3, 3)
    actual_inertia = rotation @ np.diag(model.body_inertia[body_id]) @ rotation.T
    assert model.body_mass[body_id] == expected_mass
    np.testing.assert_allclose(model.body_ipos[body_id], expected_com)
    np.testing.assert_allclose(
        actual_inertia,
        inertia_at_origin - parallel_axis(expected_mass, expected_com),
        atol=1e-14,
    )
    assert model.body_subtreemass[body_id] >= expected_mass
    assert result["properties"]["mass"] == expected_mass
    np.testing.assert_allclose(result["properties"]["com"], expected_com)


def test_payload_replacement_matches_fresh_request(remote_server):
    fields = ("body_mass", "body_ipos", "body_inertia", "body_iquat")
    desired = {
        "command": "add_payload", "body": "box",
        "mass": 3.0, "position": [0.3, -0.2, 0.5],
    }
    remote_server.handle_request(desired)
    expected = {field: getattr(remote_server.model, field).copy() for field in fields}
    remote_server.handle_request({**desired, "mass": 5.0, "position": [-0.4, 0.1, 0.2]})
    remote_server.handle_request(desired)
    for field in fields:
        np.testing.assert_array_equal(getattr(remote_server.model, field), expected[field])
    remote_server.handle_request({
        "command": "set_body_properties", "bodies": ["box"],
        "properties": {"mass": 15.0, "com": [0.5, 0.6, 0.7]},
    })
    remote_server.handle_request(desired)
    for field in fields:
        np.testing.assert_array_equal(getattr(remote_server.model, field), expected[field])


def test_restore_removes_payload(remote_server):
    model = remote_server.model
    fields = ("body_mass", "body_ipos", "body_inertia", "body_iquat")
    original = {field: getattr(model, field).copy() for field in fields}
    remote_server.handle_request({
        "command": "add_payload", "body": "box",
        "mass": 3.0, "position": [0.3, -0.2, 0.5],
    })
    remote_server.handle_request({"command": "restore"})
    for field in fields:
        np.testing.assert_array_equal(getattr(model, field), original[field])


@pytest.mark.parametrize("overrides", [
    {"body": "missing"}, {"body": "world"}, {"body": "empty"},
    {"mass": 0.0}, {"mass": -1.0}, {"mass": float("nan")},
    {"mass": True}, {"position": [0.0, 0.0]},
    {"position": [0.0, float("inf"), 0.0]},
    {"position": [1e308, 0.0, 0.0]},
])
def test_invalid_payload_does_not_modify_model(remote_server, overrides):
    original = {
        field: getattr(remote_server.model, field).copy()
        for field in remote_server._RESTORABLE_MODEL_FIELDS
    }
    with pytest.raises(CommandError):
        remote_server.handle_request({
            "command": "add_payload", "body": "box", "mass": 2.0,
            "position": [0.1, 0.2, 0.3], **overrides,
        })
    for field, value in original.items():
        np.testing.assert_array_equal(getattr(remote_server.model, field), value)


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


def test_body_frame_wrench_tracks_pose_and_adds_to_world_wrench(remote_server):
    body_id = remote_server.model.body("box").id
    data = remote_server.data
    request = {
        "command": "apply_wrench", "body": "box",
        "force": [1.0, 2.0, 3.0], "torque": [4.0, 5.0, 6.0],
        "duration": 0.2, "frame": "body",
    }
    remote_server.handle_request(request)
    remote_server.handle_request({
        **request, "force": [10.0, 20.0, 30.0],
        "torque": [40.0, 50.0, 60.0], "duration": 0.3, "frame": "world",
    })
    world_wrench = np.array([10.0, 20.0, 30.0, 40.0, 50.0, 60.0])
    # Change qpos without refreshing derived poses: the server must use the
    # current orientation, rather than stale xmat from the previous step.
    half = np.sqrt(0.5)
    poses = [
        ([1.0, 0.0, 0.0, 0.0], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]),
        ([half, 0.0, 0.0, half], [-2.0, 1.0, 3.0, -5.0, 4.0, 6.0]),
        ([half, 0.0, half, 0.0], [3.0, 2.0, -1.0, 6.0, 5.0, -4.0]),
    ]
    for quaternion, local_wrench_in_world in poses:
        data.qpos[3:7] = quaternion
        remote_server.apply_wrenches()
        np.testing.assert_allclose(
            data.xfrc_applied[body_id], world_wrench + local_wrench_in_world,
        )
        np.testing.assert_array_equal(data.qpos[3:7], quaternion)

    data.time = 0.2
    remote_server.apply_wrenches()
    np.testing.assert_array_equal(data.xfrc_applied[body_id], world_wrench)
    data.time = 0.3
    remote_server.apply_wrenches()
    np.testing.assert_array_equal(data.xfrc_applied[body_id], np.zeros(6))


@pytest.mark.parametrize("frame", ["local", "", None, 1, ["body"]])
def test_rejects_invalid_wrench_frame(remote_server, frame):
    with pytest.raises(CommandError, match="frame"):
        remote_server.handle_request({
            "command": "apply_wrench", "body": "box",
            "force": [1.0, 0.0, 0.0], "torque": [0.0, 0.0, 0.0],
            "duration": 0.2, "frame": frame,
        })
    remote_server.apply_wrenches()
    np.testing.assert_array_equal(remote_server.data.xfrc_applied, 0.0)
    assert not remote_server._active_wrenches


def test_payload_visual_follows_body_and_volume_scales_with_mass(remote_server):
    scene = mujoco.MjvScene(remote_server.model, maxgeom=4)
    remote_server.data.qpos[:3] = [1.0, 2.0, 3.0]
    remote_server.data.qpos[3:7] = [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)]
    request = {
        "command": "add_payload", "body": "box",
        "mass": 1.0, "position": [0.2, 0.0, 0.1],
    }
    remote_server.handle_request(request)
    remote_server.update_visuals(scene)
    assert scene.ngeom == 1
    sphere = scene.geoms[0]
    assert sphere.type == mujoco.mjtGeom.mjGEOM_SPHERE
    np.testing.assert_allclose(sphere.pos, [1.0, 2.2, 3.1])
    assert sphere.size[0] == pytest.approx(0.05)
    original_volume = 4.0 / 3.0 * np.pi * sphere.size[0]**3

    remote_server.handle_request({**request, "mass": 8.0})
    remote_server.update_visuals(scene)
    assert scene.ngeom == 1
    assert sphere.size[0] == pytest.approx(0.1)
    assert (4.0 / 3.0 * np.pi * sphere.size[0]**3) == pytest.approx(8 * original_volume)

    remote_server.data.qpos[:3] = [4.0, 5.0, 6.0]
    remote_server.update_visuals(scene)
    np.testing.assert_allclose(sphere.pos, [4.0, 5.2, 6.1])
    remote_server.handle_request({"command": "restore"})
    remote_server.update_visuals(scene)
    assert scene.ngeom == 0


def test_force_visual_shows_net_force_at_com_and_expires(remote_server):
    model, data = remote_server.model, remote_server.data
    scene = mujoco.MjvScene(model, maxgeom=4)
    remote_server.handle_request({
        "command": "add_payload", "body": "box", "mass": 2.0,
        "position": [0.2, 0.0, 0.0],
    })
    request = {
        "command": "apply_wrench", "body": "box", "force": [100.0, 0.0, 0.0],
        "torque": [0.0, 0.0, 0.0], "duration": 0.2, "frame": "body",
    }
    remote_server.handle_request(request)
    remote_server.handle_request({**request, "force": [0.0, 20.0, 0.0], "frame": "world"})
    data.qpos[:3] = [1.0, 2.0, 3.0]
    data.qpos[3:7] = [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)]
    # Rendering must not change persistent inputs or integration state.
    data.xfrc_applied[:] = 7.0
    data.qfrc_applied[:] = 8.0
    state_type = mujoco.mjtState.mjSTATE_INTEGRATION
    before = np.empty(mujoco.mj_stateSize(model, state_type))
    mujoco.mj_getState(model, data, before, state_type)
    remote_server.update_visuals(scene)
    after = np.empty_like(before)
    mujoco.mj_getState(model, data, after, state_type)
    np.testing.assert_array_equal(after, before)
    assert scene.ngeom == 2
    arrow = scene.geoms[1]
    assert arrow.type == mujoco.mjtGeom.mjGEOM_ARROW
    body_id = model.body("box").id
    np.testing.assert_allclose(arrow.pos, data.xipos[body_id])
    np.testing.assert_allclose(arrow.mat[:, 2] * arrow.size[2], [0.0, 0.6, 0.0], atol=1e-14)
    data.time = 0.2
    remote_server.update_visuals(scene)
    assert scene.ngeom == 1  # Payload stays, force expires.
    remote_server.handle_request({"command": "restore"})
    remote_server.update_visuals(scene)
    assert scene.ngeom == 0


def test_force_visual_skips_cancelled_forces_and_pure_torque(remote_server):
    scene = mujoco.MjvScene(remote_server.model, maxgeom=4)
    request = {
        "command": "apply_wrench", "body": "box", "force": [10.0, 0.0, 0.0],
        "torque": [0.0, 0.0, 5.0], "duration": 0.2,
    }
    remote_server.handle_request(request)
    remote_server.handle_request({**request, "force": [-10.0, 0.0, 0.0]})
    remote_server.update_visuals(scene)
    assert scene.ngeom == 0


def test_visuals_respect_scene_capacity(remote_server):
    scene = mujoco.MjvScene(remote_server.model, maxgeom=1)
    remote_server.handle_request({
        "command": "add_payload", "body": "box", "mass": 2.0,
        "position": [0.2, 0.0, 0.0],
    })
    remote_server.handle_request({
        "command": "apply_wrench", "body": "box", "force": [10.0, 0.0, 0.0],
        "torque": [0.0, 0.0, 0.0], "duration": 0.2,
    })
    remote_server.update_visuals(scene)
    assert scene.ngeom == 1


def test_direct_body_edit_removes_payload_visual(remote_server):
    scene = mujoco.MjvScene(remote_server.model, maxgeom=4)
    remote_server.handle_request({
        "command": "add_payload", "body": "box", "mass": 2.0,
        "position": [0.2, 0.0, 0.0],
    })
    remote_server.handle_request({
        "command": "set_body_properties", "bodies": ["box"], "properties": {"mass": 15.0},
    })
    remote_server.update_visuals(scene)
    assert scene.ngeom == 0


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


@pytest.fixture
def actuated_server():
    model = mujoco.MjModel.from_xml_string("""
    <mujoco>
      <default><geom contype="0" conaffinity="0"/></default>
      <worldbody>
        <body name="base"><freejoint name="floating"/><geom size=".1"/>
          <body name="left" pos=".3 0 0">
            <joint name="left_knee" actuatorfrclimited="true" actuatorfrcrange="-8 8"/>
            <geom size=".1"/>
          </body>
          <body name="right" pos="-.3 0 0">
            <joint name="right_knee"/><geom size=".1"/>
          </body>
          <body name="slide" pos="0 .3 0">
            <joint name="slider" type="slide"/><geom size=".1"/>
          </body>
        </body>
      </worldbody>
      <actuator>
        <motor joint="left_knee" gear="2"/>
        <motor joint="left_knee" gear="3"/>
        <motor joint="right_knee"/>
        <motor joint="slider" gear="3"/>
      </actuator>
    </mujoco>
    """)
    server = RemoteControlServer(model, mujoco.MjData(model), f"inproc://limits-{uuid.uuid4()}")
    try:
        yield server
    finally:
        server.close()


@pytest.mark.parametrize("limit", [0.0, 3.0, 25.0])
def test_joint_torque_limits_clamp_net_torque_after_gearing(actuated_server, limit):
    model, data = actuated_server.model, actuated_server.data
    data.ctrl[:] = [10.0, 10.0, -40.0, 10.0]
    result = actuated_server.handle_request({
        "command": "set_joint_torque_limits", "joints": [".*_knee", "left_knee"], "limit": limit,
    })
    assert result == {"joints": ["left_knee", "right_knee"], "limit": limit}
    mujoco.mj_forward(model, data)
    for name, expected in (("left_knee", limit), ("right_knee", -limit), ("slider", 30.0)):
        joint_id = model.joint(name).id
        assert data.qfrc_actuator[model.jnt_dofadr[joint_id]] == pytest.approx(expected)
    # Replacing a limit works on the next step without changing control commands.
    actuated_server.handle_request({
        "command": "set_joint_torque_limits", "joints": [".*_knee"], "limit": 1.0,
    })
    mujoco.mj_step(model, data)
    assert data.qfrc_actuator[model.jnt_dofadr[model.joint("left_knee").id]] == pytest.approx(1.0)


def test_joint_limit_preserves_state_and_restore_recovers_original_limits(actuated_server):
    model, data = actuated_server.model, actuated_server.data
    original_flags = model.jnt_actfrclimited.copy()
    original_ranges = model.jnt_actfrcrange.copy()
    data.time = 3.0
    data.qpos[:3] = [1.0, 2.0, 3.0]
    data.qvel[:] = 0.1
    data.ctrl[:] = 10.0
    state_type = mujoco.mjtState.mjSTATE_INTEGRATION
    before = np.empty(mujoco.mj_stateSize(model, state_type))
    mujoco.mj_getState(model, data, before, state_type)
    actuated_server.handle_request({
        "command": "set_joint_torque_limits", "joints": [".*_knee|slider"], "limit": 2.0,
    })
    after = np.empty_like(before)
    mujoco.mj_getState(model, data, after, state_type)
    np.testing.assert_array_equal(after, before)
    actuated_server.handle_request({"command": "restore"})
    np.testing.assert_array_equal(model.jnt_actfrclimited, original_flags)
    np.testing.assert_array_equal(model.jnt_actfrcrange, original_ranges)
    mujoco.mj_forward(model, data)
    assert data.qfrc_actuator[model.jnt_dofadr[model.joint("left_knee").id]] == pytest.approx(8.0)
    assert data.qfrc_actuator[model.jnt_dofadr[model.joint("right_knee").id]] == pytest.approx(10.0)


@pytest.mark.parametrize("overrides", [
    {"joints": ["left_knee", "missing"]}, {"joints": ["left_knee", "["]},
    {"joints": ["left_knee", "floating"]}, {"joints": []},
    {"joints": ["knee"]}, {"limit": -1.0}, {"limit": float("inf")},
    {"limit": True}, {"regex": "true"},
])
def test_joint_limit_invalid_request_is_atomic(actuated_server, overrides):
    model = actuated_server.model
    original_flags = model.jnt_actfrclimited.copy()
    original_ranges = model.jnt_actfrcrange.copy()
    with pytest.raises(CommandError):
        actuated_server.handle_request({
            "command": "set_joint_torque_limits", "joints": [".*_knee"], "limit": 3.0, **overrides,
        })
    np.testing.assert_array_equal(model.jnt_actfrclimited, original_flags)
    np.testing.assert_array_equal(model.jnt_actfrcrange, original_ranges)


def test_joint_limit_exact_selection_and_slide_force(actuated_server):
    result = actuated_server.handle_request({
        "command": "set_joint_torque_limits", "joints": ["slider"], "limit": 2.0, "regex": False,
    })
    assert result["joints"] == ["slider"]
    actuated_server.data.ctrl[:] = 10.0
    mujoco.mj_forward(actuated_server.model, actuated_server.data)
    dof_id = actuated_server.model.jnt_dofadr[actuated_server.model.joint("slider").id]
    assert actuated_server.data.qfrc_actuator[dof_id] == pytest.approx(2.0)


def test_contact_regex_expands_and_deduplicates_bodies(remote_server):
    result = remote_server.handle_request({
        "command": "set_contact_parameters", "bodies": ["box|child", "box"],
        "regex": True, "parameters": {"friction": [0.05, 0.0, 0.0]},
    })
    assert result["bodies"] == ["box", "child"]
    assert len(result["geoms"]) == 3
    np.testing.assert_allclose(remote_server.model.geom_friction[:, 0], 0.05)


def test_body_properties_regex_updates_all_matches(remote_server):
    result = remote_server.handle_request({
        "command": "set_body_properties", "bodies": ["box|child"],
        "regex": True, "properties": {"mass": 3.0},
    })
    assert result["bodies"] == ["box", "child"]
    for name in result["bodies"]:
        assert remote_server.model.body_mass[remote_server.model.body(name).id] == 3.0


def test_payload_regex_updates_each_match_from_its_baseline(remote_server):
    original = remote_server.model.body_mass.copy()
    request = {
        "command": "add_payload", "body": "box|child", "regex": True,
        "mass": 1.5, "position": [0.1, 0.0, 0.0],
    }
    remote_server.handle_request(request)
    result = remote_server.handle_request(request)
    assert result["bodies"] == ["box", "child"]
    for name in result["bodies"]:
        body_id = remote_server.model.body(name).id
        assert remote_server.model.body_mass[body_id] == original[body_id] + 1.5
        assert result["properties"][name]["mass"] == original[body_id] + 1.5
    assert len(remote_server._payloads) == 2


def test_wrench_regex_schedules_once_per_match(remote_server):
    result = remote_server.handle_request({
        "command": "apply_wrench", "bodies": ["box|child", "box"], "regex": True,
        "force": [1.0, 2.0, 3.0], "torque": [4.0, 5.0, 6.0], "duration": 0.2,
    })
    assert result["bodies"] == ["box", "child"]
    remote_server.apply_wrenches()
    for name in result["bodies"]:
        np.testing.assert_array_equal(
            remote_server.data.xfrc_applied[remote_server.model.body(name).id],
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        )
    assert len(remote_server._active_wrenches) == 2


@pytest.mark.parametrize("command", [
    {"command": "set_contact_parameters", "parameters": {"friction": [0.1, 0.0, 0.0]}},
    {"command": "set_body_properties", "properties": {"mass": 3.0}},
    {"command": "add_payload", "mass": 2.0, "position": [0.1, 0.0, 0.0]},
    {"command": "apply_wrench", "force": [1.0, 0.0, 0.0], "torque": [0.0, 0.0, 0.0], "duration": 0.2},
])
@pytest.mark.parametrize("pattern", ["[", "missing", "ox"])
def test_body_regex_errors_are_atomic(remote_server, command, pattern):
    original = {field: getattr(remote_server.model, field).copy()
                for field in remote_server._RESTORABLE_MODEL_FIELDS}
    with pytest.raises(CommandError):
        remote_server.handle_request({**command, "bodies": ["box", pattern], "regex": True})
    for field, values in original.items():
        np.testing.assert_array_equal(getattr(remote_server.model, field), values)
    assert not remote_server._active_wrenches
    assert not remote_server._payloads


def test_payload_regex_validates_all_matches_before_mutation(remote_server):
    original = remote_server.model.body_mass.copy()
    with pytest.raises(CommandError, match="positive mass"):
        remote_server.handle_request({
            "command": "add_payload", "body": "box|empty", "regex": True,
            "mass": 1.0, "position": [0.1, 0.0, 0.0],
        })
    np.testing.assert_array_equal(remote_server.model.body_mass, original)
    assert not remote_server._payloads
