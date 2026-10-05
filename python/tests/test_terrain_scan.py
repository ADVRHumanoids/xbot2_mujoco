import mujoco
import numpy as np
import pytest
from xbot2_py_mujoco.terrain_scan import TerrainScan, TerrainScanner


def make_scene(terrain):
    model = mujoco.MjModel.from_xml_string(f"""
    <mujoco>
      <default><geom group="2"/><default class="robot"><geom group="1"/></default></default>
      {terrain}
      <worldbody>
        <body name="robot" pos="0 0 1" childclass="robot">
          <freejoint/>
          <geom name="base" type="box" size=".6 .6 .1" group="0"/>
          <body name="leg" pos="0 0 -.4">
            <joint type="hinge"/>
            <geom name="leg_geom" type="box" size=".6 .6 .1"/>
            <body name="foot" pos="0 0 -.3">
              <geom name="foot_geom" type="box" size=".6 .6 .1"/>
            </body>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """)
    return model, mujoco.MjData(model)


def test_flat_scan_excludes_entire_articulated_robot_and_preserves_state():
    model, data = make_scene('<worldbody><geom name="ground" type="plane" size="10 10 .1"/></worldbody>')
    scanner = TerrainScanner(model, "robot", shape=(3, 4), spacing=(.2, .1))
    # Deliberately avoid mj_forward: scanner must handle an up-to-date qpos
    # with stale derived poses (also the normal situation after mj_step).
    data.qpos[:3] = [2, -3, 1]
    data.qpos[3:7] = [np.cos(.4), 0, 0, np.sin(.4)]
    data.qvel[:] = .5
    data.ctrl[:] = .25
    data.xfrc_applied[:] = .3
    data.qfrc_applied[:] = .4
    data.time = 7.5
    fields = ("qpos", "qvel", "ctrl", "xfrc_applied", "qfrc_applied")
    before = {name: getattr(data, name).copy() for name in fields}
    groups, rgba = model.geom_group.copy(), model.geom_rgba.copy()
    scan = scanner.scan(data)

    np.testing.assert_allclose(scan.center, [2, -3, 1])
    dx, dy = np.meshgrid([-.2, 0, .2], [-.15, -.05, .05, .15], indexing='ij')
    np.testing.assert_allclose(scan.x, 2 + np.cos(.8) * dx - np.sin(.8) * dy)
    np.testing.assert_allclose(scan.y, -3 + np.sin(.8) * dx + np.cos(.8) * dy)
    np.testing.assert_allclose(scan.heights, 0)
    assert scan.valid.all()
    assert scan.time == data.time == 7.5
    assert scan.ray_start_height == 3
    for name, value in before.items():
        np.testing.assert_array_equal(getattr(data, name), value)
    np.testing.assert_array_equal(model.geom_group, groups)
    np.testing.assert_array_equal(model.geom_rgba, rgba)
    # Reuse follows a new position while retaining the current body heading.
    data.qpos[0] += 1
    np.testing.assert_allclose(scanner.scan(data).x, scan.x + 1)


def test_step_height_and_misses():
    model, data = make_scene('''<worldbody>
      <geom name="step" type="box" pos=".2 0 .1" size=".09 .09 .1"/>
    </worldbody>''')
    scan = TerrainScanner(model, "robot", shape=(3, 1), spacing=(.2, .2)).scan(data)
    np.testing.assert_allclose(scan.heights[:, 0], [np.nan, np.nan, .2], equal_nan=True)
    assert scan.geom_ids[:, 0].tolist() == [-1, -1, model.geom("step").id]
    assert scan.valid[:, 0].tolist() == [False, False, True]
    short = TerrainScanner(model, "robot", shape=(3, 1), spacing=(.2, .2), max_distance=1)
    assert not short.scan(data).valid.any()


def test_height_field_and_mesh():
    model, data = make_scene('''
      <asset>
        <hfield name="terrain" nrow="2" ncol="2" size="1 1 .4 .1"/>
        <mesh name="cube" vertex="-.1 -.1 -.1  .1 -.1 -.1  .1 .1 -.1  -.1 .1 -.1
                                  -.1 -.1 .1  .1 -.1 .1  .1 .1 .1  -.1 .1 .1"/>
      </asset>
      <worldbody>
        <geom name="field" type="hfield" hfield="terrain"/>
        <geom name="mesh" type="mesh" mesh="cube" pos=".5 0 .5"/>
      </worldbody>
    ''')
    model.hfield_data[:] = .5
    scanner = TerrainScanner(model, "robot", shape=(3, 1), spacing=(.5, .1))
    scan = scanner.scan(data)
    np.testing.assert_allclose(scan.heights[:, 0], [.2, .2, .6], atol=1e-7)
    assert scan.geom_ids[-1, 0] == model.geom("mesh").id


def test_moving_nonrobot_body_is_included_and_invisible_terrain_is_skipped():
    model, data = make_scene('''<worldbody>
      <geom type="plane" size="10 10 .1" rgba="1 1 1 0"/>
      <body name="obstacle" pos="0 0 .3">
        <freejoint/><geom type="box" size=".1 .1 .1"/>
      </body>
    </worldbody>''')
    scanner = TerrainScanner(model, "robot", shape=(1, 1))
    np.testing.assert_allclose(scanner.scan(data).heights, .4)
    data.joint(0).qpos[0] = 2  # first free joint belongs to the obstacle
    assert not scanner.scan(data).valid.any()


@pytest.mark.parametrize("options", [
    {"robot_body": "missing"}, {"robot_body": "world"},
    {"shape": [0, 3]}, {"shape": [3.0, 3]}, {"shape": [True, 3]},
    {"spacing": [0, .1]}, {"spacing": [float("nan"), .1]},
    {"spacing": [.1]}, {"z_offset": -1}, {"max_distance": float("inf")},
    {"excluded_geom_groups": [6]}, {"excluded_geom_groups": [True]},
    {"excluded_geom_groups": []}, {"excluded_geom_groups": 1},
])
def test_invalid_configuration(options):
    model, _ = make_scene("")
    with pytest.raises(ValueError):
        TerrainScanner(model, **({"robot_body": "robot"} | options))


def test_simulator_updates_scan_at_configured_interval(monkeypatch):
    from types import SimpleNamespace
    from xbot2_py_mujoco import simulator_wrapper

    monkeypatch.setattr(simulator_wrapper, "MjXbot2Bridge", lambda **kwargs:
                        SimpleNamespace(send_state=lambda: None))
    model, data = make_scene('<worldbody><geom type="plane" size="10 10 .1"/></worldbody>')
    scanner = TerrainScanner(model, "robot", shape=(1, 1))
    sim = simulator_wrapper.SimulatorWrapper(
        model, viewer=False, ros=False, target_rtf=1e9,
        terrain_scanner=scanner, terrain_scan_decimation=2,
    )
    assert sim.terrain_scan is None
    sim.step()
    sim.post_step()
    assert sim.terrain_scan is None
    sim.step()
    sim.post_step()
    first_scan = sim.terrain_scan
    assert first_scan.time == sim.data.time
    np.testing.assert_allclose(first_scan.heights, 0)
    sim.step()
    sim.post_step()
    assert sim.terrain_scan is first_scan
    sim.close()


def test_rotated_plane_returns_world_heights():
    model, data = make_scene('''<worldbody>
      <geom type="plane" size="10 10 .1" euler="0 30 0"/>
    </worldbody>''')
    scan = TerrainScanner(model, "robot", shape=(3, 2), spacing=(.3, .2)).scan(data)
    expected = -np.tan(np.pi / 6) * scan.x
    np.testing.assert_allclose(scan.heights, expected, atol=1e-12)


def test_grid_follows_yaw_without_tilting_with_roll_or_pitch():
    model, data = make_scene('''<worldbody>
      <geom type="plane" size="10 10 .1" euler="0 30 0"/>
    </worldbody>''')
    scanner = TerrainScanner(model, "robot", shape=(3, 2), spacing=(.2, .2))
    initial = scanner.scan(data)
    np.testing.assert_allclose(initial.x, [[-.2, -.2], [0, 0], [.2, .2]])
    # Rz(90 deg) Ry(0.4) Rx(-0.3): body heading is +world Y despite tilt.
    yaw_quat = np.array([np.sqrt(.5), 0, 0, np.sqrt(.5)])
    pitch_quat = np.array([np.cos(.2), 0, np.sin(.2), 0])
    roll_quat = np.array([np.cos(-.15), np.sin(-.15), 0, 0])
    tilted = np.empty(4)
    mujoco.mju_mulQuat(tilted, yaw_quat, pitch_quat)
    mujoco.mju_mulQuat(data.qpos[3:7], tilted, roll_quat)
    scan = scanner.scan(data)
    expected_x = np.array([[.1, -.1]] * 3)
    expected_y = np.array([[-.2, -.2], [0, 0], [.2, .2]])
    np.testing.assert_allclose(scan.x, expected_x, atol=1e-12)
    np.testing.assert_allclose(scan.y, expected_y, atol=1e-12)
    # Downward world-vertical intersections with z = -tan(30 deg) * world X.
    np.testing.assert_allclose(scan.heights, -np.tan(np.pi / 6) * expected_x, atol=1e-12)
    assert scan.ray_start_height == 3
    scene = mujoco.MjvScene(model, maxgeom=10)
    scan.add_visuals(scene)
    for k, (i, j) in enumerate(np.ndindex(3, 2)):
        np.testing.assert_allclose(scene.geoms[k].pos,
                                   [expected_x[i, j], expected_y[i, j], scan.heights[i, j]],
                                   atol=1e-12)


def test_yawed_scan_hits_obstacle_along_body_forward_axis():
    model, data = make_scene('''<worldbody>
      <geom name="step" type="box" pos="0 .2 .1" size=".09 .09 .1"/>
    </worldbody>''')
    scanner = TerrainScanner(model, "robot", shape=(3, 1), spacing=(.2, .2))
    assert not scanner.scan(data).valid.any()
    data.qpos[3:7] = [np.sqrt(.5), 0, 0, np.sqrt(.5)]
    scan = scanner.scan(data)
    np.testing.assert_allclose(scan.heights[:, 0], [np.nan, np.nan, .2], equal_nan=True)
    assert scan.geom_ids[2, 0] == model.geom("step").id


def test_vertical_body_x_axis_uses_world_x_heading():
    model, data = make_scene('<worldbody><geom type="plane" size="10 10 .1"/></worldbody>')
    data.qpos[3:7] = [np.sqrt(.5), 0, np.sqrt(.5), 0]
    scan = TerrainScanner(model, "robot", shape=(3, 1), spacing=(.2, .2)).scan(data)
    np.testing.assert_allclose(scan.x[:, 0], [-.2, 0, .2])
    np.testing.assert_allclose(scan.y, 0)
    np.testing.assert_allclose(scan.heights, 0)


def test_custom_robot_groups_are_excluded_without_changing_model():
    model, data = make_scene('<worldbody><geom type="plane" size="10 10 .1" group="2"/></worldbody>')
    model.geom_group[model.geom_group == 0] = 4
    model.geom_group[model.geom_group == 1] = 5
    groups = model.geom_group.copy()
    scanner = TerrainScanner(model, "robot", shape=(1, 1), excluded_geom_groups=(4, 5))
    np.testing.assert_allclose(scanner.scan(data).heights, 0)
    np.testing.assert_array_equal(model.geom_group, groups)


def test_wrong_robot_group_is_rejected_without_model_mutation():
    model, _ = make_scene('<worldbody><geom name="ground" type="plane" size="10 10 .1"/></worldbody>')
    model.geom("foot_geom").group[:] = 2
    groups = model.geom_group.copy()
    with pytest.raises(ValueError, match="All robot geoms"):
        TerrainScanner(model, "robot")
    np.testing.assert_array_equal(model.geom_group, groups)



@pytest.mark.parametrize("group", [0, 1])
def test_environment_geoms_in_excluded_groups_are_skipped(group):
    model, data = make_scene(f'''<worldbody>
      <geom name="ground" type="plane" size="10 10 .1"/>
      <geom type="box" pos="0 0 .5" size=".1 .1 .1" group="{group}"/>
    </worldbody>''')
    groups = model.geom_group.copy()
    scanner = TerrainScanner(model, "robot", shape=(1, 1))
    scan = scanner.scan(data)
    np.testing.assert_allclose(scan.heights, 0)
    assert scan.geom_ids[0, 0] == model.geom("ground").id
    np.testing.assert_array_equal(model.geom_group, groups)


def visual_scan():
    return TerrainScan(
        time=1, center=np.zeros(3),
        x=np.array([[2., 2.], [3., 3.]]), y=np.array([[4., 5.], [4., 5.]]),
        ray_start_height=2,
        heights=np.array([[0., .5], [1., np.nan]]),
        geom_ids=np.array([[0, 0], [0, -1]], dtype=np.int32),
    )


def test_scan_spheres_have_hit_positions_and_height_hues():
    model, _ = make_scene("")
    scene = mujoco.MjvScene(model, maxgeom=10)
    visual_scan().add_visuals(scene)
    assert scene.ngeom == 3
    positions = [[2, 4, 0], [2, 5, .5], [3, 4, 1]]
    colors = [[.3, .3, 1, 1], [.3, 1, .3, 1], [1, .3, .3, 1]]
    for geom, pos, color in zip(scene.geoms[:3], positions, colors):
        assert geom.type == mujoco.mjtGeom.mjGEOM_SPHERE
        np.testing.assert_allclose(geom.pos, pos)
        np.testing.assert_allclose(geom.size, .025)
        np.testing.assert_allclose(geom.rgba, color, atol=1e-7)


def test_scan_spheres_append_and_respect_scene_capacity():
    model, _ = make_scene("")
    scene = mujoco.MjvScene(model, maxgeom=2)
    scene.ngeom = 1
    scene.geoms[0].rgba[:] = [.2, .3, .4, 1]
    visual_scan().add_visuals(scene)
    assert scene.ngeom == 2
    np.testing.assert_allclose(scene.geoms[0].rgba, [.2, .3, .4, 1])
    np.testing.assert_allclose(scene.geoms[1].pos, [2, 4, 0])
    visual_scan().add_visuals(scene)
    assert scene.ngeom == 2


def test_flat_scan_color_and_empty_scan():
    model, data = make_scene('<worldbody><geom type="plane" size="10 10 .1"/></worldbody>')
    scene = mujoco.MjvScene(model, maxgeom=10)
    scan = TerrainScanner(model, "robot", shape=(1, 1)).scan(data)
    scan.add_visuals(scene)
    np.testing.assert_allclose(scene.geoms[0].rgba, [.3, 1, .3, 1], atol=1e-7)
    scan.geom_ids[:] = -1
    scan.add_visuals(scene)
    assert scene.ngeom == 1


def test_fixed_height_range_and_marker_radius():
    model, _ = make_scene("")
    scene = mujoco.MjvScene(model, maxgeom=10)
    scan = visual_scan()
    scan.add_visuals(scene, radius=.04, height_range=(.25, .75))
    np.testing.assert_allclose(scene.geoms[0].rgba, [.3, .3, 1, 1], atol=1e-7)
    np.testing.assert_allclose(scene.geoms[2].rgba, [1, .3, .3, 1], atol=1e-7)
    np.testing.assert_allclose(scene.geoms[0].size, .04)
    with pytest.raises(ValueError, match="height_range"):
        scan.add_visuals(scene, height_range=(1, 1))
    with pytest.raises(ValueError, match="radius"):
        scan.add_visuals(scene, radius=0)


def test_viewer_composes_scan_and_remote_overlays_and_clears_old_hits():
    import uuid
    from xbot2_py_mujoco.remote_control import RemoteControlServer
    from xbot2_py_mujoco.simulator_wrapper import SimulatorWrapper

    model, data = make_scene("")
    server = RemoteControlServer(model, data, f"inproc://scan-visuals-{uuid.uuid4()}")
    try:
        server.handle_request({"command": "add_payload", "body": "robot",
                               "mass": 1, "position": [0, 0, .1]})
        sim = SimulatorWrapper.__new__(SimulatorWrapper)
        sim.remote_control = server
        sim.terrain_scan = visual_scan()
        sim.terrain_scan_visualization = True
        scene = mujoco.MjvScene(model, maxgeom=10)
        sim._update_visuals(scene)
        assert scene.ngeom == 4
        assert scene.geoms[0].label == "1 kg"
        sim._update_visuals(scene)
        assert scene.ngeom == 4  # rebuilt, never accumulated
        sim.terrain_scan.geom_ids[:] = -1
        sim._update_visuals(scene)
        assert scene.ngeom == 1  # only the remote payload remains
        sim.terrain_scan = visual_scan()
        sim.terrain_scan_visualization = False
        sim._update_visuals(scene)
        assert scene.ngeom == 1
        sim.remote_control = None
        sim.terrain_scan_visualization = True
        sim._update_visuals(scene)
        assert scene.ngeom == 3  # scan also renders without remote control
    finally:
        server.close()
