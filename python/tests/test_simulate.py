import pytest
import mujoco

from xbot2_py_mujoco.simulate import (
    _detect_spawn_file,
    _load_spawn_locations,
    parse_args,
)
from xbot2_py_mujoco.simulator_wrapper import SimulatorWrapper


def test_parse_args_preserves_xml_merge_order():
    args = parse_args([
        '--urdf', 'robot.urdf',
        '--xml-cmd', 'xacro options.xml.xacro',
        '--xml', 'world.xml',
        '--xml-cmd', 'xacro actuators.xml.xacro',
    ])

    assert args.xml_merges == [
        ('cmd', 'xacro options.xml.xacro'),
        ('path', 'world.xml'),
        ('cmd', 'xacro actuators.xml.xacro'),
    ]


def test_load_spawn_locations(tmp_path):
    spawn_file = tmp_path / 'spawn.yaml'
    spawn_file.write_text('spawn_locations:\n  - [1, 2, 3]\n  - [4, 5, 6]\n')

    assert _load_spawn_locations(spawn_file) == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    assert _load_spawn_locations(None) == [(0.0, 0.0, 0.0)]


def test_detect_spawn_file_from_merged_xml(tmp_path):
    world = tmp_path / 'world.xml'
    world.write_text('<mujoco/>')
    spawn_file = tmp_path / 'spawn_locations.yaml'
    spawn_file.write_text('spawn_locations:\n  - [1, 2, 3]\n')

    assert _detect_spawn_file([('path', str(world))]) == spawn_file
    assert _detect_spawn_file([('cmd', 'generate-world')]) is None


def test_load_spawn_locations_rejects_invalid_shape(tmp_path):
    spawn_file = tmp_path / 'spawn.yaml'
    spawn_file.write_text('spawn_locations:\n  - [1, 2]\n')

    with pytest.raises(ValueError, match=r'\[x, y, z\]'):
        _load_spawn_locations(spawn_file)


def test_respawn_key_moves_free_root():
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body><freejoint/><geom size="1 1 1"/></body></worldbody></mujoco>'
    )
    simulator = SimulatorWrapper.__new__(SimulatorWrapper)
    simulator.model = model
    simulator.data = mujoco.MjData(model)
    simulator.spawn_locations = [(1.0, 2.0, 3.0)]
    simulator.q_init = {}
    simulator._respawn_lock = __import__('threading').Lock()
    simulator._respawn_requested = False

    simulator._viewer_key_callback(ord('r'))
    simulator._handle_respawn_request()

    assert simulator.data.qpos[:7].tolist() == [1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0]
