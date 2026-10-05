import pytest
import mujoco
import numpy as np

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


@pytest.mark.parametrize('enabled', [None, '', 'base'])
def test_main_wires_terrain_scan_options(monkeypatch, enabled):
    from types import SimpleNamespace
    from xbot2_py_mujoco import simulate

    argv = ['--urdf', 'robot.urdf', '--headless', '--disable-ros']
    if enabled:
        argv += [
            '--terrain-scan-body', 'base', '--terrain-scan-shape', '3', '2',
            '--terrain-scan-spacing', '0.2', '0.3',
            '--terrain-scan-z-offset', '1', '--terrain-scan-max-distance', '4',
            '--terrain-scan-excluded-groups', '3', '4',
            '--terrain-scan-decimation', '5', '--no-terrain-scan-visualization',
            '--terrain-scan-topic', '/custom_heightscan',
        ]
    elif enabled == '':
        argv += ['--terrain-scan-body=']
    args = parse_args(argv)
    monkeypatch.setattr(simulate, 'parse_args', lambda: args)
    monkeypatch.setattr(simulate, '_read_or_run', lambda *args: '<robot/>')
    xml = '''<mujoco><worldbody>
      <geom type="plane" size="10 10 .1" group="2"/>
      <body name="base" pos="0 0 1">
        <freejoint/><geom type="box" size=".1 .1 .1" group="3"/>
      </body>
    </worldbody></mujoco>'''
    monkeypatch.setattr(simulate, 'MjcfGenerator', lambda **kwargs:
                        SimpleNamespace(q_init={}, generate_mjcf_string=lambda: xml))
    captured = {}
    closed = []

    def create_simulator(**kwargs):
        captured.update(kwargs)
        return SimpleNamespace(running=False, close=lambda: closed.append(True))

    monkeypatch.setattr(simulate, 'SimulatorWrapper', create_simulator)
    simulate.main()
    assert closed == [True]
    assert captured['viewer'] is False
    if enabled:
        scanner = captured['terrain_scanner']
        scan = scanner.scan(mujoco.MjData(captured['model']))
        assert scan.heights.shape == (3, 2)
        np.testing.assert_allclose(scan.heights, 0)
        np.testing.assert_allclose(scan.x, [[-.2, -.2], [0, 0], [.2, .2]])
        np.testing.assert_allclose(scan.y, [[-.15, .15]] * 3)
        assert scan.ray_start_height == 2
        assert scanner.max_distance == 4
        assert captured['terrain_scan_decimation'] == 5
        assert captured['terrain_scan_visualization'] is False
        assert captured['terrain_scan_topic'] == '/custom_heightscan'
    else:
        assert captured['terrain_scanner'] is None
        assert captured['terrain_scan_decimation'] == 100
        assert captured['terrain_scan_visualization'] is True
        assert captured['terrain_scan_topic'] == '/heightscan'


@pytest.mark.parametrize('options', [
    ['--terrain-scan-shape', '0', '2'],
    ['--terrain-scan-spacing', '.1', '-.1'],
    ['--terrain-scan-z-offset', 'nan'],
    ['--terrain-scan-max-distance', 'inf'],
    ['--terrain-scan-decimation', '0'],
    ['--terrain-scan-excluded-groups', '6'],
])
def test_cli_rejects_invalid_terrain_scan_values(options):
    with pytest.raises(SystemExit) as exc:
        parse_args(options)
    assert exc.value.code == 2
