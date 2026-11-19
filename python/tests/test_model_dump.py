from types import SimpleNamespace

import mujoco

from xbot2_py_mujoco import simulate
from xbot2_py_mujoco.model_dump import format_mj_model, print_mj_model
from xbot2_py_mujoco.simulate import parse_args

XML = '''<mujoco><worldbody>
  <body name="base" pos="0 0 1"><freejoint name="root"/><geom name="torso" type="box" size=".1 .1 .1"/>
    <body name="leg" pos=".2 0 0"><joint name="knee"/><geom type="capsule" size=".02 .1"/>
      <site name="foot" pos="0 0 -.1"/></body>
  </body></worldbody>
  <actuator><motor name="knee_motor" joint="knee"/></actuator>
  <sensor><jointpos name="knee_pos" joint="knee"/></sensor>
</mujoco>'''


def test_format_lists_the_model_tables():
    text = format_mj_model(mujoco.MjModel.from_xml_string(XML))
    for section in ('=== MjModel ===', '=== Bodies ===', '=== Joints ===', '=== DoFs ===', '=== Actuators ===',
                    '=== Sensors ===', '=== Geoms ===', '=== Sites ===', '=== opt ==='):
        assert section in text
    assert 'root' in text and 'FREE' in text and 'HINGE' in text
    assert 'target=joint:knee' in text and 'knee_pos' in text and 'foot' in text
    assert 'mj_printModel' not in text


def test_full_dump_to_file(tmp_path):
    path = tmp_path / 'model.txt'
    print_mj_model(mujoco.MjModel.from_xml_string(XML), full_dump=True, file=str(path))
    text = path.read_text()
    assert text.startswith('=== MjModel ===')
    assert '=== mj_printModel (verbose) ===' in text


def test_main_dumps_the_model_when_asked(monkeypatch, tmp_path):
    path = tmp_path / 'dump.txt'
    args = parse_args(['--urdf', 'robot.urdf', '--headless', '--disable-ros', '--dump-model', str(path)])
    assert parse_args([]).dump_model is None
    monkeypatch.setattr(simulate, 'parse_args', lambda: args)
    monkeypatch.setattr(simulate, '_read_or_run', lambda *args: '<robot/>')
    monkeypatch.setattr(simulate, 'MjcfGenerator', lambda **kwargs:
                        SimpleNamespace(q_init={}, rgbd_cameras=[], generate_mjcf_string=lambda: XML))
    monkeypatch.setattr(simulate, 'SimulatorWrapper', lambda **kwargs:
                        SimpleNamespace(running=False, close=lambda: None))
    simulate.main()
    assert 'knee_motor' in path.read_text()
