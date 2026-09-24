from xbot2_py_mujoco.simulate import parse_args


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
