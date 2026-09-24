import mujoco
from pathlib import Path
import subprocess
import yaml
import argparse

from xbot2_py_mujoco.mjcf_tools import MjcfGenerator
from xbot2_py_mujoco.simulator_wrapper import SimulatorWrapper


class _AppendXmlMerge(argparse.Action):

    def __call__(self, parser, namespace, values, option_string=None):
        xml_merges = getattr(namespace, self.dest, None)
        if xml_merges is None:
            xml_merges = []
        kind = 'cmd' if option_string == '--xml-cmd' else 'path'
        xml_merges.append((kind, values))
        setattr(namespace, self.dest, xml_merges)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description='MuJoCo xbot2 simulator')

    # --- MjcfGenerator args ---
    g = p.add_argument_group('MjcfGenerator')
    g.add_argument('--urdf',       metavar='PATH', help='Path to robot URDF file')
    g.add_argument('--urdf-cmd',   metavar='CMD',  help='Command that generates the URDF')
    g.add_argument('--name',       default='robot', metavar='NAME', help='Robot name (default: robot)')
    g.add_argument('--xml', dest='xml_merges', action=_AppendXmlMerge, default=None, metavar='PATH',
                   help='Extra MJCF XML file to merge (repeatable, in command-line order)')
    g.add_argument('--xml-cmd', dest='xml_merges', action=_AppendXmlMerge, default=None, metavar='CMD',
                   help='Command that generates extra MJCF XML file to merge (repeatable, in command-line order)')
    g.add_argument('--output-dir', metavar='DIR',  help='MjcfGenerator output directory')
    g.add_argument('--copy-assets', action='store_true', help='Copy assets instead of symlinking')

    # --- SimulatorWrapper args ---
    s = p.add_argument_group('SimulatorWrapper')
    s.add_argument('--srdf',       metavar='PATH', help='Path to robot SRDF file')
    s.add_argument('--srdf-cmd',   metavar='CMD',  help='Command that generates the SRDF')
    s.add_argument('--headless',              action='store_true', help='Disable MuJoCo viewer')
    s.add_argument('--viewer-fps',            type=int,   default=60,    metavar='N',   help='Viewer render fps (default: 60)')
    s.add_argument('--disable-ros',           action='store_true', help='Disable ROS2 publishing')
    s.add_argument('--ros-node-name',         metavar='NAME', help='ROS2 node name')
    s.add_argument('--ros-topic',             metavar='TOPIC', help='ROS2 robot_description topic')
    s.add_argument('--send-state-decimation', type=int,   metavar='N',   help='Send state every N steps')
    s.add_argument('--sync-interval',         type=float, metavar='SEC', help='Real-time sync interval in seconds')
    s.add_argument('--target-rtf',            type=float, metavar='RTF', help='Target real-time factor (default: 1.0)')
    s.add_argument('--socket-path',           metavar='PATH', help='Unix socket path for xbot2 bridge')

    return p.parse_args(argv)


def _read_or_run(path_arg, cmd_arg, label):
    if path_arg and cmd_arg:
        raise ValueError(f'Provide either --{label} or --{label}-cmd, not both')
    if path_arg:
        return Path(path_arg).read_text()
    if cmd_arg:
        try:
            result = subprocess.run(cmd_arg, shell=True, capture_output=True, text=True, check=True)
        except subprocess.CalledProcessError as e:
            print(e.stderr)
            raise
        return result.stdout
    return None


def main():

    args = parse_args()

    # read urdf and srdf from file or command
    urdf = _read_or_run(args.urdf, args.urdf_cmd, 'urdf')
    if urdf is None:
        raise ValueError('Provide either --urdf or --urdf-cmd')
    srdf = _read_or_run(args.srdf, args.srdf_cmd, 'srdf')

    # create the mjcf file (xml) from the urdf, merging extra xml files and 
    # finally patching the result with information from the config yaml file
    gen = MjcfGenerator(name=args.name,
                        urdf_str=urdf,
                        output_dir=args.output_dir,
                        copy_assets=args.copy_assets)

    for xml_kind, xml_value in args.xml_merges or []:
        if xml_kind == 'path':
            gen.merge_xml(Path(xml_value))
        else:
            xml_str = _read_or_run(None, xml_value, 'xml')
            gen.merge_xml(xml_str)

    # create mujoco model from the generated mjcf xml string
    model = mujoco.MjModel.from_xml_string(gen.generate_mjcf_string())

    # create simulator wrapper, which will handle the simulation loop, viewer, ROS publishing, and xbot2 bridge communication
    sim = SimulatorWrapper(
        model=model,
        q_init=gen.q_init,
        viewer=not args.headless,
        viewer_fps=args.viewer_fps,
        ros=not args.disable_ros,
        ros_node_name=args.ros_node_name,
        ros_robot_description_topic=args.ros_topic,
        urdf_str=urdf,
        srdf_str=srdf,
        send_state_decimation=args.send_state_decimation,
        sync_interval=args.sync_interval,
        target_rtf=args.target_rtf,
        socket_path=args.socket_path,
    )

    # main simulation loop, which runs until the viewer window is closed (if enabled) or the process is killed
    while sim.running:
        sim.pre_step()
        sim.step()
        sim.post_step()


if __name__ == '__main__':
    main()
