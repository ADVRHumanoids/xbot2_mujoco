
import time
import functools
from typing import Callable

import mujoco
from mujoco.viewer import launch_passive, Handle
from xbot2_py_mujoco.mj_xbot2_bridge import MjXbot2Bridge

class SimulatorWrapper:

    def __init__(self, model: mujoco.MjModel,
                 q_init: dict[str, float] = None,
                 ignore_joints: list[str] = None,
                 viewer: bool = True,
                 viewer_fps: int = 60,
                 viewer_key_callback: Callable[[int], None] = None,
                 ros: bool = True,
                 ros_node_name: str = None,
                 ros_robot_description_topic: str = None,
                 urdf_str: str = None,
                 srdf_str: str = None,
                 send_state_decimation: int = None,
                 sync_interval: float = None,
                 target_rtf: float = None,
                 socket_path: str = None):

        # define default values
        if q_init is None:
            q_init = {}

        if ignore_joints is None:
            ignore_joints = []

        if ros_robot_description_topic is None:
            ros_robot_description_topic = '/robot_description'
        
        if ros_node_name is None:
            ros_node_name = 'simulator_wrapper_node'

        if send_state_decimation is None:
            send_state_decimation = 1

        if sync_interval is None:
            sync_interval = 0.0

        if target_rtf is None:
            target_rtf = 1.0
    
        # save model and create data
        self.model = model
        self.data = mujoco.MjData(model)

        # create viewer if requested
        if viewer:
            self.viewer : Handle = launch_passive(model, self.data, key_callback=viewer_key_callback)
            self.viewer_fps = viewer_fps
        else:
            self.viewer : Handle = None

        # ros setup, urdf and srdf publishing
        if ros:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
            from rclpy.qos import QoSProfile, DurabilityPolicy
            rclpy.init()
            self.ros_node = Node(ros_node_name or 'simulator_wrapper_node')
            latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.urdf_pub = self.ros_node.create_publisher(String, ros_robot_description_topic, latching_qos)
            self.srdf_pub = self.ros_node.create_publisher(String, ros_robot_description_topic + '_semantic', latching_qos)
            if urdf_str:
                self.urdf_pub.publish(String(data=urdf_str))
                print(f"[Simulator] Published URDF to {ros_robot_description_topic}")
            if srdf_str:
                self.srdf_pub.publish(String(data=srdf_str))
                print(f"[Simulator] Published SRDF to {ros_robot_description_topic + '_semantic'}")

        # create xbot2 bridge server
        self.xbot2_bridge = MjXbot2Bridge(name='mj_simulator', model=self.model, data=self.data, socket_path=socket_path)

        # variables
        self.q_init = q_init
        self.time_start = time.perf_counter()
        self.send_state_decimation = send_state_decimation
        self.sync_interval = sync_interval
        self.rtf_print_interval = 1.0
        self.target_rtf = target_rtf
        self.iter_counter = 0
        self.frames = 0
        self.last_sync_wall = self.time_start
        self.last_sync_sim = self.data.time
        self.last_print_wall = self.time_start
        self.last_print_sim = self.data.time
        self.last_sim_time = self.data.time

        # running flag
        self.running = True

    def _catch_interrupt(fn):
        """Decorator: catch KeyboardInterrupt, set running=False and close viewer."""
        @functools.wraps(fn)
        def wrapper(self, *args, **kwargs):
            try:
                return fn(self, *args, **kwargs)
            except KeyboardInterrupt:
                self.running = False
                if self.viewer is not None:
                    self.viewer.close()
        return wrapper

    @_catch_interrupt
    def pre_step(self):
        
        # set initial position if sim time is 0 (first step or after reset)
        if self.data.time == 0.0:
            self._set_initial_position(self.data)
            self.data.time = self.last_sim_time
            self.last_sync_sim = self.data.time

        # receive commands from bridge
        self.xbot2_bridge.receive()

    
    @_catch_interrupt
    def step(self):

        # step simulation
        mujoco.mj_step(self.model, self.data)

    @_catch_interrupt
    def post_step(self):
        
        # save last sim time
        self.last_sim_time = self.data.time

        # send state to bridge at specified decimation
        self.iter_counter += 1
        if self.iter_counter % self.send_state_decimation == 0:
            self.xbot2_bridge.send_state()

        # sync viewer at given fps
        now = time.perf_counter()
        wall_elapsed_time = now - self.time_start

        if self.viewer is not None and self.frames < self.viewer_fps*wall_elapsed_time:
            self.viewer.sync(state_only=True)
            self.frames += 1
            
        # exit if viewer window is closed
        if self.viewer is not None and not self.viewer.is_running():
            self.running = False

        # print RTF at specified interval
        now = time.perf_counter()

        if now - self.last_print_wall >= self.rtf_print_interval:
            wall_delta = now - self.last_print_wall
            sim_delta = self.data.time - self.last_print_sim
            rtf = sim_delta / wall_delta if wall_delta > 0 else 0.0
            print(f'RTF: {rtf:.3f}')
            self.last_print_wall = now
            self.last_print_sim = self.data.time

        # sync to target RTF at specified interval
        if now - self.last_sync_wall >= self.sync_interval:
            wall_delta = now - self.last_sync_wall
            sim_delta = self.data.time - self.last_sync_sim
            sync_time = sim_delta/self.target_rtf - wall_delta 
            if sync_time > 0:
                time.sleep(sync_time)
            self.last_sync_wall = now
            self.last_sync_sim = self.data.time
            
            
    def run(self):
        self.pre_step()
        self.step()
        self.post_step()


    def _set_initial_position(self, data: mujoco.MjData):
        for joint, q in self.q_init.items():
            data.joint(joint).qpos = q
            data.actuator(joint).ctrl = q