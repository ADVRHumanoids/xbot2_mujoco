
import time
import math
from collections import deque
import functools
import random
import threading
from typing import Callable

import mujoco
import numpy as np
from mujoco.viewer import launch_passive, Handle
from xbot2_py_mujoco.mj_xbot2_bridge import MjXbot2Bridge
from xbot2_py_mujoco.remote_control import RemoteControlServer
from xbot2_py_mujoco.terrain_scan import TerrainScanner, TerrainScan
from xbot2_py_mujoco.rgbd import RgbdCameraCfg, RgbdCameraManager, validate_cameras
from xbot2_py_mujoco.lidar import Lidar, LidarCfg, LidarScan

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
                 socket_path: str = None,
                 remote_control_endpoint: str = None,
                 actuator_delay_s: float = 0.0,
                 spawn_locations: list[tuple[float, float, float]] = None,
                 terrain_scanner: TerrainScanner = None,
                 terrain_scan_decimation: int = 100,
                 terrain_scan_visualization: bool = True,
                 terrain_scan_topic: str = '/heightscan',
                 rgbd_cameras: list[RgbdCameraCfg] | None = None,
                 lidars: list[LidarCfg] | None = None):

        rgbd_cameras = tuple(rgbd_cameras or ())
        validate_cameras(model, rgbd_cameras)
        self.rgbd_manager = None
        self.rgbd_publisher = None

        if (isinstance(terrain_scan_decimation, bool)
                or not isinstance(terrain_scan_decimation, int)
                or terrain_scan_decimation <= 0):
            raise ValueError("terrain_scan_decimation must be a positive integer")
        if terrain_scanner is not None and terrain_scanner.model is not model:
            raise ValueError("terrain_scanner must use the simulator model")
        self.terrain_scanner = terrain_scanner
        self.terrain_scan_decimation = terrain_scan_decimation
        self.terrain_scan: TerrainScan | None = None
        self.terrain_scan_visualization = terrain_scan_visualization
        self.terrain_scan_pub = None
        self.lidars = [Lidar(model, cfg) for cfg in lidars or ()]
        self.lidar_scans: dict[str, LidarScan] = {}
        self.lidar_pubs = {}
        self.clock_pub = None

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
        if not math.isfinite(target_rtf) or target_rtf <= 0:
            raise ValueError("target_rtf must be finite and positive")

        if spawn_locations is None:
            spawn_locations = [(0.0, 0.0, 0.0)]
    
        # save model and create data
        self.model = model
        self.data = mujoco.MjData(model)
        self.spawn_locations = [tuple(location) for location in spawn_locations]
        self._respawn_lock = threading.Lock()
        self._respawn_requested = False
        # The viewer callback only changes control state, never MuJoCo data.
        self._paused = False
        self._advance_requests = deque()
        self._advance_target = None
        self._timing_reset_requested = False
        self._pre_step_skipped = False
        self._step_skipped = False
        self.viewer_fps = viewer_fps
        self.target_rtf = target_rtf

        # create viewer if requested
        if viewer:
            if viewer_key_callback is None:
                viewer_key_callback = self._viewer_key_callback
            self.viewer : Handle = launch_passive(model, self.data, key_callback=viewer_key_callback)
        else:
            self.viewer : Handle = None

        # ros setup, urdf and srdf publishing
        if ros:
            import rclpy
            from rclpy.node import Node
            from rosgraph_msgs.msg import Clock
            from std_msgs.msg import String
            from rclpy.qos import QoSProfile, DurabilityPolicy
            rclpy.init()
            self.ros_node = Node(ros_node_name or 'simulator_wrapper_node')
            self.clock_pub = self.ros_node.create_publisher(Clock, '/clock', 1)
            latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.urdf_pub = self.ros_node.create_publisher(String, ros_robot_description_topic, latching_qos)
            self.srdf_pub = self.ros_node.create_publisher(String, ros_robot_description_topic + '_semantic', latching_qos)
            if urdf_str:
                self.urdf_pub.publish(String(data=urdf_str))
                print(f"[Simulator] Published URDF to {ros_robot_description_topic}")
            if srdf_str:
                self.srdf_pub.publish(String(data=srdf_str))
                print(f"[Simulator] Published SRDF to {ros_robot_description_topic + '_semantic'}")
            if terrain_scanner is not None:
                from sensor_msgs.msg import PointCloud2
                from rclpy.qos import qos_profile_sensor_data
                self.terrain_scan_pub = self.ros_node.create_publisher(
                    PointCloud2, terrain_scan_topic, qos_profile_sensor_data,
                )
            if self.lidars:
                from sensor_msgs.msg import PointCloud2
                from rclpy.qos import qos_profile_sensor_data
                from tf2_ros import StaticTransformBroadcaster
                from xbot2_py_mujoco.lidar_ros import site_transform
                for lidar in self.lidars:
                    self.lidar_pubs[lidar.cfg.site] = self.ros_node.create_publisher(
                        PointCloud2, lidar.cfg.topic, qos_profile_sensor_data,
                    )
                self.lidar_tf = StaticTransformBroadcaster(self.ros_node)
                self.lidar_tf.sendTransform([site_transform(model, l.cfg) for l in self.lidars])
            if rgbd_cameras:
                from xbot2_py_mujoco.rgbd_ros import RgbdPublisher
                self.rgbd_publisher = RgbdPublisher(self.ros_node, rgbd_cameras)

        # create xbot2 bridge server
        self.xbot2_bridge = MjXbot2Bridge(
            name='mj_simulator', model=self.model, data=self.data,
            socket_path=socket_path, actuator_delay_s=actuator_delay_s,
        )
        self.remote_control = (
            RemoteControlServer(self.model, self.data, remote_control_endpoint,
                                actuator_bridge=self.xbot2_bridge)
            if remote_control_endpoint else None
        )

        # variables
        self.q_init = q_init
        self.time_start = time.perf_counter()
        self._next_viewer_sync_wall = self.time_start
        self.send_state_decimation = send_state_decimation
        self.sync_interval = sync_interval
        self.rtf_print_interval = 1.0
        self.iter_counter = 0
        self.frames = 0
        self.last_sync_wall = self.time_start
        self.last_sync_sim = self.data.time
        self.last_print_wall = self.time_start
        self.last_print_sim = self.data.time
        self.last_sim_time = self.data.time
        self.sync_error_integral = .0

        # running flag
        self.running = True
        if rgbd_cameras:
            self.rgbd_manager = RgbdCameraManager(model, rgbd_cameras)

    @property
    def rgbd_frames(self):
        """Independent copies of the latest RGB-D frames, keyed by camera name."""
        manager = getattr(self, 'rgbd_manager', None)
        return manager.latest_frames if manager is not None else {}

    def _poll_rgbd(self):
        manager = getattr(self, 'rgbd_manager', None)
        if manager is not None:
            for frame in manager.poll():
                if self.rgbd_publisher is not None:
                    self.rgbd_publisher.publish(frame)

    @property
    def paused(self) -> bool:
        """Whether continuous simulation updates are paused (including advances)."""
        with self._respawn_lock:
            return self._paused

    def set_paused(self, paused: bool = True):
        """Pause or resume; safe to call from a viewer callback or another thread."""
        with self._respawn_lock:
            if self._paused == paused:
                return
            self._paused = bool(paused)
            self._advance_requests.clear()
            self._advance_target = None
            self._timing_reset_requested = True
            if self._paused:
                self._respawn_requested = False

    def advance(self, duration_s: float = 0.1):
        """Queue a positive simulation-time advance while paused; otherwise ignore it."""
        if not math.isfinite(duration_s) or duration_s <= 0:
            raise ValueError("duration_s must be finite and positive")
        with self._respawn_lock:
            if self._paused:
                self._advance_requests.append(duration_s)

    def set_target_rtf(self, target_rtf: float):
        """Set a positive target RTF and rebase pacing on the simulation thread."""
        if not math.isfinite(target_rtf) or target_rtf <= 0:
            raise ValueError("target_rtf must be finite and positive")
        with self._respawn_lock:
            if self.target_rtf != target_rtf:
                self.target_rtf = target_rtf
                self._timing_reset_requested = True

    def _viewer_key_callback(self, key):
        if key == ord(' '):
            self.set_paused(not self.paused)
        elif key == 262:  # GLFW_KEY_RIGHT, as passed by MuJoCo's viewer
            self.advance(duration_s=0.01666)  # ~1 frame at 60Hz
        elif key in (ord('+'), ord('='), 334, ord('-'), 333):
            # GLFW reports the main '+' key as EQUAL; 334/333 are keypad +/-.
            delta = -0.1 if key in (ord('-'), 333) else 0.1
            self.set_target_rtf(max(0.1, round(self.target_rtf + delta, 10)))
            print(f"[Simulator] Target RTF: {self.target_rtf:g}")
        elif key in (ord('r'), ord('R')):
            with self._respawn_lock:
                if not self._paused:
                    self._respawn_requested = True

    def _handle_respawn_request(self):
        with self._respawn_lock:
            if self._paused or not self._respawn_requested:
                return
            self._respawn_requested = False
        self._respawn()

    def _reset_timing(self):
        """Rebase pacing on the simulation thread, excluding time spent idle."""
        now = time.perf_counter()
        self.time_start = now
        self.frames = 0
        self._next_viewer_sync_wall = now
        self.last_sync_wall = self.last_print_wall = now
        self.last_sync_sim = self.last_print_sim = self.data.time
        self.sync_error_integral = 0.0

    def _can_update(self, start_advance=False):
        with self._respawn_lock:
            if (self._paused and start_advance and self._advance_target is None
                    and self._advance_requests):
                self._advance_target = self.data.time + self._advance_requests.popleft()
            enabled = not self._paused or self._advance_target is not None
            reset_timing = enabled and self._timing_reset_requested
            if reset_timing:
                self._timing_reset_requested = False
        if reset_timing:
            self._reset_timing()
        return enabled

    def _sync_viewer(self, idle=False):
        """Keep the viewer interactive even when no simulation updates are allowed."""
        self._poll_rgbd()
        period = 1.0 / max(1, self.viewer_fps)
        now = time.perf_counter()
        if idle:
            delay = max(0.0, self._next_viewer_sync_wall - now) if self.viewer else period
            if delay:
                time.sleep(delay)
            now = time.perf_counter()
        if self.viewer is None:
            return
        if not self.viewer.is_running():
            self.running = False
            return
        if now >= self._next_viewer_sync_wall:
            if self.remote_control is not None or self.terrain_scanner is not None:
                with self.viewer.lock():
                    self._update_visuals(self.viewer.user_scn)
            self.viewer.sync(state_only=True)
            self.frames += 1
            self._next_viewer_sync_wall = now + period
        if not self.viewer.is_running():
            self.running = False

    def _respawn(self):
        if getattr(self, 'remote_control', None) is not None:
            self.remote_control.clear_wrenches()
        free_joints = np.flatnonzero(
            self.model.jnt_type == mujoco.mjtJoint.mjJNT_FREE
        )
        if not len(free_joints):
            print("[Simulator] Warning: No free joint found in the model, cannot respawn.")
            return
        if getattr(self, 'rgbd_manager', None) is not None:
            self.rgbd_manager.reset()
        joint_id = int(free_joints[0])
        qpos_start = self.model.jnt_qposadr[joint_id]
        dof_start = self.model.jnt_dofadr[joint_id]
        x, y, z = random.choice(self.spawn_locations)
        print(f"[Simulator] Respawning asset to location ({x}, {y}, {z}).")
        self.data.qpos[qpos_start:qpos_start + 3] = (x, y, z)
        self.data.qpos[qpos_start + 3:qpos_start + 7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qvel[dof_start:dof_start + 6] = 0.0
        self._set_initial_position(self.data)
        mujoco.mj_forward(self.model, self.data)

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
        self._pre_step_skipped = not self._can_update(start_advance=True)
        if self._pre_step_skipped:
            return
        self._handle_respawn_request()
        
        # set initial position if sim time is 0 (first step or after reset)
        if self.data.time == 0.0:
            if getattr(self, 'rgbd_manager', None) is not None:
                self.rgbd_manager.reset()
            self._set_initial_position(self.data)
            self.data.time = self.last_sim_time
            self.last_sync_sim = self.data.time

        # receive commands from bridge
        self.xbot2_bridge.receive()

        # Apply remote commands on the simulation thread. Wrenches are refreshed
        # every step because MuJoCo treats xfrc_applied as persistent input.
        if self.remote_control is not None:
            self.remote_control.process_requests()
            self.remote_control.apply_wrenches()

    
    @_catch_interrupt
    def step(self):
        # If a key arrives between phases, wait for the next complete cycle so
        # an advance cannot start with physics but without bridge receive.
        self._step_skipped = self._pre_step_skipped or not self._can_update(start_advance=True)
        if self._step_skipped:
            return

        # step simulation
        mujoco.mj_step(self.model, self.data)

    @_catch_interrupt
    def post_step(self):
        if self._pre_step_skipped or self._step_skipped or not self._can_update():
            self._sync_viewer(idle=True)
            return

        # save last sim time
        self.last_sim_time = self.data.time
        if self.clock_pub is not None:
            from builtin_interfaces.msg import Time
            from rosgraph_msgs.msg import Clock
            seconds, nanoseconds = divmod(round(self.data.time * 1e9), 1_000_000_000)
            self.clock_pub.publish(Clock(clock=Time(sec=seconds, nanosec=nanoseconds)))

        if self.rgbd_manager is not None:
            self.rgbd_manager.submit(self.data)

        # send state to bridge at specified decimation
        self.iter_counter += 1
        if (self.terrain_scanner is not None
                and self.iter_counter % self.terrain_scan_decimation == 0):
            self.terrain_scan = self.terrain_scanner.scan(self.data)
            if self.terrain_scan_pub is not None:
                from xbot2_py_mujoco.terrain_scan_ros import pointcloud_from_scan
                frame_id = self.model.body(self.terrain_scanner.body_id).name
                self.terrain_scan_pub.publish(pointcloud_from_scan(self.terrain_scan, frame_id))
        for lidar in self.lidars:
            scan = lidar.step(self.data)
            if scan is not None:
                self.lidar_scans[lidar.cfg.site] = scan
                publisher = self.lidar_pubs.get(lidar.cfg.site)
                if publisher is not None:
                    from xbot2_py_mujoco.lidar_ros import pointcloud_from_scan
                    publisher.publish(pointcloud_from_scan(scan, lidar.cfg.frame_id))
        if self.iter_counter % self.send_state_decimation == 0:
            self.xbot2_bridge.send_state()

        self._sync_viewer()

        # sync to target RTF at specified interval
        now = time.perf_counter()
        if now - self.last_sync_wall >= self.sync_interval:
            wall_delta = now - self.last_sync_wall
            sim_delta = self.data.time - self.last_sync_sim
            sync_time = sim_delta/self.target_rtf - wall_delta + self.sync_error_integral
            if sync_time > 0:
                time.sleep(sync_time)

            # Mark the sync point after sleeping. If we kept the pre-sleep
            # timestamp, the next cycle would count this sleep again and skip
            # throttling, producing an RTF above the target.
            self.last_sync_wall = time.perf_counter()
            self.last_sync_sim = self.data.time

        # print RTF at specified interval
        now = time.perf_counter()

        if now - self.last_print_wall >= self.rtf_print_interval:
            # 
            wall_delta = now - self.last_print_wall
            sim_delta = self.data.time - self.last_print_sim

            # closed loop rtf correction
            sync_error = sim_delta/self.target_rtf - wall_delta
            self.sync_error_integral += 0.5*sync_error * (self.model.opt.timestep / self.rtf_print_interval) 
            # clamp integral to avoid windup
            self.sync_error_integral = max(min(self.sync_error_integral, 0.1), -0.1)

            # print rtf
            rtf = sim_delta / wall_delta if wall_delta > 0 else 0.0
            print(f'RTF: {rtf:.3f} (Sync error: {1000*sync_error:.3f} ms, Correction: {1000*self.sync_error_integral:.3f} ms)')
            self.last_print_wall = now
            self.last_print_sim = self.data.time
            
            
        # Finish an advance only after all normal post-step updates and pacing.
        with self._respawn_lock:
            if (self._advance_target is not None
                    and self.data.time >= self._advance_target - 1e-12):
                self._advance_target = None
                if not self._advance_requests:
                    self._timing_reset_requested = True

    def _update_visuals(self, scene: mujoco.MjvScene):
        """Rebuild all simulator overlays while holding the viewer lock."""
        scene.ngeom = 0
        if self.remote_control is not None:
            self.remote_control.update_visuals(scene)
        if self.terrain_scan_visualization and self.terrain_scan is not None:
            self.terrain_scan.add_visuals(scene)

    def run(self):
        self.pre_step()
        self.step()
        self.post_step()

    def close(self):
        if getattr(self, 'rgbd_manager', None) is not None:
            self.rgbd_manager.close()
        if self.remote_control is not None:
            self.remote_control.close()
        if self.viewer is not None:
            self.viewer.close()
        self.xbot2_bridge.close()


    def _set_initial_position(self, data: mujoco.MjData):
        for joint, q in self.q_init.items():
            try:
                data.joint(joint).qpos = q
                actuator = self.model.actuator(joint)
                if actuator.dyntype[0] != mujoco.mjtDyn.mjDYN_INTEGRATOR:
                    data.actuator(joint).ctrl = q
            except KeyError as e:
                print(f"[Simulator] Warning: joint '{joint}' not found in model, cannot set initial position.")
        reset_integrators = getattr(getattr(self, 'xbot2_bridge', None),
                                    'reset_velocity_integrators', None)
        if reset_integrators is not None:
            reset_integrators()
