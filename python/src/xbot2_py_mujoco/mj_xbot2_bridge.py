from xbot2_py_bridge.bridge_server import BridgeServer, RobotCommand, JointCommand, RobotState, JointState, ImuState
import mujoco
import numpy as np
import time, struct

try:
    from pyxbot2_diagnostics.stats_accumulator import StatAccumulator
    from pyxbot2_diagnostics.publisher import DiagPublisher
except ImportError:
    # dummy diagnostics if pyxbot2_diagnostics is not available
    class StatAccumulator:
        def update(self, value):
            pass

    class DiagPublisher:
        def __init__(self, *args, **kwargs):
            pass
        def publish_stats(self, *args, **kwargs):
            pass


class MjXbot2Bridge:

    def __init__(self, name: str, model: mujoco.MjModel, data: mujoco.MjData, socket_path: str = None, ):

        # save model and data
        self.model = model
        self.data = data

        # discover joints and imus from model
        self.joint_names = self._discover_joints(model)
        self.imu_map, self.imu_names = self._discover_imus(model)

        # helper to get joint/actuator handles, returns None if not found
        def _or_none(fn, *args):
            try:
                return fn(*args)
            except Exception as e:
                return None

        # save joint handles for quick access
        self.joint_handles = [self.data.joint(jn) for jn in self.joint_names]
        self.act_handles = [_or_none(self.data.actuator, jn) for jn in self.joint_names]
        self.model_act_handles = [_or_none(self.model.actuator, jn) for jn in self.joint_names]

        # save last received command
        self.qref = [float(a.ctrl[0]) for a in self.act_handles]
        self.vref = [0.0] * len(self.joint_names)
        self.tauref = [0.0] * len(self.joint_names)

        # create bridge server
        self.bridge_server = BridgeServer(name, socket_path, self.joint_names, self.imu_names)

        # diagnostics
        self._last_stamp_us = 0.
        self._rtt_acc = StatAccumulator()
        self._diag_pub = DiagPublisher(node_name='joint_mj', hw_id='xbot2_mujoco', throttle_publish_interval_sec=1.0)

    def _encode_timestamp(self, link_pos: float, stamp_us: int) -> float:
        """Patch 20 LSBs of link_pos with current monotonic microseconds."""
        bits = struct.unpack('<Q', struct.pack('<d', link_pos))[0]
        bits = (bits & ~0xFFFFF) | stamp_us
        return struct.unpack('<d', struct.pack('<Q', bits))[0]

    def _decode_comm_delay_us(self, link_pos: float, now: float) -> tuple[int, int]:
        """Extract hidden microsecond timestamp from 20 LSBs of link_pos and return comm delay in µs."""
        bits = struct.unpack('<Q', struct.pack('<d', link_pos))[0]
        stamp_us = bits & 0xFFFFF
        now_us = int(now * 1e6) & 0xFFFFF
        delay_us = (now_us - stamp_us) & 0xFFFFF
        if delay_us > (1 << 19):
            delay_us -= (1 << 20)  # signed wraparound correction
        return delay_us, stamp_us

    def send_state(self):

        now = time.monotonic()
        now_us = int(now * 1e6) & 0xFFFFF

        joint_state = JointState(
            q=[float(jh.qpos[0]) for jh in self.joint_handles],
            dq=[float(jh.qvel[0]) for jh in self.joint_handles],
            tau=[float(jh.qfrc_smooth[0]) for jh in self.joint_handles],
            k=[float(self.model_act_handles[i].gainprm[0]) for i in range(len(self.joint_names))],
            d=[float(-self.model_act_handles[i].biasprm[2]) for i in range(len(self.joint_names))],
            qref=[float(a.ctrl[0]) for a in self.act_handles],
            vref=self.vref,
            tauref=self.tauref
        )

        ## diagnostics 
        # patch last 20 bits of each q with current time's microseconds 
        for i, _ in enumerate(self.joint_handles):
            joint_state.q[i] = self._encode_timestamp(joint_state.q[i], now_us)


        robot_state = RobotState(
            time=self.data.time,
            joints=joint_state,
            imus={n : self._get_imu_state(n) for n in self.imu_names}
        )

        self.bridge_server.send_state(robot_state)

    def receive(self):

        cmd = self.bridge_server.receive()

        if not cmd:
            return False

        if isinstance(cmd, bool):
            # Shared-memory mode: receive() only reports whether a fresh command
            # exists; the command arrays are exposed as zero-copy NumPy views.
            joint_cmd = self.bridge_server.command.joints
        else:
            # Legacy JSON mode: receive() returns a RobotCommand dataclass.
            joint_cmd: JointCommand = cmd.joint_command

        ## diagnostics
        # decode communication delay from q values and print it
        now = time.monotonic()
        for i, _ in enumerate(self.joint_handles):
            if self.joint_names[i] == 'hip_roll_1':
                delay_us, stamp_us = self._decode_comm_delay_us(joint_cmd.q[i], now)
                if stamp_us != self._last_stamp_us:
                    self._last_stamp_us = stamp_us
                    self._rtt_acc.update(delay_us)
                    self._diag_pub.publish_stats('rtt', self._rtt_acc)


        self.qref = joint_cmd.q
        self.vref = joint_cmd.dq
        self.tauref = joint_cmd.tau
        
        # apply qref as actuator's ctrl (if exists)
        for i, act in enumerate(self.act_handles):
            if act is None:
                continue
            act.ctrl = joint_cmd.q[i]

        # apply v, tau, k and d as actuator model parameters 
        # see https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-position
        # we want to achieve tau = tau_ff +k*(qref - q) + d*(vref - dq) => 
        #   biasprm[0] = d*dq + tau, gainprm[0] = k, biasprm[1] = -k, biasprm[2] = -d
        for i, model_act in enumerate(self.model_act_handles):
            if model_act is None:
                continue
            dq = joint_cmd.dq[i]
            tau = joint_cmd.tau[i]
            k = joint_cmd.k[i]
            d = joint_cmd.d[i]

            model_act.biasprm[0] = d*dq + tau  # feedforward 
            model_act.biasprm[1] = -k 
            model_act.biasprm[2] = -d
            model_act.gainprm[0] = k

        return True
    
    def _get_imu_state(self, imu_name: str) -> ImuState:
        imu_ids = self.imu_map[imu_name]
        gyro_id = imu_ids['gyro_id']
        acc_id = imu_ids['acc_id']
        site_id = imu_ids['site_id']

        quat_w = np.empty(4)
        mujoco.mju_mat2Quat(quat_w, self.data.site(site_id).xmat)
        # wxyz to xyzw
        quat_w = np.roll(quat_w, -1)
        lin_acc_b = self.data.sensordata[acc_id*3:(acc_id+1)*3]
        ang_vel_b = self.data.sensordata[gyro_id*3:(gyro_id+1)*3]

        return ImuState(quat_w=quat_w, lin_acc_b=lin_acc_b, ang_vel_b=ang_vel_b)

    @staticmethod
    def _discover_joints(model: mujoco.MjModel) -> list:
        joint_names = [model.joint(jnt).name for jnt in range(model.njnt)]
        return [jn for jn in joint_names
                if model.joint(jn).type[0] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)]

    @staticmethod
    def _discover_imus(model: mujoco.MjModel) -> tuple:
        site_imu_map = {}
        for i in range(model.nsensor):
            if model.sensor_objtype[i] != mujoco.mjtObj.mjOBJ_SITE:
                continue
            site_id = model.sensor_objid[i]
            site_name = model.site(site_id).name
            if model.sensor_type[i] == mujoco.mjtSensor.mjSENS_GYRO:
                site_imu_map.setdefault(site_name, {})['gyro_id'] = i
                site_imu_map[site_name]['site_id'] = site_id
            elif model.sensor_type[i] == mujoco.mjtSensor.mjSENS_ACCELEROMETER:
                site_imu_map.setdefault(site_name, {})['acc_id'] = i
                site_imu_map[site_name]['site_id'] = site_id
        imu_map = {name: ids for name, ids in site_imu_map.items()
                   if 'gyro_id' in ids and 'acc_id' in ids}
        return imu_map, list(imu_map.keys())
