from xbot2_py_bridge.bridge_server import BridgeServer, RobotCommand, JointCommand, RobotState, JointState, ImuState
from collections import deque
from dataclasses import dataclass
import math
import mujoco
import numpy as np
import time, struct

from xbot2_py_mujoco.imu import discover_imus, read_imu


@dataclass(frozen=True)
class _PendingCommand:
    deadline: float
    q: tuple[float, ...]
    dq: tuple[float, ...]
    tau: tuple[float, ...]
    k: tuple[float, ...]
    d: tuple[float, ...]

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

    def __init__(self, name: str, model: mujoco.MjModel, data: mujoco.MjData,
                 socket_path: str = None, actuator_delay_s: float = 0.0):

        self.set_actuator_delay(actuator_delay_s)
        self.initial_actuator_delay_s = self.actuator_delay_s
        self._pending_commands = deque()

        # save model and data
        self.model = model
        self.data = data

        # discover joints and imus from model
        self.joint_names = self._discover_joints(model)
        self.imu_map = discover_imus(model)
        self.imu_names = list(self.imu_map)
        self._joint_name_to_index = {name: i for i, name in enumerate(self.joint_names)}
        self.velocity_feedback_noise_std = np.zeros(len(self.joint_names))
        self._noise_rng = np.random.default_rng()

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
        self.velocity_integral = [
            a is not None and a.dyntype[0] == mujoco.mjtDyn.mjDYN_INTEGRATOR
            for a in self.model_act_handles
        ]

        # save last received command
        self.qref = [float(j.qpos[0]) if self.velocity_integral[i] else float(a.ctrl[0])
                     for i, (j, a) in enumerate(zip(self.joint_handles, self.act_handles))]
        self.vref = [0.0] * len(self.joint_names)
        self.tauref = [0.0] * len(self.joint_names)
        self.reset_velocity_integrators()

        # create bridge server
        self.bridge_server = BridgeServer(name, socket_path, self.joint_names, self.imu_names)

        # diagnostics
        self._last_stamp_us = 0.
        self._last_stamp_rx_feedback = 0
        self._rtt_acc = StatAccumulator()
        self._diag_pub = DiagPublisher(node_name='joint_mj/rtt', hw_id='xbot2_mujoco', throttle_publish_interval_sec=1.0)

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

        velocity = np.array([float(jh.qvel[0]) for jh in self.joint_handles])
        noisy = self.velocity_feedback_noise_std > 0
        if noisy.any():
            velocity[noisy] += self._noise_rng.normal(
                0.0, self.velocity_feedback_noise_std[noisy]
            )

        joint_state = JointState(
            q=[float(jh.qpos[0]) for jh in self.joint_handles],
            dq=velocity.tolist(),
            tau=[float(jh.qfrc_smooth[0]) for jh in self.joint_handles],
            k=[float(self.model_act_handles[i].gainprm[0]) for i in range(len(self.joint_names))],
            d=[float(-self.model_act_handles[i].biasprm[2]) for i in range(len(self.joint_names))],
            qref=[self.qref[i] if self.velocity_integral[i] else float(a.ctrl[0])
                  for i, a in enumerate(self.act_handles)],
            vref=self.vref,
            tauref=self.tauref
        )

        robot_state = RobotState(
            time=self.data.time,
            joints=joint_state,
            imus={n : self._get_imu_state(n) for n in self.imu_names}
        )

        self.bridge_server.send_state(robot_state)

    def receive(self):

        cmd = self.bridge_server.receive()

        if not cmd:
            self.apply_due_commands()
            return False

        if isinstance(cmd, bool):
            # Shared-memory mode: receive() only reports whether a fresh command
            # exists; the command arrays are exposed as zero-copy NumPy views.
            joint_cmd = self.bridge_server.command.joints
        else:
            # Legacy JSON mode: receive() returns a RobotCommand dataclass.
            joint_cmd: JointCommand = cmd.joint_command

        # Diagnostics: stamp_rx_feedback is the rx timestamp that the command
        # was computed from. Compare it with current sim time to estimate the
        # feedback-to-command roundtrip delay without perturbing joint values.
        stamp_rx_feedback = int(self.bridge_server.command.stamp_rx_feedback)
        if stamp_rx_feedback != 0:
            if stamp_rx_feedback != self._last_stamp_rx_feedback:
                self._last_stamp_rx_feedback = stamp_rx_feedback
                now_ns = int(self.data.time * 1e9)
                delay_us = (now_ns - stamp_rx_feedback) / 1e3
                self._rtt_acc.update(delay_us)
                self._diag_pub.publish_stats('rtt', self._rtt_acc)


        # Copy every field: shared-memory arrays are overwritten by later receives.
        self._pending_commands.append(_PendingCommand(
            deadline=float(self.data.time) + self.actuator_delay_s,
            q=tuple(float(value) for value in joint_cmd.q),
            dq=tuple(float(value) for value in joint_cmd.dq),
            tau=tuple(float(value) for value in joint_cmd.tau),
            k=tuple(float(value) for value in joint_cmd.k),
            d=tuple(float(value) for value in joint_cmd.d),
        ))
        self.apply_due_commands()
        return True

    def set_actuator_delay(self, seconds: float) -> float:
        if isinstance(seconds, bool) or not isinstance(seconds, (int, float)):
            raise ValueError("actuator delay must be finite and non-negative")
        seconds = float(seconds)
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("actuator delay must be finite and non-negative")
        self.actuator_delay_s = seconds
        return seconds

    def set_velocity_feedback_noise(self, joint_names: list[str], stddev: float) -> None:
        """Set per-joint Gaussian velocity feedback noise in joint units/s."""
        if isinstance(stddev, bool) or not isinstance(stddev, (int, float)) \
                or not math.isfinite(stddev) or stddev < 0:
            raise ValueError("velocity feedback noise stddev must be finite and non-negative")
        indices = [self._joint_name_to_index[name] for name in joint_names]
        self.velocity_feedback_noise_std[indices] = stddev

    def reset_velocity_feedback_noise(self) -> None:
        self.velocity_feedback_noise_std[:] = 0.0

    def reset_velocity_integrators(self):
        """Align integrated velocity targets with the current joint positions."""
        for i, (joint, act, model_act, is_integral) in enumerate(zip(
                self.joint_handles, self.act_handles, self.model_act_handles,
                self.velocity_integral)):
            if is_integral:
                self.data.act[int(model_act.actadr[0])] = joint.qpos[0]
                act.ctrl = 0.0
                self.qref[i] = float(joint.qpos[0])

    def apply_due_commands(self) -> bool:
        # Deadlines can be out of order after a live delay change. Never apply
        # an older command after a newer one has already reached the actuators.
        newest_due = None
        for index, command in enumerate(self._pending_commands):
            if command.deadline <= float(self.data.time) + 1e-12:
                newest_due = index
        if newest_due is None:
            return False
        command = self._pending_commands[newest_due]
        for _ in range(newest_due + 1):
            self._pending_commands.popleft()

        self.qref = list(command.q)
        self.vref = command.dq
        self.tauref = command.tau
        
        # A position actuator takes qref; intvelocity integrates the velocity
        # reference internally, so its control is vref.
        for i, act in enumerate(self.act_handles):
            if act is None:
                continue
            act.ctrl = command.dq[i] if self.velocity_integral[i] else command.q[i]

        # Position: tau = tau_ff + k*(qref-q) + d*(vref-v).
        # Intvelocity: tau = tau_ff + k*integral(vref-v) + d*(vref-v),
        # provided its activation was initialized to the joint position.
        # Both use the same affine gain/bias parameters; only ctrl differs.
        for i, model_act in enumerate(self.model_act_handles):
            if model_act is None:
                continue
            dq = command.dq[i]
            tau = command.tau[i]
            k = command.k[i]
            d = command.d[i]

            model_act.biasprm[0] = d*dq + tau  # feedforward 
            model_act.biasprm[1] = -k 
            model_act.biasprm[2] = -d
            model_act.gainprm[0] = k

        return True

    def close(self):
        """Release bridge and diagnostics transports before interpreter shutdown."""
        if self._diag_pub is not None:
            close = getattr(self._diag_pub, 'close', None)
            if close is not None:
                close()
            self._diag_pub = None
        if self.bridge_server is not None:
            self.bridge_server.close()
            self.bridge_server = None

    def _get_imu_state(self, imu_name: str) -> ImuState:
        quat_w, ang_vel_b, lin_acc_b = read_imu(self.model, self.data, self.imu_map[imu_name])
        # wxyz to xyzw
        return ImuState(quat_w=np.roll(quat_w, -1), lin_acc_b=lin_acc_b, ang_vel_b=ang_vel_b)

    @staticmethod
    def _discover_joints(model: mujoco.MjModel) -> list:
        # Tuple membership invokes enum.__eq__(numpy_scalar), which returns
        # False in MuJoCo 3.15 even for matching values. Compare plain integers.
        scalar_types = (int(mujoco.mjtJoint.mjJNT_HINGE),
                        int(mujoco.mjtJoint.mjJNT_SLIDE))
        return [model.joint(jnt).name for jnt in range(model.njnt)
                if int(model.jnt_type[jnt]) in scalar_types]
