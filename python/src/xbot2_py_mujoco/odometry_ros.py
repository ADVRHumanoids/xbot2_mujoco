"""Ground-truth root-body odometry; imported only when ROS is enabled."""

import mujoco
import numpy as np
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry


class OdometryPublisher:
    def __init__(self, node, model: mujoco.MjModel):
        self.model = model
        self.roots = []
        for body_id in range(1, model.nbody):
            if (model.body_parentid[body_id] == 0
                    and model.body_dofnum[body_id] > 0):
                name = model.body(body_id).name or f'body_{body_id}'
                publisher = node.create_publisher(Odometry, f'/{name}/odom', 1)
                self.roots.append((body_id, name, publisher))
        self.data = mujoco.MjData(model)

    def publish(self, data: mujoco.MjData):
        if not self.roots:
            return
        # mj_step advances qpos/qvel beyond its cached Cartesian quantities.
        # Refresh only kinematics, on private data to leave physics untouched.
        snapshot = self.data
        snapshot.qpos[:] = data.qpos
        snapshot.qvel[:] = data.qvel
        snapshot.mocap_pos[:] = data.mocap_pos
        snapshot.mocap_quat[:] = data.mocap_quat
        mujoco.mj_kinematics(self.model, snapshot)
        mujoco.mj_comPos(self.model, snapshot)
        mujoco.mj_comVel(self.model, snapshot)
        seconds, nanoseconds = divmod(round(data.time * 1e9), 1_000_000_000)
        velocity = np.empty(6)
        for body_id, name, publisher in self.roots:
            msg = Odometry()
            msg.header.stamp = Time(sec=seconds, nanosec=nanoseconds)
            msg.header.frame_id = 'world'
            msg.child_frame_id = name
            position = snapshot.xpos[body_id]
            quaternion = snapshot.xquat[body_id]  # MuJoCo uses w, x, y, z.
            msg.pose.pose.position.x = float(position[0])
            msg.pose.pose.position.y = float(position[1])
            msg.pose.pose.position.z = float(position[2])
            msg.pose.pose.orientation.w = float(quaternion[0])
            msg.pose.pose.orientation.x = float(quaternion[1])
            msg.pose.pose.orientation.y = float(quaternion[2])
            msg.pose.pose.orientation.z = float(quaternion[3])
            # XBODY selects the body origin; BODY would select its COM.
            # The result is angular then linear velocity in the local frame.
            mujoco.mj_objectVelocity(self.model, snapshot,
                                    mujoco.mjtObj.mjOBJ_XBODY, body_id, velocity, 1)
            msg.twist.twist.angular.x = float(velocity[0])
            msg.twist.twist.angular.y = float(velocity[1])
            msg.twist.twist.angular.z = float(velocity[2])
            msg.twist.twist.linear.x = float(velocity[3])
            msg.twist.twist.linear.y = float(velocity[4])
            msg.twist.twist.linear.z = float(velocity[5])
            publisher.publish(msg)
