import unittest
import time
import numpy as np
import random
import argparse

from xbot2_mujoco.PyXbotMjSim import XBotMjSim
from xbot2_mujoco.PyXbotMjSim import LoadingUtils

import rospy

class TestSimStepping(unittest.TestCase):

    @staticmethod
    def quaternion_from_rotation_z(theta_degrees):
        theta_radians = np.deg2rad(theta_degrees)
        w = np.cos(theta_radians / 2.0)
        x = 0.0
        y = 0.0
        z = np.sin(theta_radians / 2.0)
        return [w, x, y, z]

    def setUp(self):
        robot_name = self.args.robot_name  # Get robot name from arguments
        blink_name = self.args.blink_name
        headless= self.args.headless
        manual_stepping= not self.args.auto_stepping

        self.loader = LoadingUtils("XMjEnvPyTest")
        FILES_DIR = "/root/ibrido_ws/src/xbot2_mujoco/tests/files"
        FILES_DIR=FILES_DIR+"/"+robot_name
        
        self.loader.set_urdf_path(f"{FILES_DIR}/{robot_name}.urdf")
        self.loader.set_simopt_path(f"{FILES_DIR}/sim_opt.xml")
        self.loader.set_world_path(f"{FILES_DIR}/world.xml")
        self.loader.set_sites_path(f"{FILES_DIR}/sites.xml")
        self.loader.set_xbot_config_path(f"{FILES_DIR}/xbot2_basic.yaml")
        self.loader.generate()

        mj_xml_path = self.loader.xml_path()

        self._xmj_sim = XBotMjSim(
            model_fname=mj_xml_path,
            xbot2_config_path=f"{FILES_DIR}/xbot2_basic.yaml",
            headless=headless,
            manual_stepping=manual_stepping,
            init_steps=100,
            timeout=1000,
            base_link_name=blink_name
        )

    def tearDown(self):
        self._xmj_sim.close()

    def test_sim_stepping(self):
        np.set_printoptions(precision=2, linewidth=200)

        n_steps = 20000
        reset_freq = 1000
        state_print_freq = 400
        start_time = time.time()
        stepping_time = 0.0
        n_steps_done = 0

        actual_done = self._xmj_sim.step_counter
        jnt_names = self._xmj_sim.jnt_names()
        print("\nControllable joint names: ->\n", ", ".join(jnt_names))
        pi = np.zeros((3))
        qi = np.zeros((4))
        qi[0] = 1  # Quaternion identity
        pi[2] = self._xmj_sim.get_pi()[2]

        for i in range(n_steps):
            step_start = time.time()
            if not self._xmj_sim.step():
                break
            step_end = time.time()
            stepping_time += (step_end - step_start)

            if (i + 1) % state_print_freq == 0:
                print(f"\n########## MEAS STATE DUMP ({self._xmj_sim.step_counter+1}/{n_steps}) ############\n")
                print("p:", self._xmj_sim.p)
                print("q:", self._xmj_sim.q)
                print("v:", self._xmj_sim.twist[0:3])
                print("omega:", self._xmj_sim.twist[3:6])
                print(", ".join(jnt_names))
                print("jnts q:", self._xmj_sim.jnts_q)
                print("jnts v:", self._xmj_sim.jnts_v)
                print("jnts a:", self._xmj_sim.jnts_a)
                print("jnts eff:", self._xmj_sim.jnts_eff)

            if (i + 1) % reset_freq == 0:
                pi[0] += random.uniform(-1.0, 1.0)
                pi[1] += random.uniform(-1.0, 1.0)
                random_theta = random.uniform(-180.0, 180.0)
                qi[:] = self.quaternion_from_rotation_z(random_theta)
                self._xmj_sim.set_pi(pi)
                self._xmj_sim.set_qi(qi)
                self._xmj_sim.reset()

            n_steps_done += 1

        total_elapsed_time = time.time() - start_time
        actual_done = self._xmj_sim.step_counter - actual_done
        physics_dt = self._xmj_sim.physics_dt
        simtime_elapsed = physics_dt * actual_done
        stepping_rt_factor = simtime_elapsed / stepping_time

        print(f"[test_xmj_env][ManualSteppingTest]: n of timesteps done {actual_done} VS {n_steps}")
        print(f"[test_xmj_env][ManualSteppingTest]: total elapsed wall time {total_elapsed_time:.6f} [s].")
        print(f"[test_xmj_env][ManualSteppingTest]: actual stepping time {stepping_time:.6f} [s].")
        print(f"[test_xmj_env][ManualSteppingTest]: simulated time {simtime_elapsed:.6f} [s].")
        print(f"RT factor based on total elapsed time: {simtime_elapsed / total_elapsed_time:.2f}")
        print(f"RT factor based on actual stepping time: {stepping_rt_factor:.2f}, physics dt {physics_dt:.6f} [s]")

        self.assertEqual(actual_done, n_steps)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run XBotMjSim tests with specified parameters.')
    parser.add_argument('--robot_name', type=str, default='centauro', help='Specify the robot name')
    parser.add_argument('--blink_name', type=str, default='base_link', help='robot root link (used to move the robot around)')
    parser.add_argument('--headless', action="store_true")
    parser.add_argument('--auto_stepping', action="store_true")

    args, unknown = parser.parse_known_args()  # Allow unknown arguments for unittest

    # Bind arguments to the test case
    TestSimStepping.args = args

    unittest.main(argv=['first-arg-is-ignored'], exit=False)