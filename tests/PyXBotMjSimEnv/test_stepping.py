import unittest
import time
import numpy as np
import random
import argparse

from xbot2_mujoco.PyXbotMjSimEnv import XBotMjSimEnv
from xbot2_mujoco.PyXbotMjSimEnv import LoadingUtils

import rospy

class TestSimStepping(unittest.TestCase):

    @staticmethod
    def quaternion_from_rotation_z(theta_degrees):
        # Convert theta from degrees to radians
        theta_radians = np.deg2rad(theta_degrees)

        # Calculate the quaternion components for z-axis rotation
        w = np.cos(theta_radians / 2.0)
        x = 0.0
        y = 0.0
        z = np.sin(theta_radians / 2.0)
        return [w, x, y, z]

    def setUp(self):
        # Set up the LoadingUtils instance using command-line arguments
        self.loader = LoadingUtils("XMjEnvPyTest")

        files_dir = "/root/ibrido_ws/src/xbot2_mujoco/tests/files"

        # optonally set a custom root dir
        # mesh_root_directory = "/root/ibrido_ws/src/iit-centauro-ros-pkg/centauro_urdf/meshes"
        # subdirs = ["v2", "realsense", "simple"]
        # self.loader.set_mesh_rootdir(mesh_root_directory)
        # self.loader.set_mesh_rootdir_subdirs(subdirs)
        self.loader.set_urdf_path(f"{files_dir}/centauro.urdf")
        self.loader.set_simopt_path(f"{files_dir}/sim_opt.xml")
        self.loader.set_world_path(f"{files_dir}/world.xml")
        self.loader.set_sites_path(f"{files_dir}/sites.xml")
        self.loader.set_xbot_config_path(f"{files_dir}/xbot2_basic.yaml")
        self.loader.generate()

        mj_xml_path = self.loader.xml_path()
        # mj_xml_path = "/tmp/XMjEnvTest_mujoco/XMjEnvTest.mjcf"

        # Create the XBotMjSimEnv instance
        self._xmj_env = XBotMjSimEnv(
            model_fname=mj_xml_path,
            xbot2_config_path=f"{files_dir}/xbot2_basic.yaml",
            headless=False,
            manual_stepping=True,
            init_steps=100,
            timeout=1000
        )

    def tearDown(self):
        self._xmj_env.close()

    def test_sim_stepping(self):
        np.set_printoptions(precision=2, 
            threshold=None, 
            edgeitems=None, 
            linewidth=200, 
            suppress=None,
            nanstr=None,
            infstr=None, 
            formatter=None, 
            sign=None, 
            floatmode=None, 
            legacy=None)

        n_steps = 200000
        reset_freq = 1000
        state_print_freq = 200
        start_time = time.time()
        n_steps_done = 0

        actual_done = self._xmj_env.step_counter
        
        jnt_names=self._xmj_env.jnt_names()
        print("\nControllable joint names: ->\n")
        print(", ".join(jnt_names))
        pi=np.zeros((3))
        qi=np.zeros((4))
        qi[0] = 1
        pi[2]=self._xmj_env.get_pi()[2]

        for i in range(n_steps):
            if not self._xmj_env.step():
                break
            
            if (i + 1) % state_print_freq == 0:
                print("\n########## MEAS STATE DUMP ############\n p")
                print(self._xmj_env.p)
                print("q")
                print(self._xmj_env.q)
                print("v")
                print(self._xmj_env.twist[0:3])
                print("omega")
                print(self._xmj_env.twist[3:6])
                print(", ".join(jnt_names))
                print("jnts q")
                print(self._xmj_env.jnts_q)
                print("jnts v")
                print(self._xmj_env.jnts_v)
                print("jnts a")
                print(self._xmj_env.jnts_a)
                print("jnts eff")
                print(self._xmj_env.jnts_eff)

            if (i + 1) % reset_freq == 0:
                # Update position and orientation
                pi[0] += random.uniform(-1.0, 1.0)
                pi[1] += random.uniform(-1.0, 1.0)
                random_theta = random.uniform(-180.0, 180.0)
                qi[:] = self.quaternion_from_rotation_z(random_theta)
                self._xmj_env.set_pi(pi)
                self._xmj_env.set_qi(qi)
                self._xmj_env.reset()
            
            n_steps_done += 1

        elapsed_time = time.time() - start_time

        # Compute simulated time
        actual_done = self._xmj_env.step_counter-actual_done
        physics_dt = self._xmj_env.physics_dt
        simtime_elapsed = physics_dt * actual_done

        print(f"[test_xmj_env][ManualSteppingTest]: n of timesteps done {actual_done} VS {n_steps}")
        print(f"[test_xmj_env][ManualSteppingTest]: elapsed wall time {elapsed_time:.6f} [s] VS simulated time {simtime_elapsed:.6f} [s].")
        print(f"RT factor {simtime_elapsed / elapsed_time:.2f}, physics dt {physics_dt:.6f} [s]")

        self.assertEqual(actual_done, n_steps)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Run XBotMjSimEnv tests with specified parameters.')

    args = parser.parse_args()

    # Bind arguments to the test case
    TestSimStepping.args = args

    unittest.main()
