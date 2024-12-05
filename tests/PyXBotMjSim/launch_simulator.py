import time
import numpy as np
import argparse
import rospy
from rosgraph_msgs.msg import Clock
from xbot2_mujoco.PyXbotMjSim import XBotMjSim
from xbot2_mujoco.PyXbotMjSim import LoadingUtils


class SimulatorLauncher:
    def __init__(self, args):
        self.args = args
        self.clock_publisher = None

        if self.args.pub_rostime:
            rospy.init_node('sim_clock_publisher', anonymous=True)
            self.clock_publisher = rospy.Publisher('/clock', Clock, queue_size=10)

        # Initialize the LoadingUtils instance
        self.loader = LoadingUtils("XMjEnvPy")
        files_dir = self.args.files_dir or "/root/ibrido_ws/src/xbot2_mujoco/tests/files"

        self.loader.set_urdf_path(self.args.urdf_path or f"{files_dir}/centauro.urdf")
        self.loader.set_simopt_path(self.args.simopt_path or f"{files_dir}/sim_opt.xml")
        self.loader.set_world_path(self.args.world_path or f"{files_dir}/world.xml")
        self.loader.set_sites_path(self.args.sites_path or f"{files_dir}/sites.xml")
        self.loader.set_xbot_config_path(self.args.xbot_config_path or f"{files_dir}/xbot2_basic.yaml")
        self.loader.generate()

        mj_xml_path = self.loader.xml_path()

        # Initialize the XBotMjSim environment
        self.sim = XBotMjSim(
            model_fname=mj_xml_path,
            xbot2_config_path=self.args.xbot_config_path or f"{files_dir}/xbot2_basic.yaml",
            headless=self.args.headless,
            manual_stepping=True,
            init_steps=100,
            timeout=1000
        )

    def run(self):
        np.set_printoptions(precision=2, 
                            linewidth=200)

        start_time = time.time()  # Track total time
        stepping_time = 0.0       # Track only time spent stepping
        n_steps_done = 0
        db_stepfreq = 1000  # Frequency to print and update RT factor
        initial_step_counter = self.sim.step_counter
        
        ros_clock_freq=100

        jnt_names = self.sim.jnt_names()
        print("\nControllable joint names: ->\n")
        print(", ".join(jnt_names))

        while self.sim.is_running():
            step_start = time.time()  # Start timing the step
            if not self.sim.step():
                break
            step_end = time.time()  # End timing the step
            stepping_time += (step_end - step_start)  # Accumulate stepping time
            n_steps_done += 1

            # Publish simulation time to /clock if enabled
            if self.clock_publisher and (n_steps_done % ros_clock_freq == 0):
                simtime_elapsed = rospy.Time.from_sec(self.sim.physics_dt * self.sim.step_counter)
                clock_msg = Clock(clock=simtime_elapsed)
                self.clock_publisher.publish(clock_msg)

            # Update and print RT factor every db_stepfreq steps
            if n_steps_done % db_stepfreq == 0:
                elapsed_time = time.time() - start_time  # Total elapsed wall time so far
                simtime_elapsed = self.sim.physics_dt * self.sim.step_counter
                rt_factor = simtime_elapsed / elapsed_time if elapsed_time > 0 else float('inf')
                print(f"RT Factor at step {n_steps_done}: {rt_factor:.2f}, Simulated Time: {simtime_elapsed:.6f}s, Elapsed Time: {elapsed_time:.6f}s")

        # Final stats after simulation ends
        total_elapsed_time = time.time() - start_time  # Total wall time (including everything)
        total_steps_done = self.sim.step_counter - initial_step_counter
        simtime_elapsed = self.sim.physics_dt * total_steps_done
        stepping_rt_factor = simtime_elapsed / stepping_time if stepping_time > 0 else float('inf')

        print("\nSimulation Finished:")
        print(f"Number of timesteps done: {total_steps_done}")
        print(f"Total elapsed wall time: {total_elapsed_time:.6f} seconds (including all operations).")
        print(f"Actual stepping time: {stepping_time:.6f} seconds.")
        print(f"Simulated time: {simtime_elapsed:.6f} seconds.")
        print(f"RT factor (total elapsed time): {simtime_elapsed / total_elapsed_time:.2f}")
        print(f"RT factor (actual stepping time): {stepping_rt_factor:.2f}, physics dt: {self.sim.physics_dt:.6f} seconds.")

    def close(self):
        self.sim.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Launch XBotMjSimEnv simulation.')

    # File paths
    parser.add_argument('--files_dir', type=str, help='Base directory for simulation files.')
    parser.add_argument('--urdf_path', type=str, help='Path to the URDF file.')
    parser.add_argument('--simopt_path', type=str, help='Path to the simulation options file.')
    parser.add_argument('--world_path', type=str, help='Path to the world file.')
    parser.add_argument('--sites_path', type=str, help='Path to the sites file.')
    parser.add_argument('--xbot_config_path', type=str, help='Path to the XBot2 configuration file.')

    # Simulation parameters
    parser.add_argument('--headless', action='store_true', help='Run the simulation in headless mode.')
    parser.add_argument('--pub_rostime', action='store_true', help='Publish simulation time to the /clock topic.')
    parser.add_argument('--physics_dt', type=float, default=1e-3, help='Physics time step.')

    args = parser.parse_args()

    simulator = SimulatorLauncher(args)
    try:
        simulator.run()
    finally:
        simulator.close()


