import os
from pathlib import Path
from threading import Thread
import time

from xbot2_interface import pyxbot2_interface as xb
from xbot2.pyxbot2 import XBot2Executor, getRobot
from xbot2_py_mujoco.mjcf_tools import MjcfGenerator
from xbot2_py_mujoco.simulator_wrapper import SimulatorWrapper

def test_xbot2_bridge():

    script_dir = Path(__file__).parent
    urdf_str = (script_dir / 'resources' / 'nice_robot.urdf').read_text()
    config_xml_str = (script_dir / 'resources' / 'config.xml').read_text()
    world_xml_str = (script_dir / 'resources' / 'world.xml').read_text()

    os.environ['RESOURCE_DIR'] = str(script_dir / 'resources')

    # generate mj model and data from urdf and xml
    gen = MjcfGenerator(name='quadruped', urdf_str=urdf_str)
    gen.merge_xml(config_xml_str)
    gen.merge_xml(world_xml_str)
    model, _ = gen.create_mj_model_data()

    # create simulator wrapper
    sim = SimulatorWrapper(model=model,
                        q_init=gen.q_init,
                        viewer=False,
                        ros=False,
                        socket_path='/tmp/mj_simulator_server.sock')

    # note: we need to keep the simulator running during the initialization of the xbot2 executor
    warmup_sim = True
    def spin_simulator():
        while warmup_sim:
            print("Warming up simulator...")
            sim.pre_step()
            time.sleep(0.1)
            
    sim_thread = Thread(target=spin_simulator)
    sim_thread.start()

    # create xbot2 executor
    exe = XBot2Executor(cfg_path=(script_dir / 'resources' / 'xbot2_config.yaml').as_posix(),
                        hw_type='mj',
                        verbose=False,)

    # stop the simulator warmup
    warmup_sim = False
    sim_thread.join()

    # get robot
    robot = getRobot()
    robot.setControlMode(xb.ControlMode.POSITION)
    joint_names = robot.getJointNames()[1:] # skip the first joint which is the floating base

    # run sim and xbot2
    print('>>>> Let xbot2 transition to running state...')
    for _ in range(10):
        sim.run()
        exe.run(False, False)

    # update robot state
    assert robot.sense(), "Failed to sense robot state"

    # get position reference
    qref = robot.qToMap(robot.getPositionReferenceFeedback())

    # it must be close to the initial position defined in the xml
    for jn in joint_names:
        assert abs(qref[jn] - gen.q_init[jn]) < 1e-3, f"qref for joint {jn} is not close to initial position: {qref[jn]} vs {gen.q_init[jn]}"

    # set position reference to a new value
    qref_new = robot.getPositionReferenceFeedback() + 0.001
    robot.setPositionReference(qref_new)
    robot.move()

    #
    exe.run(False, False)  # send the new position reference to the simulator
    sim.run() # receive the new position reference in the simulator and apply it
    exe.run(False, False) # update robot state after applying the new position reference

    # check if the new position reference is applied
    assert robot.sense(), "Failed to sense robot state after setting new position reference"
    qref_after = robot.qToMap(robot.getPositionReferenceFeedback())
    qref_new = robot.qToMap(qref_new)
    for jn in joint_names:
        assert abs(qref_after[jn] - qref_new[jn]) < 1e-3, f"qref for joint {jn} is not close to new position reference: {qref_after[jn]} vs {qref_new[jn]}"

