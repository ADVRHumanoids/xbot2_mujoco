import os
from xbot2_py_mujoco.mjcf_tools import MjcfGenerator

KYON_URDF = '/home/alaurenzi/code/ros2_ws/src/iit-kyon-ros-pkg/kyon_urdf/urdf/kyon.urdf'


def test_construct_from_kyon_urdf():
    with open(KYON_URDF, 'r') as f:
        urdf_str = f.read()
    gen = MjcfGenerator(name='kyon', urdf_str=urdf_str)
    assert gen.urdf_orig == urdf_str
    assert gen.urdf_processed is not None
