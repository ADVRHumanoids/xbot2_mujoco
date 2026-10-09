import mujoco
import pytest

from xbot2_py_mujoco.mj_xbot2_bridge import MjXbot2Bridge


@pytest.mark.parametrize('free_name', ['', 'name="reference"'])
def test_joint_discovery_preserves_scalar_joint_order(free_name):
    model = mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody>
      <body><freejoint {free_name}/><geom size=".1"/>
        <body><joint name="hip" type="hinge"/><geom size=".1"/></body>
        <body><joint name="slider" type="slide"/><geom size=".1"/></body>
        <body><joint name="ball" type="ball"/><geom size=".1"/></body>
        <body><joint name="knee" type="hinge"/><geom size=".1"/></body>
      </body>
    </worldbody></mujoco>''')
    assert MjXbot2Bridge._discover_joints(model) == ['hip', 'slider', 'knee']


def test_joint_discovery_without_joints():
    model = mujoco.MjModel.from_xml_string('<mujoco/>')
    assert MjXbot2Bridge._discover_joints(model) == []
