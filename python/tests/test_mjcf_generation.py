import pytest
from pathlib import Path
from lxml import etree
import mujoco

from xbot2_py_mujoco.mjcf_tools import MjcfGenerator

RESOURCES = Path(__file__).parent / 'resources'
URDF_PATH = RESOURCES / 'nice_robot.urdf'
CONFIG_XML_PATH = RESOURCES / 'config.xml'
WORLD_XML_PATH = RESOURCES / 'world.xml'


@pytest.fixture(scope='module')
def urdf_str():
    return URDF_PATH.read_text()

@pytest.fixture(scope='module')
def world_xml_str():
    return WORLD_XML_PATH.read_text()


@pytest.fixture(scope='module')
def config_xml_str():
    return CONFIG_XML_PATH.read_text()


@pytest.fixture
def gen(urdf_str):
    """Basic generator from URDF only."""
    return MjcfGenerator(name='quadruped', urdf_str=urdf_str)


@pytest.fixture
def gen_with_config(urdf_str, config_xml_str):
    """Generator with config.xml merged in."""
    g = MjcfGenerator(name='quadruped', urdf_str=urdf_str)
    g.merge_xml(config_xml_str)
    return g


@pytest.fixture
def gen_finalized(urdf_str, config_xml_str, world_xml_str):
    """Generator with config.xml merged and _finalize_xml already applied."""
    g = MjcfGenerator(name='quadruped', urdf_str=urdf_str)
    g.merge_xml(config_xml_str)
    g.merge_xml(world_xml_str)
    g._finalize_xml()
    return g


# ---------------------------------------------------------------------------
# Construction
# ---------------------------------------------------------------------------

def test_generator_constructs(gen):
    assert gen is not None
    assert gen.mj_xml_tree is not None


def test_generator_has_worldbody(gen):
    worldbody = gen.mj_xml_tree.xpath('//worldbody')
    assert worldbody, 'Generated XML should contain a <worldbody>'


def test_generator_root_body_present(gen):
    base = gen.mj_xml_tree.xpath('//body[@name="base_link"]')
    assert base, 'Robot root body "base_link" should be present'


# ---------------------------------------------------------------------------
# merge_xml
# ---------------------------------------------------------------------------

def test_merge_xml_adds_default_class(gen_with_config):
    default_leg = gen_with_config.mj_xml_tree.xpath('//default[@class="robot_leg"]')
    assert default_leg, '<default class="robot_leg"> should be present after merge'


def test_merge_xml_adds_actuators(gen_with_config):
    actuators = gen_with_config.mj_xml_tree.xpath('//actuator/position')
    assert len(actuators) > 0, 'Actuators should be present after merge'


def test_merge_xml_adds_sensor(gen_with_config):
    sensors = gen_with_config.mj_xml_tree.xpath('//sensor/accelerometer')
    assert sensors, 'Accelerometer sensor should be present after merge'


# ---------------------------------------------------------------------------
# _finalize_xml: <reference> processing
# ---------------------------------------------------------------------------

def test_finalize_reference_inserts_site(gen_finalized):
    base_link = gen_finalized.mj_xml_tree.xpath('//body[@name="base_link"]')
    assert base_link, '"base_link" body not found'
    sites = base_link[0].xpath('./site[@name="base_link"]')
    assert sites, '<site name="base_link"/> should have been inserted into body "base_link"'


def test_finalize_reference_tag_removed(gen_finalized):
    mujoco_root = gen_finalized.mj_xml_tree.xpath('/mujoco')[0]
    refs = mujoco_root.findall('reference')
    assert not refs, '<reference> tags should be removed from the root after finalization'


# ---------------------------------------------------------------------------
# _finalize_xml: <setattribute> processing
# ---------------------------------------------------------------------------

def test_finalize_setattribute_sets_childclass(gen_finalized):
    for leg in ('fl_hip_link', 'fr_hip_link', 'rl_hip_link', 'rr_hip_link'):
        body = gen_finalized.mj_xml_tree.xpath(f'//body[@name="{leg}"]')
        assert body, f'Body "{leg}" not found'
        assert body[0].get('childclass') == 'robot_leg', \
            f'childclass="robot_leg" should be set on body "{leg}"'


def test_finalize_setattribute_tag_removed(gen_finalized):
    mujoco_root = gen_finalized.mj_xml_tree.xpath('/mujoco')[0]
    setattrs = mujoco_root.findall('setattribute')
    assert not setattrs, '<setattribute> tags should be removed from the root after finalization'


# ---------------------------------------------------------------------------
# _finalize_xml: <q_init> processing
# ---------------------------------------------------------------------------

def test_finalize_q_init_populates_dict(gen_finalized):
    assert len(gen_finalized.q_init) > 0, 'q_init dict should be populated after finalization'
    assert 'fl_hip' in gen_finalized.q_init
    assert pytest.approx(gen_finalized.q_init['fl_hip'], abs=1e-6) == 0.15


def test_finalize_q_init_tag_removed(gen_finalized):
    mujoco_root = gen_finalized.mj_xml_tree.xpath('/mujoco')[0]
    q_inits = mujoco_root.findall('q_init')
    assert not q_inits, '<q_init> tags should be removed from the root after finalization'


# ---------------------------------------------------------------------------
# generate_mjcf_string
# ---------------------------------------------------------------------------

def test_generate_mjcf_string_returns_string(gen_with_config):
    xml_str = gen_with_config.generate_mjcf_string()
    assert isinstance(xml_str, str)
    assert len(xml_str) > 0


def test_generated_xml_is_valid(gen_with_config):
    xml_str = gen_with_config.generate_mjcf_string()
    # Should parse without error
    tree = etree.fromstring(xml_str.encode())
    assert tree is not None


def test_generated_xml_loadable_by_mujoco(gen_with_config, urdf_str):
    """Full pipeline: URDF -> merge config -> generate -> load in MuJoCo."""
    # Create a fresh generator to avoid double-finalize issues
    g = MjcfGenerator(name='quadruped_mj_test', urdf_str=urdf_str)
    g.merge_xml(CONFIG_XML_PATH.read_text())
    xml_str = g.generate_mjcf_string()
    model = mujoco.MjModel.from_xml_string(xml_str)
    assert model is not None
    assert model.njnt > 0


# ---------------------------------------------------------------------------
# MjModel: actuator gains
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def mj_model(urdf_str):
    """Fully generated and loaded MjModel, created once per module."""
    g = MjcfGenerator(name='quadruped_gains_test', urdf_str=urdf_str)
    g.merge_xml(CONFIG_XML_PATH.read_text())
    xml_str = g.generate_mjcf_string()
    return mujoco.MjModel.from_xml_string(xml_str)


def test_actuator_count(mj_model):
    # config.xml defines 8 position actuators (hip + knee for each of 4 legs)
    assert mj_model.nu == 8


def test_actuator_names_present(mj_model):
    expected = ['fl_hip', 'fl_knee', 'fr_hip', 'fr_knee',
                'rl_hip', 'rl_knee', 'rr_hip', 'rr_knee']
    actual = [mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
              for i in range(mj_model.nu)]
    for name in expected:
        assert name in actual, f'Actuator "{name}" not found in model'


def test_actuator_kp_gain(mj_model):
    """Position actuator gain (kp) should be 200 for robot_leg class."""
    # In MuJoCo position actuators: gainprm[0] = kp
    kp_expected = 200.0
    for i in range(mj_model.nu):
        kp = mj_model.actuator_gainprm[i, 0]
        assert pytest.approx(kp, rel=1e-3) == kp_expected, \
            f'Actuator {i} kp={kp}, expected {kp_expected}'


def test_actuator_kv_damping(mj_model):
    """Velocity damping (kv) should be 10 for robot_leg class.
    For MuJoCo position actuators, biasprm = [-kp, 0, -kv]."""
    kv_expected = 10.0
    for i in range(mj_model.nu):
        kv = -mj_model.actuator_biasprm[i, 2]
        assert pytest.approx(kv, rel=1e-3) == kv_expected, \
            f'Actuator {i} kv={kv}, expected {kv_expected}'


# ---------------------------------------------------------------------------
# MjModel: sensors
# ---------------------------------------------------------------------------

def test_sensor_count(mj_model):
    # config.xml defines 2 sensors: accelerometer + gyro
    assert mj_model.nsensor == 2


def test_sensor_names(mj_model):
    expected = ['base_link_acc', 'base_link_gyro']
    actual = [mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_SENSOR, i)
              for i in range(mj_model.nsensor)]
    assert actual == expected


def test_sensor_types(mj_model):
    # accelerometer = 7, gyro = 8 in mjtSensor
    assert mj_model.sensor_type[0] == mujoco.mjtSensor.mjSENS_ACCELEROMETER
    assert mj_model.sensor_type[1] == mujoco.mjtSensor.mjSENS_GYRO


