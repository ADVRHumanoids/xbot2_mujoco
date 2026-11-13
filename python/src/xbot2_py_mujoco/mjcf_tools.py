import os 
import shutil
import rospkg
from lxml import etree
from copy import deepcopy
from pathlib import Path

import mujoco
from xbot2_py_mujoco.rgbd import RgbdCameraCfg, validate_cameras
from xbot2_py_mujoco.lidar import LidarCfg, validate_lidars
from xbot2_py_mujoco.imu import ImuCfg, validate_imus

# Mesh formats MuJoCo loads
MESH_FORMATS = ('.stl', '.obj', '.msh')

class MjcfGenerator:
    
    def __init__(self, name: str, urdf_str: str, output_dir: str = None, copy_assets: bool = False):

        # inputs
        self.name = name
        self.urdf_orig = urdf_str
        self.output_dir = output_dir
        self.copy_assets = copy_assets

        # useful paths
        self.mj_xml_dir = f'/tmp/{self.name}_mujoco' if self.output_dir is None else self.output_dir
        self.mj_urdf_path = os.path.join(self.mj_xml_dir, f'{self.name}.urdf')
        self.mj_xml_path = os.path.join(self.mj_xml_dir, f'{self.name}.xml')
        self.mj_xml_path_orig = os.path.join(self.mj_xml_dir, f'{self.name}.orig.xml')
        self.mj_assetsdir = os.path.join(self.mj_xml_dir, 'assets')

        # 
        self.q_init : dict[str, float] = {}
        self.rgbd_cameras: list[RgbdCameraCfg] = []
        self.lidars: list[LidarCfg] = []
        self.imus: list[ImuCfg] = []

        # create directories
        shutil.rmtree(self.mj_xml_dir, ignore_errors=True)
        os.makedirs(self.mj_xml_dir, exist_ok=True)
        os.makedirs(self.mj_assetsdir, exist_ok=True)

        # urdf processing + copy assets
        self.urdf_processed = MjcfGenerator._preprocess_urdf(urdf_str)
        self.urdf_processed = self._resolve_ros_package_uri_and_copy_assets(self.urdf_processed)
        
        # write pre-processed urdf
        with open(self.mj_urdf_path, 'w') as f:
            f.write(self.urdf_processed)

        # produce mujoco's xml via the Python API (loads URDF, saves as MJCF)
        _model = mujoco.MjModel.from_xml_path(str(self.mj_urdf_path))
        mujoco.mj_saveLastXML(str(self.mj_xml_path_orig), _model)

        # load the produced xml, add compiler directives, and write final xml
        with open(self.mj_xml_path_orig, 'r') as file:
            mj_xml = file.read()
            self.mj_xml_tree = etree.fromstring(mj_xml)
            mujoco_tag = self.mj_xml_tree.xpath('/mujoco')[0]
            etree.strip_tags(self.mj_xml_tree, etree.Comment)

        # add compiler directives
        compiler = etree.Element("compiler")
        compiler.attrib['meshdir'] = self.mj_xml_dir
        compiler.attrib['texturedir'] = self.mj_xml_dir
        mujoco_tag.append(compiler)


    def merge_xml(self, xml_str: str | Path):
        source_dir = None
        if isinstance(xml_str, Path):
            source_dir = xml_str.parent.resolve()
            xml_str = xml_str.read_text()
        mj_opt_tree = etree.fromstring(xml_str.encode())
        etree.strip_tags(mj_opt_tree, etree.Comment)
        if source_dir is not None:
            for asset in mj_opt_tree.xpath('//asset//*[@file]'):
                source = Path(asset.get('file'))
                if not source.is_absolute():
                    source = source_dir / source
                if not source.is_file():
                    continue
                destination = Path(self.mj_assetsdir) / source.name
                shutil.copy2(source, destination)
                asset.set('file', f'./assets/{destination.name}')
        self.mj_xml_tree = MjcfGenerator._tree_merge(self.mj_xml_tree, mj_opt_tree)


    def generate_mjcf_string(self):
        self._finalize_xml()
        mjcf_str = etree.tostring(self.mj_xml_tree).decode()
        if self.rgbd_cameras or self.lidars or self.imus:
            model = mujoco.MjModel.from_xml_string(mjcf_str)
            validate_cameras(model, self.rgbd_cameras)
            validate_lidars(model, self.lidars)
            validate_imus(model, self.imus)
        with open(self.mj_xml_path, 'w') as f:
            print(f'[MjcfGenerator] writing final mjcf to {self.mj_xml_path}')
            f.write(mjcf_str)
        return mjcf_str


    def create_mj_model_data(self):
        mjcf_str = self.generate_mjcf_string()
        model = mujoco.MjModel.from_xml_string(mjcf_str)
        data = mujoco.MjData(model)

        # set initial position
        for joint, q in self.q_init.items():
            data.joint(joint).qpos = q
            actuator = model.actuator(joint)
            if actuator.dyntype[0] == mujoco.mjtDyn.mjDYN_INTEGRATOR:
                data.act[int(actuator.actadr[0])] = q
                data.actuator(joint).ctrl = 0.0
            else:
                data.actuator(joint).ctrl = q

        return model, data


    def _uri_to_path(uri, rospack):
        if uri.startswith('package://'):
            pkg_start = len('package://')
            pkg_end = uri.find('/', pkg_start)
            return rospack.get_path(uri[pkg_start:pkg_end]) + uri[pkg_end:]
        if uri.startswith('file://'):
            return uri[len('file://'):]
        return uri


    def _replace_unsupported_meshes(tree):
        """Use a same-name sibling MuJoCo loads (e.g. a .dae's .stl), else drop the visual or collision."""
        rospack = rospkg.RosPack()
        for mesh in tree.xpath('//mesh[@filename]'):
            uri = mesh.get('filename')
            stem, ext = os.path.splitext(uri)
            if ext.lower() in MESH_FORMATS:
                continue
            sibling = next((stem + e for e in MESH_FORMATS
                            if os.path.isfile(MjcfGenerator._uri_to_path(stem + e, rospack))), None)
            if sibling is not None:
                mesh.set('filename', sibling)
                continue
            element = mesh.getparent().getparent()  # <visual> or <collision>
            print(f'[MjcfGenerator] warning: MuJoCo cannot load mesh {uri}, dropping its <{element.tag}>')
            element.getparent().remove(element)


    def _drop_degenerate_primitives(tree):
        """Drop the visual or collision of a zero-size primitive (e.g. a <sphere radius="0"/> placeholder)."""
        for shape in tree.xpath('//geometry/box | //geometry/cylinder | //geometry/sphere'):
            values = [float(v) for a in ('size', 'radius', 'length') for v in shape.get(a, '1').split()]
            if min(values) > 0:
                continue
            element = shape.getparent().getparent()  # <visual> or <collision>
            link = element.getparent()
            print(f'[MjcfGenerator] warning: zero-size <{shape.tag}> in link {link.get("name")}, '
                  f'dropping its <{element.tag}>')
            link.remove(element)


    def _preprocess_urdf(XML):
        tree = etree.fromstring(XML.encode())
        etree.strip_tags(tree, etree.Comment)
        MjcfGenerator._replace_unsupported_meshes(tree)
        MjcfGenerator._drop_degenerate_primitives(tree)
        robot = tree.xpath('/robot')[0]
        mujoco = etree.Element("mujoco")
        compiler = etree.Element("compiler")
        compiler.attrib['fusestatic'] = 'false'
        compiler.attrib['discardvisual'] = 'false'
        compiler.attrib['angle'] = 'radian'
        compiler.attrib['strippath'] = 'false'
        mujoco.append(compiler)
        robot.append(mujoco)
        return etree.tostring(tree).decode()
    
    
    def _resolve_ros_package_uri_and_copy_assets(self, urdf: str):
        rospack = rospkg.RosPack()
        seq_id = 0
        urdf_processed = str()
        last_pos = 0
        pos = urdf.find('filename="')    
        while pos != -1:
            uri_start = pos + len('filename="')
            uri_end = urdf.find('"', uri_start)
            uri = urdf[uri_start:uri_end]

            uri = MjcfGenerator._uri_to_path(uri, rospack)

            filename = os.path.basename(uri)
            dst_file = os.path.join(self.mj_assetsdir, str(seq_id) + '_' + filename)
            seq_id += 1
            if not os.path.exists(dst_file):
                try: 
                    os.remove(dst_file)
                except: pass

                if self.copy_assets:
                    shutil.copy(uri, dst_file)
                else:
                    os.symlink(uri, dst_file)

            urdf_processed = urdf_processed + urdf[last_pos:uri_start] + './assets/' + os.path.basename(dst_file)
            last_pos = uri_end
            pos = urdf.find('filename="', uri_end)    

        urdf_processed = urdf_processed + urdf[last_pos:]
        return urdf_processed
    
    
    def _tree_merge(a, b):

        def merge_key(elem):
            if elem.tag == 'default':
                return 'class', elem.get('class')
            return 'name', elem.get('name')

        def inner(aparent, bparent):
            for bchild in bparent:
                attr_name, attr_value = merge_key(bchild)
                if attr_value is not None:
                    achild = aparent.xpath(f'./{bchild.tag}[@{attr_name}="{attr_value}"]')
                else:
                    achild = aparent.xpath('./' + bchild.tag)
                if achild and list(bchild):
                    inner(achild[0], bchild)
                else:
                    aparent.append(deepcopy(bchild))

        res = deepcopy(a)
        inner(res, b)
        return res
    
    
    def _finalize_xml(self):
        # process <reference> tags by inserting their body content in place 
        # and removing the <reference> tag itself
        mujoco_root = self.mj_xml_tree.xpath('/mujoco')[0]
        for ref in mujoco_root.findall('reference'):
            elem_tag = ref.get('element')
            elem_name = ref.get('name')
            if elem_tag is None or elem_name is None:
                print(f'Warning: <reference> tag missing "element" or "name" attribute, skipping')
                mujoco_root.remove(ref)
                continue
            targets = self.mj_xml_tree.xpath(f'//{elem_tag}[@name="{elem_name}"]')
            if not targets:
                print(f'Warning: <reference> target <{elem_tag} name="{elem_name}"> not found, skipping')
                mujoco_root.remove(ref)
                continue
            target = targets[0]
            for child in ref:
                target.append(deepcopy(child))
            mujoco_root.remove(ref)
            
        # process <setattribute> tags by setting the specified attribute in the target element
        # and removing the <setattribute> tag itself
        for setattr_elem in mujoco_root.findall('setattribute'):
            elem_tag = setattr_elem.get('element')
            elem_name = setattr_elem.get('name')
            if elem_tag is None or elem_name is None:
                print(f'Warning: <setattribute> tag missing "element" or "name" attribute, skipping')
                mujoco_root.remove(setattr_elem)
                continue
            targets = self.mj_xml_tree.xpath(f'//{elem_tag}[@name="{elem_name}"]')
            if not targets:
                print(f'Warning: <setattribute> target <{elem_tag} name="{elem_name}"> not found, skipping')
                mujoco_root.remove(setattr_elem)
                continue
            target = targets[0]
            for attr in setattr_elem.findall('attribute'):
                attr_name = attr.get('name')
                attr_value = attr.get('value')
                if attr_name is not None and attr_value is not None:
                    target.attrib[attr_name] = attr_value
            mujoco_root.remove(setattr_elem)
            
        # parse <q_init> tags to fill self.q_init dict and remove the tags from the xml
        for q_init_elem in mujoco_root.findall('q_init'):
            for joint in q_init_elem.findall('joint'):
                joint_name = joint.get('name')
                joint_value = joint.get('value')
                if joint_name is None or joint_value is None:
                    print(f'Warning: <joint> inside <q_init> missing "name" or "value" attribute, skipping')
                    continue
                try:
                    self.q_init[joint_name] = float(joint_value)
                except ValueError:
                    print(f'Warning: <q_init> joint "{joint_name}" has non-numeric value "{joint_value}", skipping')
            mujoco_root.remove(q_init_elem)

        # Keep capture settings outside MuJoCo's schema, like q_init.
        elements = mujoco_root.findall('rgbd')
        cameras = [RgbdCameraCfg.from_xml(element) for element in elements]
        self.rgbd_cameras.extend(cameras)
        for element in elements:
            mujoco_root.remove(element)

        for tag, cls, configs in (('lidar', LidarCfg, self.lidars), ('imu', ImuCfg, self.imus)):
            elements = mujoco_root.findall(tag)
            configs.extend(cls.from_xml(element) for element in elements)
            for element in elements:
                mujoco_root.remove(element)
        
        
        
