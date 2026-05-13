import subprocess
import os 
import shutil
import rospkg
from lxml import etree
from copy import deepcopy
from pathlib import Path

import mujoco

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

        # produce mujoco's xml
        cmd = f'mujoco_compile {self.mj_urdf_path} {self.mj_xml_path_orig}'
        subprocess.run(cmd.split())

        # load the produced xml, add compiler directives, and write final xml
        with open(self.mj_xml_path_orig, 'r') as file:
            mj_xml = file.read()
            self.mj_xml_tree = etree.fromstring(mj_xml)
            mujoco_tag = self.mj_xml_tree.xpath('/mujoco')[0]
            etree.strip_tags(self.mj_xml_tree, etree.Comment)

        # add compiler directives
        compiler = etree.Element("compiler")
        compiler.attrib['meshdir'] = self.mj_xml_dir
        mujoco_tag.append(compiler)


    def merge_xml(self, xml_str: str | Path):
        if isinstance(xml_str, Path):
            xml_str = xml_str.read_text()
        mj_opt_tree = etree.fromstring(xml_str)
        etree.strip_tags(mj_opt_tree, etree.Comment)
        self.mj_xml_tree = MjcfGenerator._tree_merge(self.mj_xml_tree, mj_opt_tree)

    
    def load_config(self, config: dict):

        childclass = config.get('childclass')
        sites = config.get('sites')
        self.q_init = config.get('q_init', {})
        
        # patch xml to add site tags
        if sites is not None:
            for site in sites:
                body = site['body']
                site_name = site['site']
                # find body in mj_xml_tree and add the site
                body_elem = self.mj_xml_tree.xpath(f'//body[@name="{body}"]')
                if body_elem:
                    site_elem = etree.Element('site')
                    site_elem.attrib['name'] = site_name
                    body_elem[0].append(site_elem)
                else:
                    print(f'Warning: body {body} not found in mj_xml_tree')

        # patch xml to add childclass attributes
        if childclass is not None:
            for body, cc in childclass.items():
                # find body in mj_xml_tree and add the childclass attribute
                body_elem = self.mj_xml_tree.xpath(f'//body[@name="{body}"]')
                if body_elem:
                    body_elem[0].attrib['childclass'] = cc
                else:
                    print(f'Warning: body {body} not found in mj_xml_tree')


    def generate_mjcf_string(self):
        mjcf_str = etree.tostring(self.mj_xml_tree).decode()
        with open(self.mj_xml_path, 'w') as f:
            print(f'writing final mjcf to {self.mj_xml_path}')
            f.write(mjcf_str)
        return mjcf_str

    def create_mj_model_data(self):
        mjcf_str = self.generate_mjcf_string()
        model = mujoco.MjModel.from_xml_string(mjcf_str)
        data = mujoco.MjData(model)

        # set initial position
        for joint, q in self.q_init.items():
            data.joint(joint).qpos = q
            data.actuator(joint).ctrl = q

        return model, data
    
    def set_initial_position(self, data: mujoco.MjData):
        for joint, q in self.q_init.items():
            data.joint(joint).qpos = q
            data.actuator(joint).ctrl = q

    def _preprocess_urdf(XML):
        tree = etree.fromstring(XML)
        etree.strip_tags(tree, etree.Comment)
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

            if uri.startswith('package://'):
                pkg_start = len('package://')
                pkg_end = uri.find('/', pkg_start)
                pkg_name = uri[pkg_start:pkg_end]
                uri = rospack.get_path(pkg_name) + uri[pkg_end:] 
            
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

        def inner(aparent, bparent):
            for bchild in bparent:
                achild = aparent.xpath('./' + bchild.tag)
                if achild and bchild.getchildren():
                    inner(achild[0], bchild)
                else:
                    aparent.append(bchild)

        res = deepcopy(a)
        inner(res, b)
        return res