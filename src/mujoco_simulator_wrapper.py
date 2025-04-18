#!/usr/bin/env python3

from ast import arg
import subprocess
import os 
import shutil
import argparse
import rospkg
from lxml import etree
from copy import deepcopy
import sys
import yaml

print(sys.argv)

descr = """
A wrapper aroung mujoco_simulator that assembles a set of separate files
that describe the simulation into a sigle XML file that mujoco can understand.
These files are:
 - the robot urdf
 - a simulation option file (e.g. step size, disabled collisions, ...)
 - a world description (e.g. a ground plane, lights, ...)
 - a configuration file for the decentralized controllers (e.g., default gains)
"""
parser = argparse.ArgumentParser(description=descr)
parser.add_argument('--urdf', required=False, help='The path to the robot urdf file')
parser.add_argument('--urdf-command', required=False, help='The shell command to obtain the robot urdf (e.g. xacro)')
parser.add_argument('--simopt', help='The path to an XML file containing simulator options')
parser.add_argument('--world', help='The path to an XML file containing the world description')
parser.add_argument('--ctrlcfg', help='The path to a YAML file containing decentralized control configuration')
parser.add_argument('--sdf', help='The path to a YAML file containing links with sdf collision models')
parser.add_argument('--sites', help='The path to an XML file containing additional sites for the model')
parser.add_argument('--actuators', help='The path to an XML file containing actuators for the model')
parser.add_argument('--name', help='Unique robot name')
parser.add_argument('--output-dir', '-o', help='Output directory')
parser.add_argument('--skip-sim', action='store_true', help='Do not run simulate (only generate self contained directory with mesh symlinks and xml file)')
parser.add_argument('--copy-assets', action='store_true', help='Copy assets instead of symlinking them')
args, _ = parser.parse_known_args()

# handle to rospack system
rospack = rospkg.RosPack()

# util: remove comments, add mujoco compiler options
def preprocess_urdf(XML):
    tree = etree.fromstring(XML)
    etree.strip_tags(tree, etree.Comment)
    robot = tree.xpath('/robot')[0]
    mujoco = etree.Element("mujoco")
    compiler = etree.Element("compiler")
    compiler.attrib['fusestatic'] = 'false'
    compiler.attrib['discardvisual'] = 'false'
    compiler.attrib['strippath'] = 'false'
    mujoco.append(compiler)
    robot.append(mujoco)
    return etree.tostring(tree)


def treeMerge(a, b):

    def inner(aparent, bparent):
        print(f'processing {aparent.tag} vs {bparent.tag}')
        for bchild in bparent:
            print(f'..processing {bchild.tag}')
            achild = aparent.xpath('./' + bchild.tag)
            if achild and bchild.getchildren():
                if len(achild[0].attrib) == 0:
                    inner(achild[0], bchild)
                else:
                    aparent.append(bchild)
            else:
                aparent.append(bchild)


    res = deepcopy(a)
    inner(res, b)
    return res


# useful paths
mj_xml_dir = f'/tmp/{args.name}_mujoco' if args.output_dir is None else args.output_dir
mj_urdf_path = os.path.join(mj_xml_dir, f'{args.name}.urdf')
mj_xml_path = os.path.join(mj_xml_dir, f'{args.name}.xml')
mj_xml_path_orig = os.path.join(mj_xml_dir, f'{args.name}.orig.xml')
mj_assetsdir = os.path.join(mj_xml_dir, 'assets')

# create directory
shutil.rmtree(mj_xml_dir, ignore_errors=True)
os.makedirs(mj_xml_dir, exist_ok=True)
os.makedirs(mj_assetsdir, exist_ok=True)

# get urdf
if args.urdf:
    urdf_path = args.urdf
elif args.urdf_command:
    print(f'calling {args.urdf_command}')
    urdf_content = subprocess.check_output(args.urdf_command, shell=True).decode()
    urdf_path = os.path.join(mj_xml_dir, f'{args.name}.orig.urdf')
    open(urdf_path, 'w').write(urdf_content)
else:
    raise RuntimeError('either --urdf or --urdf-command must be specified')

# pre-process urdf to 
#  1) turn package:// directives into absolute paths
#  2) create a different symlink for each mesh to circumvent mjc's bug
with open(urdf_path, 'r') as file:
    urdf = file.read()

seq_id = 0
urdf = preprocess_urdf(urdf.encode()).decode()
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
    dst_file = os.path.join(mj_assetsdir, str(seq_id) + '_' + filename)
    seq_id += 1
    if not os.path.exists(dst_file):
        try: 
            os.remove(dst_file)
        except: pass

        if args.copy_assets:
            shutil.copy(uri, dst_file)
        else:
            os.symlink(uri, dst_file)

    urdf_processed = urdf_processed + urdf[last_pos:uri_start] + './assets/' + os.path.basename(dst_file)
    last_pos = uri_end
    pos = urdf.find('filename="', uri_end)    

urdf_processed = urdf_processed + urdf[last_pos:]

# write pre-processed urdf
open(mj_urdf_path, 'w').write(urdf_processed)

# produce mujoco's xml
cmd = f'mujoco_compile {mj_urdf_path} {mj_xml_path_orig}'
print(f'calling {cmd}')
subprocess.run(cmd.split())

with open(mj_xml_path_orig, 'r') as file:
    mj_xml = file.read()
    mj_xml_tree = etree.fromstring(mj_xml)
    mujoco = mj_xml_tree.xpath('/mujoco')[0]
    etree.strip_tags(mj_xml_tree, etree.Comment)

try: 
    with open(args.sdf, 'r') as file:
        sdf_yaml = yaml.safe_load(file)
        worldbody = mj_xml_tree.findall('worldbody')[0]

        for elem in worldbody.iter():
            if elem.attrib.get('name') in sdf_yaml.keys():
                for child in list(elem):
                    if child.tag == 'geom' and 'contype' not in child.attrib.keys():  # remove collision geometry (the one withoud contype attrib)
                        elem.remove(child)

                sdf_geom = etree.Element('geom')  # add sdf geompetry in place of mesh collision
                sdf_geom.set('name', f"{elem.attrib.get('name')}_sdf")
                sdf_geom.set('type', 'sdf')
                sdf_geom.set('mesh', sdf_yaml[elem.attrib.get('name')])
                sdf_geom.set('type', 'sdf')
                sdf = etree.SubElement(sdf_geom, 'plugin')
                sdf.set('instance', sdf_yaml[elem.attrib.get('name')])
                elem.append(sdf_geom)
except:
    print(f'{args.sdf} not found, no collision geom susbstitution done.')

# add default joint configuration to pelvis
# for elem in mj_xml_tree.iter():
#     if elem.attrib.get('name') == 'pelvis':
#         elem.set('childclass', 'kyon_all')

# add compiler attr


# add options and world
with open(args.simopt, 'r') as file:
    mj_opt = file.read()
    mj_opt_tree = etree.fromstring(mj_opt)
    etree.strip_tags(mj_opt_tree, etree.Comment)

with open(args.world, 'r') as file:
    mj_world = file.read()
    mj_world_tree = etree.fromstring(mj_world)
    etree.strip_tags(mj_world_tree, etree.Comment)

with open(args.sites, 'r') as file:
    mj_sites = file.read()
    mj_sites_tree = etree.fromstring(mj_sites)
    etree.strip_tags(mj_sites_tree, etree.Comment)

mj_act = None 
if args.actuators:
    with open(args.actuators, 'r') as file:
        mj_act = file.read()
        mj_act_tree = etree.fromstring(mj_act)
        etree.strip_tags(mj_sites_tree, etree.Comment)

# why do I do this?
# try:
#     mj_xml_tree.remove(mj_xml_tree.xpath('./compiler')[0])
# except IndexError:
#     pass

compiler = etree.Element("compiler")
compiler.attrib['meshdir'] = '.'
mujoco.append(compiler)

try:
    mj_xml_tree.remove(mj_xml_tree.xpath('./size')[0])
except IndexError:
    pass


xml_merged = treeMerge(mj_xml_tree, mj_opt_tree)
xml_merged = treeMerge(xml_merged, mj_world_tree)
if mj_act is not None:
    xml_merged = treeMerge(xml_merged, mj_act_tree)

# add sites
site_bodies = mj_sites_tree.xpath('./body')
for sb in site_bodies:
    sname = sb.get('name')
    body = xml_merged.findall(f".//body[@name='{sname}']")[0]
    site = etree.Element('site')
    site.attrib['name'] = sb.xpath('./site')[0].get('name')
    body.append(site)

open(mj_xml_path, 'w').write(etree.tostring(xml_merged, pretty_print=True).decode())

if args.skip_sim:
    exit(0)

print(f'running mujoco_simulator {mj_xml_path} {args.ctrlcfg}')
subprocess.run(['mujoco_simulator', mj_xml_path, args.ctrlcfg])

print('bye')
