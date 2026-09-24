#!/usr/bin/env python3

# Copyright 2026 Berkan Tali
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Place robots and objects in RViz2 and save the workcell layout to YAML.

Four slots: robot1 (gripper arm on its rail), robot2 (welder arm), base_link
(workpiece), child_link (picked part). Right-click the palette, pick
"Place <slot> > <type>", then click-drag with the 2D Goal Pose tool to set
position and heading; drag a slot's own handles to fine-tune it. robot1's
gripper is swapped from its own menu or the palette ("Change gripper").

Saving ("Save layout", ~/save, or on exit) writes robot/object slots to
hold_and_weld_bringup/config/configurator_output/{workcell,objects}.yaml
(kept in git as a folder, contents git-ignored), never the live files
directly. Every launch reloads workcell_file/objects_file fresh - there is no
resume from a previous session's save. Robot meshes are previews only; copy
a staged layout in yourself to apply it.
"""

import ast
import copy
from dataclasses import dataclass
import glob
import math
import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from interactive_markers import InteractiveMarkerServer, MenuHandler
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker)
import xacro
import yaml

# Robot slot -> top-level xacro that renders that slot on its own.
ROBOT_SLOT_XACROS = {
    'robot1': 'robot1_gripper.xacro',
    'robot2': 'robot2_welder.xacro',
}
OBJECT_SLOTS = ('base_link', 'child_link')
OBJECT_URDF_DIR = 'urdf/environment'
OBJECT_URDF_SUFFIX = '.urdf.xacro'
ARM_MACRO_SUFFIX = '_arm_prefix.xacro'
GRIPPER_SLOT = 'robot1'   # the robot slot that carries workcell.yaml's gripper_model
GRIPPER_CATALOG = 'urdf/end_effectors/gripper_catalog.xacro'
OUTPUT_DIR = 'config/configurator_output'   # in hold_and_weld_bringup

DEFAULT_RGBA = (0.6, 0.6, 0.65, 1.0)


@dataclass
class Slot:
    """One placeable thing: a robot slot or an object slot."""

    name: str
    is_robot: bool
    type_name: str          # arm model for robots, object xacro stem for objects
    position: list          # [x, y, z] in the layout frame [m]
    orientation: list       # quaternion [x, y, z, w]


@dataclass
class PreviewVisual:
    """A URDF <visual> geometry placed in the robot's root frame at zero joint positions."""

    geometry: ET.Element
    transform: np.ndarray   # 4x4
    rgba: tuple


def rpy_to_matrix(roll, pitch, yaw):
    """Return the 3x3 rotation of URDF fixed-axis roll/pitch/yaw."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def matrix_to_quaternion(rotation):
    """Return the [x, y, z, w] quaternion of a 3x3 rotation matrix."""
    m = rotation
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s]
    return q


def yaw_to_quaternion(yaw):
    """Return the [x, y, z, w] quaternion of a rotation about +Z."""
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


def quaternion_to_yaw(q):
    """Return the heading (rotation about +Z) of an [x, y, z, w] quaternion."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def origin_transform(origin):
    """Return the 4x4 transform of a URDF <origin> element (identity when absent)."""
    transform = np.eye(4)
    if origin is None:
        return transform
    xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
    transform[:3, :3] = rpy_to_matrix(*rpy)
    transform[:3, 3] = xyz
    return transform


def zero_pose_visuals(urdf_xml):
    """
    Return every <visual> of a URDF, placed in its root link frame with all joints at zero.

    At zero, a revolute or prismatic joint's transform is just its <origin>, so the
    preview needs no joint-type handling.
    """
    robot = ET.fromstring(urdf_xml)
    named_colors = {}
    for material in robot.findall('material'):
        color = material.find('color')
        if color is not None:
            named_colors[material.get('name')] = tuple(float(v) for v in color.get('rgba').split())

    parent_of = {}
    for joint in robot.findall('joint'):
        parent_of[joint.find('child').get('link')] = (
            joint.find('parent').get('link'), origin_transform(joint.find('origin')))

    link_transforms = {}

    def link_transform(link):
        if link not in link_transforms:
            if link in parent_of:
                parent, joint_transform = parent_of[link]
                link_transforms[link] = link_transform(parent) @ joint_transform
            else:
                link_transforms[link] = np.eye(4)
        return link_transforms[link]

    visuals = []
    for link in robot.findall('link'):
        for visual in link.findall('visual'):
            geometry = visual.find('geometry')
            if geometry is None or len(geometry) == 0:
                continue
            rgba = None
            material = visual.find('material')
            if material is not None:
                color = material.find('color')
                if color is not None:
                    rgba = tuple(float(v) for v in color.get('rgba').split())
                else:
                    rgba = named_colors.get(material.get('name'))
            visuals.append(PreviewVisual(
                geometry[0],
                link_transform(link.get('name')) @ origin_transform(visual.find('origin')),
                rgba))
    return visuals


def visual_to_marker(visual):
    """Convert a PreviewVisual to a Marker, or None for unsupported geometry."""
    marker = Marker()
    shape = visual.geometry
    if shape.tag == 'mesh':
        filename = shape.get('filename')
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = filename
        scale = [float(v) for v in shape.get('scale', '1 1 1').split()]
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        # All-zero colour keeps a COLLADA mesh's own materials.
        marker.mesh_use_embedded_materials = filename.lower().endswith('.dae')
    elif shape.tag == 'box':
        marker.type = Marker.CUBE
        marker.scale.x, marker.scale.y, marker.scale.z = (
            float(v) for v in shape.get('size').split())
    elif shape.tag == 'cylinder':
        marker.type = Marker.CYLINDER
        diameter = 2.0 * float(shape.get('radius'))
        marker.scale.x, marker.scale.y, marker.scale.z = (
            diameter, diameter, float(shape.get('length')))
    elif shape.tag == 'sphere':
        marker.type = Marker.SPHERE
        diameter = 2.0 * float(shape.get('radius'))
        marker.scale.x = marker.scale.y = marker.scale.z = diameter
    else:
        return None

    if not (marker.mesh_use_embedded_materials and visual.rgba is None):
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            visual.rgba or DEFAULT_RGBA)
    marker.pose = transform_to_pose(visual.transform)
    return marker


def transform_to_pose(transform):
    """Convert a 4x4 transform to a Pose."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in transform[:3, 3])
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = matrix_to_quaternion(transform[:3, :3])
    return pose


def expand_xacro(path, mappings):
    """Run xacro on a file and return the URDF string."""
    return xacro.process_file(path, mappings=mappings).toxml()


def discover_robot_models(description_dir):
    """Return arm model names that have a <model>_arm_prefix.xacro."""
    pattern = os.path.join(description_dir, 'urdf', 'robots', '*' + ARM_MACRO_SUFFIX)
    return sorted(os.path.basename(p)[:-len(ARM_MACRO_SUFFIX)] for p in glob.glob(pattern))


def discover_gripper_models(description_dir):
    """Return the gripper model names listed in gripper_catalog.xacro's gripper_catalog."""
    path = os.path.join(description_dir, GRIPPER_CATALOG)
    for prop in ET.parse(path).getroot().iter('{http://ros.org/wiki/xacro}property'):
        if prop.get('name') == 'gripper_catalog':
            value = prop.get('value').strip()
            if not (value.startswith('${[') and value.endswith(']}')):
                raise RuntimeError(
                    f"{path}: gripper_catalog must be a list expression like \"${{['a', 'b']}}\", "
                    f'got {value!r}')
            return list(ast.literal_eval(value[2:-1]))   # strip the ${ }
    raise RuntimeError(f'{path} has no gripper_catalog property')


def discover_object_types(description_dir):
    """Return object type names that have a urdf/environment/<type>.urdf.xacro."""
    pattern = os.path.join(description_dir, OBJECT_URDF_DIR, '*' + OBJECT_URDF_SUFFIX)
    return sorted(os.path.basename(p)[:-len(OBJECT_URDF_SUFFIX)] for p in glob.glob(pattern))


def output_directory(bringup_share_dir):
    """
    Return the configurator_output folder saves go to.

    Under --symlink-install the installed .gitignore links back to src/, so this
    is the source folder; otherwise it is the installed copy.
    """
    marker = os.path.join(bringup_share_dir, OUTPUT_DIR, '.gitignore')
    return os.path.dirname(os.path.realpath(marker))


def staged_output_path(output_dir, live_path):
    """Return the default staging file a live config's edits are written to."""
    return os.path.join(output_dir, os.path.basename(live_path))


def object_type_to_urdf_path(type_name):
    """Return the objects.yaml urdf_path of an object type."""
    return f'{OBJECT_URDF_DIR}/{type_name}{OBJECT_URDF_SUFFIX}'


def urdf_path_to_object_type(urdf_path):
    """Return the object type of an objects.yaml urdf_path."""
    name = os.path.basename(urdf_path)
    return name[:-len(OBJECT_URDF_SUFFIX)] if name.endswith(OBJECT_URDF_SUFFIX) else name


def objects_parameters(objects_doc):
    """Return the ros__parameters block of an objects.yaml document."""
    return objects_doc.setdefault('/**', {}).setdefault('ros__parameters', {})


def robot_slots_from_workcell(workcell_doc):
    """Read the robot slots of a workcell.yaml document."""
    slots = []
    robots = workcell_doc.get('robots', {})
    for name in ROBOT_SLOT_XACROS:
        entry = robots.get(name)
        if entry is None:
            raise RuntimeError(f'workcell.yaml has no robots.{name} entry')
        slots.append(Slot(
            name, True, str(entry['model']),
            [float(entry.get(k, 0.0)) for k in ('x', 'y', 'z')],
            yaw_to_quaternion(float(entry.get('yaw', 0.0)))))
    return slots


def object_slots_from_objects(objects_doc):
    """
    Read the object slots of an objects.yaml document.

    Orientation comes from `orientation: {x, y, z, w}` (planning-scene readers),
    falling back to `pose: {qx, qy, qz, qw}` (the Gazebo spawner).
    """
    params = objects_parameters(objects_doc)
    slots = []
    for name in OBJECT_SLOTS:
        entry = params.get(name)
        if entry is None:
            continue
        pose = entry.get('pose', {})
        orientation = entry.get('orientation')
        if orientation is not None:
            quaternion = [float(orientation.get(k, d)) for k, d in
                          (('x', 0.0), ('y', 0.0), ('z', 0.0), ('w', 1.0))]
        else:
            quaternion = [float(pose.get(k, d)) for k, d in
                          (('qx', 0.0), ('qy', 0.0), ('qz', 0.0), ('qw', 1.0))]
        slots.append(Slot(
            name, False, urdf_path_to_object_type(entry.get('urdf_path', '')),
            [float(pose.get(k, 0.0)) for k in ('x', 'y', 'z')],
            quaternion))
    return slots


def gripper_model_from_workcell(workcell_doc):
    """Read the gripper model of a workcell.yaml document."""
    if 'gripper_model' not in workcell_doc:
        raise RuntimeError('workcell.yaml has no gripper_model entry')
    return str(workcell_doc['gripper_model'])


def robot_preview_layout(slot_name, model, gripper_model):
    """Return a workcell.yaml document placing one robot slot's model at the origin."""
    return {'robots': {slot_name: {'model': model, 'x': 0.0, 'y': 0.0, 'z': 0.0,
                                   'yaw': 0.0}},
            'gripper_model': gripper_model}


def updated_workcell(workcell_doc, slots, gripper_model):
    """Return a copy of a workcell.yaml document with the robot slots and gripper written in."""
    doc = copy.deepcopy(workcell_doc)
    doc['gripper_model'] = gripper_model
    robots = doc.setdefault('robots', {})
    for slot in slots:
        if slot.is_robot:
            robots[slot.name] = {
                'model': slot.type_name,
                'x': round(slot.position[0], 6),
                'y': round(slot.position[1], 6),
                'z': round(slot.position[2], 6),
                'yaw': round(quaternion_to_yaw(slot.orientation), 6),
            }
    return doc


def updated_objects(objects_doc, slots):
    """
    Return a copy of an objects.yaml document with the object slots written in.

    Keys the configurator does not edit (id, spawn_name, end_pose, ...) are kept.
    Orientation is written both as `orientation` and as `pose.q*`, the two
    schemas this file's readers use.
    """
    doc = copy.deepcopy(objects_doc)
    params = objects_parameters(doc)
    for slot in slots:
        if slot.is_robot:
            continue
        entry = params.setdefault(slot.name, {'id': slot.name, 'spawn_name': slot.name})
        entry['urdf_path'] = object_type_to_urdf_path(slot.type_name)
        qx, qy, qz, qw = (round(v, 6) for v in slot.orientation)
        pose = entry.setdefault('pose', {})
        pose.update({
            'x': round(slot.position[0], 6),
            'y': round(slot.position[1], 6),
            'z': round(slot.position[2], 6),
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
        })
        entry['orientation'] = {'x': qx, 'y': qy, 'z': qz, 'w': qw}
    params.setdefault('frame_id', 'world')
    return doc


def leading_comments(path):
    """Return the comment/blank lines at the top of a file, to carry over on rewrite."""
    lines = []
    with open(path, 'r') as file:
        for line in file:
            if line.strip() and not line.lstrip().startswith('#'):
                break
            lines.append(line)
    return ''.join(lines)


def write_yaml(path, doc):
    """
    Write a YAML document, keeping the file's leading comment block.

    Writes through symlinks, so under --symlink-install the source file is updated.
    """
    target = os.path.realpath(path)
    header = leading_comments(target) if os.path.exists(target) else ''
    os.makedirs(os.path.dirname(target), exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(dir=os.path.dirname(target), suffix='.tmp')
    try:
        with os.fdopen(fd, 'w') as file:
            file.write(header)
            yaml.safe_dump(doc, file, sort_keys=False, default_flow_style=False)
        os.replace(tmp_path, target)
    except BaseException:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise
    return target


def load_yaml(path):
    """Load a YAML file, raising RuntimeError naming the path on failure."""
    try:
        with open(path, 'r') as file:
            data = yaml.safe_load(file)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"Could not load '{path}': {exc}") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"'{path}' is empty or not a mapping")
    return data


def axis_control(name, mode, axis):
    """Return a move/rotate control along world-frame axis 'x', 'y' or 'z'."""
    control = InteractiveMarkerControl()
    control.name = name
    control.interaction_mode = mode
    half = math.sqrt(0.5)
    # A control acts along its own x axis; rotate x onto the requested axis.
    control.orientation.w = half
    if axis == 'x':
        control.orientation.x = half
    elif axis == 'y':
        control.orientation.z = half
    else:
        control.orientation.y = half
    return control


def set_check_states(menu, entries, selected):
    """Check the menu entry (handle -> name) whose name is `selected`, uncheck the rest."""
    for handle, name in entries.items():
        menu.setCheckState(
            handle, MenuHandler.CHECKED if name == selected else MenuHandler.UNCHECKED)


class WorkcellConfigurator(Node):
    """Interactive marker server for placing workcell robots and objects."""

    def __init__(self):
        """Load the current layout and build the palette and slot markers."""
        super().__init__('workcell_configurator')
        self.description_dir = get_package_share_directory('hold_and_weld_description')
        bringup_dir = get_package_share_directory('hold_and_weld_bringup')

        self.workcell_file = self.declare_parameter(
            'workcell_file',
            os.path.join(self.description_dir, 'config', 'workcell.yaml')).value
        self.objects_file = self.declare_parameter(
            'objects_file',
            os.path.join(bringup_dir, 'config', 'objects', 'objects.yaml')).value
        output_dir = output_directory(bringup_dir)
        self.workcell_output_file = self.declare_parameter(
            'workcell_output_file', '').value or staged_output_path(
            output_dir, self.workcell_file)
        self.objects_output_file = self.declare_parameter(
            'objects_output_file', '').value or staged_output_path(
            output_dir, self.objects_file)
        self.frame_id = self.declare_parameter('frame_id', 'world').value
        self.save_on_exit = self.declare_parameter('save_on_exit', True).value
        self.palette_position = self.declare_parameter(
            'palette_position', [-1.0, -1.0, 0.0]).value

        self.workcell_doc = load_yaml(self.workcell_file)
        self.objects_doc = load_yaml(self.objects_file)
        self.slots = {slot.name: slot for slot in
                      robot_slots_from_workcell(self.workcell_doc)
                      + object_slots_from_objects(self.objects_doc)}
        self.gripper_model = gripper_model_from_workcell(self.workcell_doc)

        self.previews = {}
        self.robot_models, self.gripper_models = self.build_robot_previews()
        self.object_types = self.build_object_previews()
        self.pending = None   # (slot name, type name) placed by the next click
        self.changed_slots = set()

        self.server = InteractiveMarkerServer(self, 'workcell_configurator')
        # slot name -> (MenuHandler, {entry handle: type name}, {entry handle: gripper model})
        self.slot_menus = {}
        self.palette_menu = self.build_palette_menu()
        for slot in self.slots.values():
            self.insert_slot(slot)
        self.insert_palette()
        self.server.applyChanges()

        self.create_subscription(PoseStamped, '~/place', self.on_place, 10)
        self.create_service(Trigger, '~/save', self.on_save)

        self.get_logger().info(
            f'Robots  {os.path.realpath(self.workcell_file)} -> {self.workcell_output_file}\n'
            f'Objects {os.path.realpath(self.objects_file)} -> {self.objects_output_file}\n'
            'Right-click the palette, choose "Place <slot>", then click-drag with the '
            '2D Goal Pose tool; "Change gripper" swaps robot1\'s gripper. Saving writes '
            'to the staging files above; copy their contents into the live config '
            'yourself to apply a layout.')

    def robot_preview_key(self, slot_name, model):
        """Return the previews key of a robot slot's model; robot1's includes its gripper."""
        return (slot_name, model, self.gripper_model if slot_name == GRIPPER_SLOT else None)

    def build_robot_previews(self):
        """
        Render every arm model in every robot slot, and robot1 with every gripper.

        Returns the arm models that render in every slot with the configured
        gripper, and the gripper models that render on every one of those arms.
        """
        arms = []
        for model in discover_robot_models(self.description_dir):
            rendered = {
                self.robot_preview_key(slot_name, model): self.render_robot(
                    slot_name, xacro_file, model, self.gripper_model)
                for slot_name, xacro_file in ROBOT_SLOT_XACROS.items()}
            if None not in rendered.values():
                self.previews.update(rendered)
                arms.append(model)
        if not arms:
            raise RuntimeError(
                f"No arm model could be rendered with gripper '{self.gripper_model}'; "
                'see errors above')

        grippers = []
        for gripper in discover_gripper_models(self.description_dir):
            if gripper == self.gripper_model:
                grippers.append(gripper)
                continue
            rendered = {
                (GRIPPER_SLOT, model, gripper): self.render_robot(
                    GRIPPER_SLOT, ROBOT_SLOT_XACROS[GRIPPER_SLOT], model, gripper)
                for model in arms}
            if None not in rendered.values():
                self.previews.update(rendered)
                grippers.append(gripper)
        return arms, grippers

    def render_robot(self, slot_name, xacro_file, model, gripper_model):
        """Expand a robot slot's xacro with the model at the origin; None on failure."""
        layout = robot_preview_layout(slot_name, model, gripper_model)
        fd, layout_path = tempfile.mkstemp(suffix='.yaml')
        try:
            with os.fdopen(fd, 'w') as file:
                yaml.safe_dump(layout, file)
            urdf = expand_xacro(
                os.path.join(self.description_dir, 'urdf', xacro_file),
                {'workcell_config': layout_path, 'controller_config_file': ''})
        except Exception as exc:  # xacro raises its own exception types
            self.get_logger().error(
                f"Arm model '{model}' / gripper '{gripper_model}' left out: "
                f'{xacro_file} failed with them: {exc}')
            return None
        finally:
            os.remove(layout_path)
        return self.markers_from_urdf(urdf)

    def build_object_previews(self):
        """Render every discovered object type; return the types that work."""
        working = []
        for type_name in discover_object_types(self.description_dir):
            path = os.path.join(
                self.description_dir, object_type_to_urdf_path(type_name))
            try:
                urdf = expand_xacro(path, {})
            except Exception as exc:  # xacro raises its own exception types
                self.get_logger().error(f"Object type '{type_name}' left out: {exc}")
                continue
            self.previews[(None, type_name)] = self.markers_from_urdf(urdf)
            working.append(type_name)
        return working

    def markers_from_urdf(self, urdf):
        """Convert a URDF's zero-pose visuals to markers."""
        markers = []
        for visual in zero_pose_visuals(urdf):
            marker = visual_to_marker(visual)
            if marker is not None:
                markers.append(marker)
        return markers

    def preview_for(self, slot):
        """Return the preview markers of a slot's current type (empty if unknown)."""
        key = (self.robot_preview_key(slot.name, slot.type_name) if slot.is_robot
               else (None, slot.type_name))
        if key not in self.previews:
            self.get_logger().warn(
                f"{slot.name}: no preview for type '{slot.type_name}'; showing handles only")
        return copy.deepcopy(self.previews.get(key, []))

    def insert_slot(self, slot):
        """(Re)insert a slot's interactive marker and its context menu."""
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = slot.name
        type_label = (f'{slot.type_name} + {self.gripper_model} gripper'
                      if slot.name == GRIPPER_SLOT else slot.type_name)
        marker.description = (
            f'{slot.name}: {type_label} (preview - applies on next launch)'
            if slot.is_robot else f'{slot.name}: {type_label}')
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = slot.position
        (marker.pose.orientation.x, marker.pose.orientation.y,
         marker.pose.orientation.z, marker.pose.orientation.w) = slot.orientation
        marker.scale = 1.0 if slot.is_robot else 0.5

        visual = InteractiveMarkerControl()
        visual.name = 'menu'
        visual.always_visible = True
        visual.interaction_mode = InteractiveMarkerControl.MENU
        visual.markers.extend(self.preview_for(slot))
        marker.controls.append(visual)

        if slot.is_robot:
            # Robots stand on the floor: translate freely, rotate about Z only.
            for axis in ('x', 'y', 'z'):
                control = axis_control(
                    f'move_{axis}', InteractiveMarkerControl.MOVE_AXIS, axis)
                control.orientation_mode = InteractiveMarkerControl.FIXED
                marker.controls.append(control)
            marker.controls.append(
                axis_control('rotate_z', InteractiveMarkerControl.ROTATE_AXIS, 'z'))
        else:
            for axis in ('x', 'y', 'z'):
                marker.controls.append(axis_control(
                    f'move_{axis}', InteractiveMarkerControl.MOVE_AXIS, axis))
                marker.controls.append(axis_control(
                    f'rotate_{axis}', InteractiveMarkerControl.ROTATE_AXIS, axis))

        self.server.insert(marker, feedback_callback=self.on_feedback)
        if slot.name not in self.slot_menus:
            self.slot_menus[slot.name] = self.build_slot_menu(slot)
        menu, type_entries, gripper_entries = self.slot_menus[slot.name]
        set_check_states(menu, type_entries, slot.type_name)
        set_check_states(menu, gripper_entries, self.gripper_model)
        menu.apply(self.server, slot.name)

    def build_slot_menu(self, slot):
        """Build a slot's right-click menu; return it with its type and gripper entry handles."""
        menu = MenuHandler()
        type_entries = {}
        parent = menu.insert('Change model' if slot.is_robot else 'Change object')
        for type_name in (self.robot_models if slot.is_robot else self.object_types):
            handle = menu.insert(
                type_name, parent=parent,
                callback=lambda feedback, t=type_name, s=slot.name: self.set_type(s, t))
            type_entries[handle] = type_name
        gripper_entries = (self.insert_gripper_menu(menu) if slot.name == GRIPPER_SLOT
                           else {})
        return menu, type_entries, gripper_entries

    def insert_gripper_menu(self, menu):
        """Add a "Change gripper" submenu to a menu; return its entry handles."""
        parent = menu.insert('Change gripper')
        return {menu.insert(gripper, parent=parent,
                            callback=lambda feedback, g=gripper: self.set_gripper(g)): gripper
                for gripper in self.gripper_models}

    def build_palette_menu(self):
        """Build the palette's menu: "Place <slot>" submenus, "Change gripper", and Save."""
        menu = MenuHandler()
        for slot_name in list(ROBOT_SLOT_XACROS) + list(OBJECT_SLOTS):
            is_robot = slot_name in ROBOT_SLOT_XACROS
            parent = menu.insert(f'Place {slot_name}')
            for type_name in (self.robot_models if is_robot else self.object_types):
                menu.insert(
                    type_name, parent=parent,
                    callback=lambda feedback, s=slot_name, t=type_name: self.arm_click(s, t))
        self.palette_gripper_entries = self.insert_gripper_menu(menu)
        menu.insert('Save layout', callback=lambda feedback: self.save())
        return menu

    def insert_palette(self):
        """(Re)insert the palette marker showing what the next click will place."""
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = 'palette'
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            float(v) for v in self.palette_position)
        marker.scale = 0.4

        body = Marker()
        body.type = Marker.CUBE
        body.scale.x = body.scale.y = 0.25
        body.scale.z = 0.05
        body.color.r, body.color.g, body.color.b, body.color.a = 1.0, 0.6, 0.1, 1.0

        label = Marker()
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.position.z = 0.3
        label.scale.z = 0.12
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = 'Workcell palette (right-click)'
        if self.pending is not None:
            label.text += f'\nnext 2D Goal Pose click places {self.pending[0]}: {self.pending[1]}'

        control = InteractiveMarkerControl()
        control.name = 'menu'
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.markers.extend([body, label])
        marker.controls.append(control)

        self.server.insert(marker)
        set_check_states(self.palette_menu, self.palette_gripper_entries, self.gripper_model)
        self.palette_menu.apply(self.server, 'palette')

    def arm_click(self, slot_name, type_name):
        """Make the next 2D Goal Pose click place `slot_name` as `type_name`."""
        self.pending = (slot_name, type_name)
        self.insert_palette()
        self.server.applyChanges()
        self.get_logger().info(
            f'Click-drag with 2D Goal Pose to place {slot_name} ({type_name})')

    def on_place(self, msg):
        """Move the pending slot to a 2D Goal Pose click, facing the drag direction."""
        if self.pending is None:
            self.get_logger().info('Choose "Place <slot>" on the palette before clicking')
            return
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.get_logger().warn(
                f"Ignoring click in frame '{msg.header.frame_id}'; set RViz2's fixed "
                f"frame to '{self.frame_id}'")
            return
        slot_name, type_name = self.pending
        slot = self.slots.get(slot_name)
        if slot is None:
            slot = Slot(slot_name, False, type_name, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
            self.slots[slot_name] = slot
        # Height is kept: the click lands on the floor, but a workpiece may sit on a table.
        slot.type_name = type_name
        slot.position[0] = msg.pose.position.x
        slot.position[1] = msg.pose.position.y
        o = msg.pose.orientation
        slot.orientation = yaw_to_quaternion(quaternion_to_yaw([o.x, o.y, o.z, o.w]))
        self.pending = None
        self.changed_slots.add(slot_name)
        self.insert_slot(slot)
        self.insert_palette()
        self.server.applyChanges()
        self.get_logger().info(
            f'{slot_name} ({type_name}) placed at '
            f'({slot.position[0]:.3f}, {slot.position[1]:.3f}, {slot.position[2]:.3f})')

    def set_type(self, slot_name, type_name):
        """Switch a slot to another model/object type in place."""
        slot = self.slots[slot_name]
        slot.type_name = type_name
        self.changed_slots.add(slot_name)
        self.insert_slot(slot)
        self.server.applyChanges()

    def set_gripper(self, gripper_model):
        """Switch robot1's gripper; saved with robot1's slot."""
        self.gripper_model = gripper_model
        self.changed_slots.add(GRIPPER_SLOT)
        self.insert_slot(self.slots[GRIPPER_SLOT])
        self.insert_palette()
        self.server.applyChanges()
        self.get_logger().info(f'{GRIPPER_SLOT} gripper: {gripper_model}')

    def on_feedback(self, feedback):
        """Track slot poses as their handles are dragged."""
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        slot = self.slots.get(feedback.marker_name)
        if slot is None:
            return
        p, o = feedback.pose.position, feedback.pose.orientation
        slot.position = [p.x, p.y, p.z]
        slot.orientation = [o.x, o.y, o.z, o.w]
        if slot.is_robot:
            slot.orientation = yaw_to_quaternion(quaternion_to_yaw(slot.orientation))
        self.changed_slots.add(slot.name)

    def on_save(self, request, response):
        """Handle the ~/save service."""
        try:
            response.message = self.save()
            response.success = True
        except (OSError, yaml.YAMLError) as exc:
            response.message = str(exc)
            response.success = False
        return response

    def save(self):
        """
        Write changed slots to the staging files, never the live config.

        Robot slots go to workcell_output_file, object slots to objects_output_file.
        Copy their contents into the live workcell.yaml/objects.yaml yourself to apply
        a layout; this keeps a manual review step between placing things in RViz2 and
        the files the rest of the framework actually reads.
        """
        changed = [self.slots[name] for name in sorted(self.changed_slots)]
        written = []
        if any(slot.is_robot for slot in changed):
            self.workcell_doc = updated_workcell(self.workcell_doc, changed, self.gripper_model)
            written.append(write_yaml(self.workcell_output_file, self.workcell_doc))
        if any(not slot.is_robot for slot in changed):
            self.objects_doc = updated_objects(self.objects_doc, changed)
            written.append(write_yaml(self.objects_output_file, self.objects_doc))
        self.changed_slots.clear()
        message = (
            f'Saved layout to {" and ".join(written)} (copy into the live config to apply it)'
            if written else 'Nothing changed')
        self.get_logger().info(message)
        return message


def main(args=None):
    """Run the workcell configurator; save unsaved changes on exit if enabled."""
    rclpy.init(args=args)
    node = WorkcellConfigurator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node.save_on_exit and node.changed_slots:
                node.save()
        finally:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
