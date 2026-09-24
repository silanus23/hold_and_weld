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

"""Tests for the workcell configurator's layout, preview and YAML logic."""

import math
import os
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import workcell_configurator as wc  # noqa: E402, I100

DESCRIPTION_DIR = get_package_share_directory('hold_and_weld_description')


def test_yaw_quaternion_roundtrip():
    for yaw in (0.0, 0.7, -2.5, math.pi - 1e-6):
        assert wc.quaternion_to_yaw(wc.yaw_to_quaternion(yaw)) == pytest.approx(yaw)


def test_matrix_to_quaternion_matches_rpy():
    rotation = wc.rpy_to_matrix(0.3, -0.4, 1.2)
    x, y, z, w = wc.matrix_to_quaternion(rotation)
    # Rebuild the matrix from the quaternion and compare.
    rebuilt = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    assert rebuilt == pytest.approx(rotation, abs=1e-9)


def test_zero_pose_visuals_chains_joint_and_visual_origins():
    urdf = """
    <robot name="r">
      <material name="red"><color rgba="1 0 0 1"/></material>
      <link name="world"/>
      <link name="arm">
        <visual>
          <origin xyz="1 0 0"/>
          <geometry><box size="0.1 0.2 0.3"/></geometry>
          <material name="red"/>
        </visual>
      </link>
      <joint name="j" type="revolute">
        <parent link="world"/><child link="arm"/>
        <origin xyz="0 0 1" rpy="0 0 1.5707963267948966"/>
      </joint>
    </robot>"""
    visuals = wc.zero_pose_visuals(urdf)
    assert len(visuals) == 1
    # Joint yaws 90 deg, so the visual's +x offset lands on world +y.
    assert visuals[0].transform[:3, 3] == pytest.approx([0.0, 1.0, 1.0])
    assert visuals[0].rgba == (1.0, 0.0, 0.0, 1.0)
    marker = wc.visual_to_marker(visuals[0])
    assert (marker.scale.x, marker.scale.y, marker.scale.z) == pytest.approx((0.1, 0.2, 0.3))


def test_robot1_preview_puts_arm_on_rail_carriage(tmp_path):
    layout = tmp_path / 'layout.yaml'
    layout.write_text(yaml.safe_dump(
        {'robots': {'robot1': {'model': 'gp25', 'x': 0, 'y': 0, 'z': 0, 'yaw': 0}}}))
    urdf = wc.expand_xacro(os.path.join(DESCRIPTION_DIR, 'urdf', 'robot1_gripper.xacro'),
                           {'workcell_config': str(layout), 'controller_config_file': ''})
    visuals = wc.zero_pose_visuals(urdf)
    base_mesh = [v for v in visuals if v.geometry.tag == 'mesh'
                 and v.geometry.get('filename').endswith('gp25/visual/base_link.dae')]
    assert len(base_mesh) == 1
    # Rail height 0.2 + half the 0.05 carriage: the arm stands on the carriage.
    assert base_mesh[0].transform[:3, 3] == pytest.approx([0.0, 0.0, 0.225])


def test_objects_roundtrip_keeps_unedited_keys_and_writes_both_schemas():
    doc = {'/**': {'ros__parameters': {
        'base_link': {'urdf_path': 'urdf/environment/workpiece.urdf.xacro', 'id': 'base_link',
                      'spawn_name': 'wp', 'pose': {'x': 1.2, 'y': -0.5, 'z': 0.65},
                      'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
        'child_link': {'urdf_path': 'urdf/environment/cube.urdf.xacro', 'id': 'child_link',
                       'spawn_name': 'cube', 'pose': {'x': 1.2, 'y': 0.3, 'z': 0.125},
                       'end_pose': {'position': {'x': 1.2, 'y': -0.5, 'z': 0.925}}},
        'frame_id': 'world'}}}
    slots = {s.name: s for s in wc.object_slots_from_objects(doc)}
    assert slots['child_link'].type_name == 'cube'

    slots['child_link'].type_name = 'cylinder'
    slots['child_link'].position = [2.0, 1.0, 0.125]
    slots['child_link'].orientation = wc.yaw_to_quaternion(0.5)
    out = wc.updated_objects(doc, list(slots.values()))

    child = out['/**']['ros__parameters']['child_link']
    assert child['urdf_path'] == 'urdf/environment/cylinder.urdf.xacro'
    assert child['spawn_name'] == 'cube'
    assert child['end_pose'] == doc['/**']['ros__parameters']['child_link']['end_pose']
    # Gazebo spawner reads pose.q*, planning-scene readers read orientation.
    assert child['pose']['qz'] == child['orientation']['z'] == pytest.approx(math.sin(0.25))
    assert doc['/**']['ros__parameters']['child_link']['pose']['x'] == 1.2, 'input mutated'

    reread = {s.name: s for s in wc.object_slots_from_objects(out)}
    assert reread['child_link'].position == pytest.approx([2.0, 1.0, 0.125])
    assert wc.quaternion_to_yaw(reread['child_link'].orientation) == pytest.approx(0.5)


def test_write_yaml_keeps_header_and_writes_through_symlink(tmp_path):
    target = tmp_path / 'real.yaml'
    target.write_text('# header line\n\nrobots: {}\n')
    link = tmp_path / 'link.yaml'
    link.symlink_to(target)

    wc.write_yaml(str(link), {'robots': {'robot1': {'model': 'gp25'}}})

    assert link.is_symlink(), 'a --symlink-install link must not be replaced by a copy'
    text = target.read_text()
    assert text.startswith('# header line\n\n')
    assert yaml.safe_load(text) == {'robots': {'robot1': {'model': 'gp25'}}}


def test_saved_layout_is_what_xacro_builds(tmp_path):
    """The configurator's workcell.yaml output must drive dual_robot.xacro as placed."""
    workcell = wc.load_yaml(os.path.join(DESCRIPTION_DIR, 'config', 'workcell.yaml'))
    slots = {s.name: s for s in wc.robot_slots_from_workcell(workcell)}
    slots['robot1'].position = [0.5, 1.5, 0.0]
    slots['robot1'].orientation = wc.yaw_to_quaternion(math.pi / 2)
    slots['robot2'].type_name = 'ar2010'
    slots['robot2'].position = [3.0, -1.0, 0.1]
    slots['robot2'].orientation = wc.yaw_to_quaternion(-1.0)
    path = tmp_path / 'workcell.yaml'
    wc.write_yaml(str(path), wc.updated_workcell(workcell, list(slots.values())))

    urdf = wc.expand_xacro(os.path.join(DESCRIPTION_DIR, 'urdf', 'dual_robot.xacro'),
                           {'workcell_config': str(path), 'controller_config_file': ''})
    robot = ET.fromstring(urdf)
    joints = {j.get('name'): j for j in robot.findall('joint')}

    def origin(joint_name):
        o = joints[joint_name].find('origin')
        return [float(v) for v in o.get('xyz').split() + o.get('rpy').split()]

    assert origin('robot1_rail_base_joint') == pytest.approx(
        [0.5, 1.5, 0.0, 0.0, 0.0, math.pi / 2])
    assert origin('robot2_base_joint') == pytest.approx([3.0, -1.0, 0.1, 0.0, 0.0, -1.0])
    meshes = {m.get('filename') for m in robot.iter('mesh')}
    assert any('/ar2010/' in m for m in meshes)
    assert any('/gp25/' in m for m in meshes), 'robot1 should still be gp25'


def test_staged_output_path_inserts_suffix_before_extension():
    assert wc.staged_output_path('/a/b/workcell.yaml') == '/a/b/workcell.configurator.yaml'
    assert wc.staged_output_path('/a/b/objects.yml') == '/a/b/objects.configurator.yml'


def test_catalog_discovery_finds_shipped_models_and_objects():
    assert {'gp25', 'ar2010'} <= set(wc.discover_robot_models(DESCRIPTION_DIR))
    assert {'cube', 'workpiece'} <= set(wc.discover_object_types(DESCRIPTION_DIR))
