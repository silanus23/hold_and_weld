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
import struct
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
    layout.write_text(yaml.safe_dump(wc.robot_preview_layout(
        'robot1', 'gp25', 'placeholder')))
    urdf = wc.expand_xacro(os.path.join(DESCRIPTION_DIR, 'urdf', 'robot1_gripper.xacro'),
                           {'workcell_config': str(layout), 'controller_config_file': ''})
    visuals = wc.zero_pose_visuals(urdf)
    base_mesh = [v for v in visuals if v.geometry.tag == 'mesh'
                 and v.geometry.get('filename').endswith('gp25/visual/base_link.dae')]
    assert len(base_mesh) == 1
    # Rail height 0.2 + half the 0.05 carriage: the arm stands on the carriage.
    assert base_mesh[0].transform[:3, 3] == pytest.approx([0.0, 0.0, 0.225])


def test_robot1_preview_shows_the_configured_gripper(tmp_path):
    layout = tmp_path / 'layout.yaml'
    layout.write_text(yaml.safe_dump(wc.robot_preview_layout(
        'robot1', 'gp25', 'rethink_electric')))
    urdf = wc.expand_xacro(os.path.join(DESCRIPTION_DIR, 'urdf', 'robot1_gripper.xacro'),
                           {'workcell_config': str(layout), 'controller_config_file': ''})
    meshes = {m.get('filename') for m in ET.fromstring(urdf).iter('mesh')}
    assert any('/rethink_electric_gripper/' in m for m in meshes)


def _stl_vertices(path):
    with open(path, 'rb') as f:
        data = f.read()
    if data[:5] == b'solid' and b'facet' in data[:300]:
        return np.array([[float(v) for v in line.split()[1:]]
                         for line in data.decode().splitlines()
                         if line.strip().startswith('vertex')])
    count = struct.unpack('<I', data[80:84])[0]
    facets = np.frombuffer(data[84:84 + count * 50], dtype=np.dtype(
        [('normal', '<3f4'), ('vertices', '<9f4'), ('attribute', '<u2')]))
    return facets['vertices'].reshape(-1, 3).astype(float)


def _points_in_frame(transform, points):
    return (transform[:3, :3] @ np.asarray(points).T).T + transform[:3, 3]


def _box_corners(size_attribute):
    size = np.array([float(v) for v in size_attribute.split()])
    return np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * size / 2


def _drawn_points(link, link_transform):
    """Mesh vertices and box corners of a link's visuals, in the frame link_transform maps to."""
    points = []
    for visual in link.findall('visual'):
        geometry = visual.find('geometry')[0]
        if geometry.tag == 'mesh':
            local = _stl_vertices(geometry.get('filename').replace(
                'package://hold_and_weld_description', DESCRIPTION_DIR))
        else:
            local = _box_corners(geometry.get('size'))
        points.append(_points_in_frame(
            link_transform @ wc.origin_transform(visual.find('origin')), local))
    return np.vstack(points)


def _robotiq_gripper(tmp_path):
    """Return the robotiq_2f_140 robot1 URDF root and a finger-link -> gripper_base transform."""
    layout = tmp_path / 'layout.yaml'
    layout.write_text(yaml.safe_dump(wc.robot_preview_layout(
        'robot1', 'gp25', 'robotiq_2f_140')))
    urdf = wc.expand_xacro(os.path.join(DESCRIPTION_DIR, 'urdf', 'robot1_gripper.xacro'),
                           {'workcell_config': str(layout), 'controller_config_file': ''})
    robot = ET.fromstring(urdf)
    joint_to = {j.find('child').get('link'): j for j in robot.findall('joint')}

    def finger_in_gripper_base(side, position=0.0):
        joint = joint_to[f'robot1_{side}_finger']
        transform = wc.origin_transform(joint.find('origin'))
        axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
        transform[:3, 3] += transform[:3, :3] @ axis * position
        return transform

    return robot, finger_in_gripper_base


def test_robotiq_fingers_are_drawn_where_their_pads_grip(tmp_path):
    """Closed (q=0) finger meshes must cover their collision pads, and the pads must meet."""
    robot, finger_in_gripper_base = _robotiq_gripper(tmp_path)
    pad_faces = {}
    for side in ('left', 'right'):
        link = robot.find(f"link[@name='robot1_{side}_finger']")
        link_transform = finger_in_gripper_base(side)
        pad_corners = np.vstack([
            _points_in_frame(link_transform @ wc.origin_transform(collision.find('origin')),
                             _box_corners(collision.find('geometry/box').get('size')))
            for collision in link.findall('collision')])
        drawn_points = _drawn_points(link, link_transform)
        # The collision geometry must sit inside the drawn finger, within 2 mm.
        assert np.all(pad_corners >= drawn_points.min(axis=0) - 0.002), side
        assert np.all(pad_corners <= drawn_points.max(axis=0) + 0.002), side
        # Inner (gripping) face: the side of the pad nearest the gripper centre.
        pad_faces[side] = pad_corners[:, 1].min() if side == 'left' else pad_corners[:, 1].max()

    # 0 = closed: the pads touch, neither gapping nor overlapping by more than 1 mm.
    assert pad_faces['left'] - pad_faces['right'] == pytest.approx(0.0, abs=0.001)


def test_robotiq_fingers_are_drawn_attached_to_the_base(tmp_path):
    """At closed and fully open, each drawn finger must reach up to and under the drawn base."""
    robot, finger_in_gripper_base = _robotiq_gripper(tmp_path)
    base = _drawn_points(robot.find("link[@name='robot1_gripper_base']"), np.eye(4))
    joint = robot.find("joint[@name='robot1_left_finger_joint']")
    travel = float(joint.find('limit').get('upper'))
    for side in ('left', 'right'):
        link = robot.find(f"link[@name='robot1_{side}_finger']")
        for position in (0.0, travel):
            finger = _drawn_points(link, finger_in_gripper_base(side, position))
            label = f'{side} at {position}'
            # Fingers hang along -z: the finger's top must meet the base's bottom.
            assert finger[:, 2].max() >= base[:, 2].min() - 0.002, label
            # ...and sit under the base's footprint, not beside it.
            assert finger[:, 1].min() <= base[:, 1].max(), label
            assert finger[:, 1].max() >= base[:, 1].min(), label


@pytest.mark.parametrize('gripper_model', wc.discover_gripper_models(DESCRIPTION_DIR))
def test_gripper_fingers_can_move_in_gazebo(tmp_path, gripper_model):
    """
    Every catalog gripper meets gripper_catalog.xacro's Gazebo requirements.

    Gazebo drives finger position commands as joint velocity commands: non-zero joint
    friction holds the fingers still, and so does spawning on a limit, where numeric
    noise can leave a finger a hair past it.
    """
    layout = tmp_path / 'layout.yaml'
    layout.write_text(yaml.safe_dump(wc.robot_preview_layout('robot1', 'gp25', gripper_model)))
    urdf = wc.expand_xacro(os.path.join(DESCRIPTION_DIR, 'urdf', 'robot1_gripper.xacro'),
                           {'workcell_config': str(layout), 'controller_config_file': ''})
    robot = ET.fromstring(urdf)
    for side in ('left', 'right'):
        name = f'robot1_{side}_finger_joint'
        joint = robot.find(f"joint[@name='{name}']")
        dynamics = joint.find('dynamics')
        assert dynamics is None or float(dynamics.get('friction', 0.0)) == 0.0, name
        limit = joint.find('limit')
        initial = robot.find(
            f"ros2_control/joint[@name='{name}']/state_interface[@name='position']"
            "/param[@name='initial_value']")
        assert initial is not None, f'{name} has no spawn position'
        assert float(limit.get('lower')) < float(initial.text) < float(limit.get('upper')), name


def test_workcell_needs_gripper_model():
    with pytest.raises(RuntimeError, match='gripper_model'):
        wc.gripper_model_from_workcell({'robots': {}})


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
    wc.write_yaml(str(path), wc.updated_workcell(workcell, list(slots.values()), 'schunk_pg70'))

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
    assert any('/schunk_pg70/' in m for m in meshes), 'robot1 should carry the chosen gripper'


def test_staged_output_path_keeps_the_live_file_name():
    assert wc.staged_output_path('/out', '/a/b/workcell.yaml') == '/out/workcell.yaml'
    assert wc.staged_output_path('/out', '/c/objects.yaml') == '/out/objects.yaml'


def test_output_directory_follows_installed_marker_back_to_source(tmp_path):
    source = tmp_path / 'src' / 'configurator_output'
    source.mkdir(parents=True)
    (source / '.gitignore').write_text('*\n')
    share = tmp_path / 'share'
    (share / 'config' / 'configurator_output').mkdir(parents=True)
    (share / 'config' / 'configurator_output' / '.gitignore').symlink_to(source / '.gitignore')
    assert wc.output_directory(str(share)) == str(source)


def test_workcell_without_gripper_model_fails_xacro_by_name(tmp_path):
    layout = tmp_path / 'layout.yaml'
    layout.write_text(yaml.safe_dump(
        {'robots': {'robot1': {'model': 'gp25', 'x': 0, 'y': 0, 'z': 0, 'yaw': 0}}}))
    with pytest.raises(Exception, match='workcell.yaml has no gripper_model'):
        wc.expand_xacro(os.path.join(DESCRIPTION_DIR, 'urdf', 'robot1_gripper.xacro'),
                        {'workcell_config': str(layout), 'controller_config_file': ''})


def test_gripper_discovery_rejects_a_catalog_not_written_as_a_list(tmp_path):
    catalog = tmp_path / wc.GRIPPER_CATALOG
    catalog.parent.mkdir(parents=True)
    catalog.write_text(
        '<robot xmlns:xacro="http://ros.org/wiki/xacro">'
        '<xacro:property name="gripper_catalog" value="placeholder"/></robot>')
    with pytest.raises(RuntimeError, match='gripper_catalog'):
        wc.discover_gripper_models(str(tmp_path))


def test_srdf_open_states_match_the_live_gripper_travel(tmp_path):
    """The SRDFs' named open states are the live gripper's (workcell.yaml) full travel."""
    workcell_path = os.path.join(DESCRIPTION_DIR, 'config', 'workcell.yaml')
    urdf = wc.expand_xacro(os.path.join(DESCRIPTION_DIR, 'urdf', 'robot1_gripper.xacro'),
                           {'workcell_config': workcell_path, 'controller_config_file': ''})
    robot = ET.fromstring(urdf)
    travel = float(robot.find(
        "joint[@name='robot1_left_finger_joint']/limit").get('upper'))
    for srdf, state in (('dual_robot.srdf', 'robot1_gripper_open'),
                        ('robot1_gripper.srdf', 'gripper_open')):
        group_state = ET.parse(os.path.join(DESCRIPTION_DIR, 'config', srdf)).getroot().find(
            f"group_state[@name='{state}']")
        for joint in group_state.findall('joint'):
            assert float(joint.get('value')) == pytest.approx(travel), (srdf, joint.get('name'))


def test_catalog_discovery_finds_shipped_models_and_objects():
    assert {'gp25', 'ar2010'} <= set(wc.discover_robot_models(DESCRIPTION_DIR))
    assert {'cube', 'workpiece'} <= set(wc.discover_object_types(DESCRIPTION_DIR))
    assert wc.discover_gripper_models(DESCRIPTION_DIR) == [
        'placeholder', 'rethink_electric', 'schunk_pg70', 'onrobot_2fg7', 'robotiq_2f_140']
