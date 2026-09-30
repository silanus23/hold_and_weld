# Copyright 2026 Berkan Tali
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for input handling: loaders, generators, config and job-level failures."""

import contextlib
import logging

from hold_and_weld_planning import seam_generator
from hold_and_weld_planning.core.line_segment import LineSegment
from hold_and_weld_planning.core.seam import Seam
from hold_and_weld_planning.mesh.mesh_loader import MeshLoader
from hold_and_weld_planning.mesh.shell_generator import ShellGenerator
from hold_and_weld_planning.occt import seam_extractor_occt
from hold_and_weld_planning.occt.occt_generator import OCCTGenerator
from hold_and_weld_planning.occt.seam_extractor_occt import SeamExtractorOCCT
from hold_and_weld_planning.planning import job_planner
from hold_and_weld_planning.planning.job_planner import JobPlanner
from hold_and_weld_planning.utils import transforms
from hold_and_weld_planning.utils.path_utils import load_urdf_config
from hold_and_weld_planning.utils.transforms import numpy_to_gp_trsf
import numpy as np
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.gp import gp_Pnt
import pytest
import trimesh
from urdf_parser_py.urdf import URDF

REQUIRED = {'work_angle_deg': 45.0, 'travel_angle_deg': 0.0, 'gap_mm': 1.0}


@contextlib.contextmanager
def warnings_logged(module_logger):
    """Collect WARNING and above from one module's logger.

    Attached to the module logger itself rather than caplog: with ROS sourced, the launch
    pytest plugin makes loggers that do not propagate, so caplog captures nothing.
    """
    records = []
    handler = logging.Handler(logging.WARNING)
    handler.emit = records.append
    module_logger.addHandler(handler)
    try:
        yield records
    finally:
        module_logger.removeHandler(handler)


def text(records):
    return '\n'.join(record.getMessage() for record in records)


def write_yaml(tmp_path, text):
    path = tmp_path / 'job.yaml'
    path.write_text(text)
    return path


WORKPIECE = """
workpiece:
  main_part:
    main_path: a.stl
  secondary_part:
    secondary_path: b.stl
"""


class TestMeshLoader:

    def test_a_multi_solid_stl_loads_as_the_union_of_its_solids(self, tmp_path):
        # Two overlapping unit cubes: trimesh reads the file as a Scene of two meshes.
        first = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
        second = trimesh.creation.box(extents=(1.0, 1.0, 1.0))
        second.apply_translation((0.5, 0.0, 0.0))
        path = tmp_path / 'two_solids.stl'
        path.write_bytes(
            trimesh.exchange.stl.export_stl_ascii(first).encode()
            + trimesh.exchange.stl.export_stl_ascii(second).encode()
        )
        assert isinstance(trimesh.load(path), trimesh.Scene)

        loader = MeshLoader(path, refine_iterations=0)
        assert loader.manifold.volume() == pytest.approx(1.5, rel=1e-9)


NO_COLLISION = URDF.from_xml_string(
    '<robot name="part"><link name="base"><visual><geometry>'
    '<box size="1 1 1"/></geometry></visual></link></robot>'
)


class TestGenerators:

    def test_shell_generator_refuses_a_part_with_no_collision_geometry(self):
        with pytest.raises(ValueError, match='collision'):
            ShellGenerator(NO_COLLISION, refine_iterations=0).create_shells_for_all_links()

    def test_occt_generator_refuses_a_part_with_no_collision_geometry(self):
        with pytest.raises(ValueError, match='collision'):
            OCCTGenerator(NO_COLLISION).create_shape_for_all_links()

    @pytest.mark.parametrize('build', [
        lambda robot: ShellGenerator(robot, refine_iterations=0).create_shells_for_all_links(),
        lambda robot: OCCTGenerator(robot).create_shape_for_all_links(),
    ], ids=['mesh', 'occt'])
    @pytest.mark.parametrize('size', ['0 1 1', '1 -1 1'])
    def test_a_non_positive_box_is_a_value_error(self, build, size):
        # ValueError is what the CLI reports as a configuration error rather than a crash.
        robot = URDF.from_xml_string(
            '<robot name="part"><link name="base"><collision><geometry>'
            f'<box size="{size}"/></geometry></collision></link></robot>'
        )
        with pytest.raises(ValueError, match='positive'):
            build(robot)


class TestConfig:

    def test_seams_are_not_required_when_auto_detect_is_off(self, tmp_path):
        # The CLI never reads a hand-written seam list, so demanding one only blocks the job.
        path = write_yaml(tmp_path, WORKPIECE + 'parameters:\n  gap_mm: 1.0\n')
        _, parameters, workpiece = load_urdf_config(path)
        assert parameters == {'gap_mm': 1.0}
        assert workpiece['main_part']['main_path'] == 'a.stl'

    @pytest.mark.parametrize('text', [
        '',
        'workpiece:\n',
        'workpiece:\n  main_part:\n  secondary_part:\n',
        WORKPIECE + 'parameters:\n',
        WORKPIECE + 'parameters: [1, 2]\n',
        'workpiece: [\n',
    ])
    def test_malformed_config_is_a_value_error(self, tmp_path, text):
        with pytest.raises(ValueError):
            load_urdf_config(write_yaml(tmp_path, text))

    def test_verbose_reports_a_missing_parameter_as_configuration(self, tmp_path, monkeypatch):
        path = write_yaml(tmp_path, WORKPIECE + 'parameters:\n  gap_mm: 1.0\n')
        monkeypatch.setattr('sys.argv', ['seam_generator', '-i', str(path), '-v'])
        monkeypatch.setattr(seam_generator, 'setup_logging', lambda verbose: None)
        with warnings_logged(logging.getLogger(seam_generator.__name__)) as records:
            assert seam_generator.main() == 1
        assert 'Invalid configuration' in text(records)


class TestTransforms:

    def test_shear_is_reported(self):
        shear = np.eye(4)
        shear[0, 1] = 0.5
        assert np.linalg.det(shear[:3, :3]) == pytest.approx(1.0)
        with warnings_logged(transforms.logger) as records:
            numpy_to_gp_trsf(shear)
        assert 'shear' in text(records)


class TestJobLevelFailures:
    """A job missing a seam is the wrong job, not a smaller one."""

    def test_a_seam_that_fails_to_plan_fails_the_job(self):
        planner = JobPlanner('a.stl', 'b.stl', parameters=dict(REQUIRED))
        good = Seam(line_segment=LineSegment(start=np.zeros(3), end=np.array([0.0, 0.1, 0.0])))
        good.config.update({
            'smoothed_points': np.array([[0.0, 0.0, 0.0], [0.0, 0.1, 0.0]]),
            'normals_main': np.array([[0.0, 0.0, 1.0]] * 2),
            'normals_secondary': np.array([[1.0, 0.0, 0.0]] * 2),
            'is_edge_joint': False,
        })
        broken = Seam(line_segment=LineSegment(start=np.zeros(3), end=np.ones(3)))

        with pytest.raises(RuntimeError, match='seam 1'):
            planner._generate_poses([good, broken])

    def test_an_occt_edge_that_fails_fails_the_extraction(self, monkeypatch):
        plate = BRepPrimAPI_MakeBox(1.0, 1.0, 0.1).Shape()
        block = BRepPrimAPI_MakeBox(gp_Pnt(0.25, 0.25, 0.1), 0.5, 0.5, 0.5).Shape()
        extractor = SeamExtractorOCCT(plate, block, {'epsilon': 1e-3})

        def fail(edge_data):
            raise RuntimeError('boom')
        monkeypatch.setattr(extractor, '_process_single_edge', fail)

        with pytest.raises(RuntimeError, match='boom'):
            extractor.extract_seams()

    def test_an_occt_face_pair_that_fails_fails_the_extraction(self, monkeypatch):
        plate = BRepPrimAPI_MakeBox(1.0, 1.0, 0.1).Shape()
        block = BRepPrimAPI_MakeBox(gp_Pnt(0.25, 0.25, 0.1), 0.5, 0.5, 0.5).Shape()
        extractor = SeamExtractorOCCT(plate, block, {'epsilon': 1e-3})

        def fail(*args):
            raise RuntimeError('boom')
        monkeypatch.setattr(seam_extractor_occt, 'BRepAlgoAPI_Common', fail)

        with pytest.raises(RuntimeError, match='boom'):
            extractor.extract_seams()

    def test_an_unknown_parameter_key_is_warned_about_by_name(self):
        with warnings_logged(job_planner.logger) as records:
            JobPlanner('a.stl', 'b.stl', parameters=dict(REQUIRED, path_tolerence_mm=2.0))
        assert 'path_tolerence_mm' in text(records)

    def test_known_keys_raise_no_warning(self):
        known = dict(
            REQUIRED, path_tolerance_mm=1.0, epsilon=0.002, refine_iterations=16,
            waypoint_spacing_mm=10.0, num_smooth_points=100, coincidence_samples=5,
            near_contact_edge_fraction=0.1,
        )
        with warnings_logged(job_planner.logger) as records:
            JobPlanner('a.stl', 'b.stl', parameters=known)
        assert not records
