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

"""Unit tests for the OCCT pipeline: loading, URDF assembly and seam extraction."""

from hold_and_weld_planning.mesh.path_creator import PathCreator
from hold_and_weld_planning.mesh.seam_point import SeamPoint
from hold_and_weld_planning.occt.occt_generator import OCCTGenerator
from hold_and_weld_planning.occt.occt_loader import OCCTLoader
from hold_and_weld_planning.occt.seam_extractor_occt import SeamExtractorOCCT
import numpy as np
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.GeomLProp import GeomLProp_SLProps
from OCC.Core.gp import gp_Ax3, gp_Dir, gp_Pln, gp_Pnt
from OCC.Core.IGESControl import IGESControl_Writer
from OCC.Core.STEPControl import STEPControl_AsIs, STEPControl_Writer
import pytest
from urdf_parser_py.urdf import URDF

PARAMS = {'epsilon': 1e-3, 'num_smooth_points': 50}


def extent(shape):
    box = Bnd_Box()
    brepbndlib.Add(shape, box)
    x0, y0, z0, x1, y1, z1 = box.Get()
    return np.array([x1 - x0, y1 - y0, z1 - z0])


def cube_mm(size_mm=100.0):
    """Model a cube as a millimetre CAD tool would."""
    return BRepPrimAPI_MakeBox(size_mm, size_mm, size_mm).Shape()


def urdf(links, joints=''):
    return URDF.from_xml_string(f'<robot name="part">{links}{joints}</robot>')


def box_link(name, size, xyz=(0.0, 0.0, 0.0)):
    x, y, z = xyz
    sx, sy, sz = size
    return (
        f'<link name="{name}"><collision>'
        f'<origin xyz="{x} {y} {z}" rpy="0 0 0"/>'
        f'<geometry><box size="{sx} {sy} {sz}"/></geometry>'
        f'</collision></link>'
    )


def fixed_joint(parent, child, xyz):
    x, y, z = xyz
    return (
        f'<joint name="{parent}_{child}" type="fixed">'
        f'<parent link="{parent}"/><child link="{child}"/>'
        f'<origin xyz="{x} {y} {z}" rpy="0 0 0"/></joint>'
    )


PLATE = urdf(box_link('plate', (0.4, 0.4, 0.02)))


def extract(main_robot, secondary_robot):
    main = OCCTGenerator(main_robot).create_shape_for_all_links()
    secondary = OCCTGenerator(secondary_robot).create_shape_for_all_links()
    return SeamExtractorOCCT(main, secondary, dict(PARAMS)).extract_seams()


class TestUnits:
    """The pipeline works in metres whatever unit the CAD file was saved in."""

    def test_a_millimetre_step_loads_in_metres(self, tmp_path):
        path = tmp_path / 'cube.step'
        writer = STEPControl_Writer()
        writer.Transfer(cube_mm(), STEPControl_AsIs)
        writer.Write(str(path))

        shape = OCCTLoader(path).shape
        np.testing.assert_allclose(extent(shape), [0.1, 0.1, 0.1], atol=1e-6)

    @pytest.mark.parametrize('unit,size', [('MM', 100.0), ('M', 0.1)])
    def test_an_iges_loads_in_metres_whatever_unit_it_declares(self, tmp_path, unit, size):
        path = tmp_path / f'cube_{unit}.igs'
        writer = IGESControl_Writer(unit, 0)
        writer.AddShape(BRepPrimAPI_MakeBox(size, size, size).Shape())
        writer.ComputeModel()
        writer.Write(str(path))

        shape = OCCTLoader(path).shape
        np.testing.assert_allclose(extent(shape), [0.1, 0.1, 0.1], atol=1e-6)

    def test_the_world_pose_moves_the_part_in_metres(self, tmp_path):
        path = tmp_path / 'cube.step'
        writer = STEPControl_Writer()
        writer.Transfer(cube_mm(), STEPControl_AsIs)
        writer.Write(str(path))

        pose = np.eye(4)
        pose[:3, 3] = [1.0, 0.0, 0.0]
        box = Bnd_Box()
        brepbndlib.Add(OCCTLoader(path, pose).shape, box)
        assert box.Get()[0] == pytest.approx(1.0, abs=1e-6)


class TestMultiLinkAssembly:
    """Links of one part are one solid; where they meet is not a seam."""

    def test_two_links_give_the_seams_of_the_block_they_form(self):
        # Two 0.1 cubes side by side on the plate form one 0.2 x 0.1 block. The joint between
        # the links is internal to the part and must not be welded.
        split = urdf(
            box_link('left', (0.1, 0.1, 0.1), (-0.05, 0.0, 0.06))
            + box_link('right', (0.1, 0.1, 0.1), (0.0, 0.0, 0.0)),
            fixed_joint('left', 'right', (0.05, 0.0, 0.06)),
        )
        whole = urdf(box_link('block', (0.2, 0.1, 0.1), (0.0, 0.0, 0.06)))

        split_seams = extract(PLATE, split)
        whole_seams = extract(PLATE, whole)

        assert len(whole_seams) > 0
        assert len(split_seams) == len(whole_seams)

        for seam in split_seams:
            xs = seam.config['smoothed_points'][:, 0]
            assert not np.allclose(xs, 0.0, atol=1e-6), 'seam along the internal link joint'


class TestClosedCurves:
    """The welder runs one Pilz CIRC per arc, which needs distinct start and goal."""

    def test_a_full_circle_is_split_into_arcs_with_distinct_ends(self):
        cylinder = urdf(
            '<link name="pin"><collision><origin xyz="0 0 0.06" rpy="0 0 0"/>'
            '<geometry><cylinder radius="0.05" length="0.1"/></geometry>'
            '</collision></link>'
        )
        seams = extract(PLATE, cylinder)

        arcs = [s for s in seams if s.config['geometry_type'] == 'arc']
        assert arcs, 'the circular seam should come out as arcs'
        total = sum(s.length() for s in arcs)
        assert total == pytest.approx(2.0 * np.pi * 0.05, rel=1e-3)
        for seam in arcs:
            points = seam.config['smoothed_points']
            assert np.linalg.norm(points[-1] - points[0]) > 1e-3
            assert len(seam.config['normals_main']) == len(points)
            assert len(seam.config['normals_secondary']) == len(points)


class TestPlaneNormals:
    """The planar fast path must agree with the general surface evaluation."""

    @pytest.mark.parametrize('left_handed', [False, True])
    def test_fast_path_matches_the_surface_normal(self, left_handed):
        axes = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0))
        if left_handed:
            axes.YReverse()
        face = BRepBuilderAPI_MakeFace(gp_Pln(axes), -1.0, 1.0, -1.0, 1.0).Face()
        assert BRepAdaptor_Surface(face).Plane().Position().Direct() != left_handed

        props = GeomLProp_SLProps(BRep_Tool.Surface(face), 0.0, 0.0, 1, 1e-6)
        truth = props.Normal()
        expected = np.array([truth.X(), truth.Y(), truth.Z()])

        box = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape()
        extractor = SeamExtractorOCCT(box, box, dict(PARAMS))
        normal = extractor._evaluate_normal_at_point(np.zeros(3), face)

        np.testing.assert_allclose(normal, expected, atol=1e-12)


class TestOutputParity:
    """Both pipelines write the same seam config keys into the welder JSON."""

    def test_occt_seams_carry_the_mesh_pipelines_keys(self):
        mesh_points = [
            SeamPoint(position=np.array([x, 0.0, 0.0]), normal_base=np.array([0.0, 0.0, 1.0]),
                      normal_wall=np.array([1.0, 0.0, 0.0]), on_edge_1=False, on_edge_2=True,
                      owner_side=2)
            for x in np.linspace(0.0, 0.1, 20)
        ]
        mesh_keys = set(PathCreator().process_path(mesh_points)[0].config)

        block = urdf(box_link('block', (0.1, 0.1, 0.1), (0.0, 0.0, 0.06)))
        pin = urdf(
            '<link name="pin"><collision><origin xyz="0 0 0.06" rpy="0 0 0"/>'
            '<geometry><cylinder radius="0.05" length="0.1"/></geometry>'
            '</collision></link>'
        )
        occt_seams = extract(PLATE, block) + extract(PLATE, pin)

        assert {s.config['geometry_type'] for s in occt_seams} == {'line', 'arc'}
        for seam in occt_seams:
            assert set(seam.config) == mesh_keys
