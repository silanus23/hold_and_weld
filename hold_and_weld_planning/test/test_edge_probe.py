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

"""Unit tests for the edge probe on synthetic geometry with known answers."""

import numpy as np
import pytest
import trimesh

from hold_and_weld_planning.mesh.edge_probe import EdgeProbe, ProbeMesh

EDGE_LEN = 0.01  # grid step of all synthetic meshes


def _grid_faces(nx, ny):
    """Triangulate an (nx+1)x(ny+1) vertex grid into 2*nx*ny faces."""
    faces = []
    for i in range(nx):
        for j in range(ny):
            v00 = i * (ny + 1) + j
            v01 = v00 + 1
            v10 = (i + 1) * (ny + 1) + j
            v11 = v10 + 1
            faces.append([v00, v10, v11])
            faces.append([v00, v11, v01])
    return faces


def make_wedge(dihedral_deg, width=0.2, length=0.2, h=EDGE_LEN):
    """Two rectangular half-planes meeting along the y-axis crease.

    Half-plane A lies in z=0 spanning +x (normal +z). Half-plane B rises so
    the angle between the face normals equals dihedral_deg. Not watertight —
    the probe only reads normals/areas/adjacency, so an open sheet is fine.
    """
    theta = np.radians(dihedral_deg)
    nx = int(round(width / h))
    ny = int(round(length / h))

    dir_b = np.array([-np.cos(theta), 0.0, np.sin(theta)])

    verts = []
    for i in range(nx, 0, -1):  # half-plane B, far to near
        for j in range(ny + 1):
            verts.append(i * h * dir_b + [0.0, j * h, 0.0])
    for i in range(nx + 1):  # crease row (i == 0) then half-plane A
        for j in range(ny + 1):
            verts.append([i * h, j * h, 0.0])

    faces = _grid_faces(2 * nx, ny)
    return trimesh.Trimesh(vertices=verts, faces=faces, process=False)


def make_cylinder_sheet(radius=0.08, height=0.2, sections=128, h=EDGE_LEN):
    """Finely tessellated open cylinder wall (smooth curvature, no edges)."""
    nz = int(round(height / h))
    verts = []
    for i in range(sections + 1):
        phi = 2.0 * np.pi * i / sections
        for j in range(nz + 1):
            verts.append(
                [radius * np.cos(phi), radius * np.sin(phi), j * h]
            )
    faces = _grid_faces(sections, nz)
    return trimesh.Trimesh(vertices=verts, faces=faces, process=False)


@pytest.fixture(scope='module')
def probe():
    return EdgeProbe({})


class TestWedge:
    """Probe at the crease of wedges with known dihedral angles."""

    @pytest.mark.parametrize('angle', [90.0, 45.0, 20.0])
    def test_sharp_wedge_is_edge(self, probe, angle):
        pm = ProbeMesh(make_wedge(angle))
        crease = np.array([0.0, 0.1, 0.0])
        r = probe.probe(pm, crease)
        assert r.is_edge, f'{angle} deg wedge not detected as edge: {r}'
        assert abs(np.degrees(r.dihedral) - angle) < 3.0
        assert r.gap_ratio > 0.9

    def test_shallow_wedge_is_not_edge(self, probe):
        # 5 deg is a tessellation-crease magnitude, below the 15 deg floor.
        pm = ProbeMesh(make_wedge(5.0))
        r = probe.probe(pm, np.array([0.0, 0.1, 0.0]))
        assert not r.is_edge
        assert np.degrees(r.dihedral) < 15.0

    def test_edge_direction_along_crease(self, probe):
        pm = ProbeMesh(make_wedge(90.0))
        r = probe.probe(pm, np.array([0.0, 0.1, 0.0]))
        assert r.edge_dir is not None
        assert abs(abs(r.edge_dir[1]) - 1.0) < 0.05  # +-y

    def test_flat_region_is_not_edge(self, probe):
        pm = ProbeMesh(make_wedge(90.0))
        # Deep inside half-plane A, > probe radius (4 edges) from the crease.
        r = probe.probe(pm, np.array([0.1, 0.1, 0.0]))
        assert not r.is_edge
        assert r.spread < 1e-6

    def test_response_peaks_at_crease(self, probe):
        pm = ProbeMesh(make_wedge(90.0))
        at_crease = probe.probe(pm, np.array([0.0, 0.1, 0.0])).response
        near = probe.probe(pm, np.array([2 * EDGE_LEN, 0.1, 0.0])).response
        off = probe.probe(pm, np.array([3.5 * EDGE_LEN, 0.1, 0.0])).response
        assert at_crease > near > off

    def test_balance_centered_at_crease(self, probe):
        pm = ProbeMesh(make_wedge(90.0))
        r = probe.probe(pm, np.array([0.0, 0.1, 0.0]))
        assert r.balance < 0.65  # roughly even split between the two planes


class TestCurvature:
    """Smooth curvature must never classify as an edge."""

    def test_fine_cylinder_is_not_edge(self, probe):
        pm = ProbeMesh(make_cylinder_sheet(sections=128))
        point = np.array([0.08, 0.0, 0.1])
        r = probe.probe(pm, point)
        assert not r.is_edge
        # Variation exists but is distributed: no dominant single step.
        assert r.spread > 0.01
        assert r.gap_ratio < 0.5

    def test_coarse_cylinder_is_not_edge(self, probe):
        # 32 sections -> 11.25 deg facet steps: below the dihedral floor
        # even where one step dominates locally.
        pm = ProbeMesh(make_cylinder_sheet(sections=32))
        for phi in np.linspace(0.0, np.pi / 4, 5):
            point = np.array([0.08 * np.cos(phi), 0.08 * np.sin(phi), 0.1])
            r = probe.probe(pm, point)
            assert not r.is_edge, f'coarse cylinder flagged edge at phi={phi}: {r}'


class TestBox:
    """Watertight primitive: box edges and faces."""

    def test_box_edge_and_face(self, probe):
        box = trimesh.creation.box(extents=[0.2, 0.2, 0.2])
        box = box.subdivide().subdivide().subdivide()
        pm = ProbeMesh(box)

        on_edge = probe.probe(pm, np.array([0.1, 0.0, 0.1]))
        assert on_edge.is_edge
        assert abs(np.degrees(on_edge.dihedral) - 90.0) < 3.0

        on_face = probe.probe(pm, np.array([0.1, 0.0, 0.0]))
        assert not on_face.is_edge

    def test_degenerate_input(self, probe):
        box = trimesh.creation.box(extents=[0.2, 0.2, 0.2])
        pm = ProbeMesh(box)  # 12 faces, huge edges: probe ball spans few faces
        r = probe.probe(pm, np.array([0.1, 0.0, 0.1]))
        # Must not crash; any sane answer is acceptable on a degenerate mesh.
        assert r.num_faces >= 0
