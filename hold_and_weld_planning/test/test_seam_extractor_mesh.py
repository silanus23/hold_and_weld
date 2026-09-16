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

"""Unit tests for `SeamExtractorMesh` and its fields, on synthetic boxes.

Every expected value here is derived from the geometry, never from what the
code happens to return. Two boxes give an exactly known contact region, and a
box overhanging a plate puts the true seam corner BETWEEN mesh vertices, which
is the case the coverage field exists to solve.
"""

from hold_and_weld_planning.mesh.chaining import stitch
from hold_and_weld_planning.mesh.mesh_fields import reject_holes
from hold_and_weld_planning.mesh.params import SeamExtractorMeshParams
from hold_and_weld_planning.mesh.seam_extractor_mesh import SeamExtractorMesh

import numpy as np
import pytest
import trimesh

UP = np.array([0.0, 0.0, 1.0])
PARAMS = {'epsilon': 0.001}

SUBDIVISIONS = 5
ROWS = 2 ** SUBDIVISIONS            # face rows per box side
SPACING = 1.0 / ROWS                # plate rim sampling, 0.03125m

# Both parts are 0.4+ thick against epsilon=1mm, so the wall-marking cliff sits
# at 0.4/(3*0.001) = 133 rows and these scenes run at 32 - a 4x margin. That
# cliff is per PART and set by the THINNEST one: at 128 rows a 0.2-thick box
# has its wall wrongly marked as contact and the seam collapses, refinement or
# no refinement. Keep the margin if these numbers are ever changed.
PLATE = (1.0, 1.0, 0.5)
BLOCK = (0.4, 0.4, 0.4)
PLATE_TOP = 0.25


def box(extents, center, subdivisions=SUBDIVISIONS):
    """Axis-aligned box, subdivided so its faces are small against rho."""
    mesh = trimesh.creation.box(extents=extents)
    mesh.apply_translation(center)
    for _ in range(subdivisions):
        mesh = mesh.subdivide()
    return mesh


@pytest.fixture
def seated():
    """Build a 0.4 block seated wholly on a 1.0 plate, meeting at z=0.25.

    Contact region is the block's footprint, x,y in [-0.2, 0.2].
    """
    plate = box(PLATE, (0.0, 0.0, 0.0))
    block = box(BLOCK, (0.0, 0.0, PLATE_TOP + 0.2))
    return SeamExtractorMesh(plate, block, PARAMS)


@pytest.fixture
def overhanging():
    """Build a 0.4 block hung off the plate's +x rim, seam crossing both parts.

    Contact region is x in [0.4, 0.5], y in [-0.2, 0.2]. The seam is that
    rectangle's perimeter - 1.0m - with the block's own rim on three sides and
    the plate's top rim on the fourth, so the true corners sit at (0.5, +-0.2).
    The plate's rim is sampled at 1/32, and 0.2 is not a multiple of it, so
    neither corner exists as a vertex on either part.
    """
    plate = box(PLATE, (0.0, 0.0, 0.0))
    block = box(BLOCK, (0.6, 0.0, PLATE_TOP + 0.2))
    return SeamExtractorMesh(plate, block, PARAMS)


def brute_force_edge_distance(extractor, side, point):
    """Nearest point-to-segment distance over EVERY sharp edge of one mesh."""
    mesh = extractor.mesh[side]
    edges = mesh.face_adjacency_edges[
        mesh.face_adjacency_angles > extractor.cfg.edge_angle_min]
    a, b = mesh.vertices[edges[:, 0]], mesh.vertices[edges[:, 1]]
    ab = b - a
    span = np.einsum('ij,ij->i', ab, ab)
    span = np.where(span > 0.0, span, 1e-30)
    t = np.clip(np.einsum('ij,ij->i', point - a, ab) / span, 0.0, 1.0)
    return float(np.linalg.norm(a + t[:, None] * ab - point, axis=1).min())


def unweighted_coverage(extractor, side, point, rho):
    """`MeshFields.coverage` with the facing weight removed, for comparison only."""
    other = extractor.mesh[2 if side == 1 else 1]
    index = np.asarray(
        extractor.fields.centroid_tree(2 if side == 1 else 1)
        .query_ball_point(point, rho), dtype=np.int64)
    if not len(index):
        return 0.0
    t = np.linalg.norm(other.triangles_center[index] - point, axis=1) / rho
    weight = (1.0 - t ** 2) ** 3 * other.area_faces[index]
    return float(weight.sum() / (np.pi * rho ** 2 / 4.0))


class TestCoverage:
    """The field itself: 1 inside the contact, 1/2 on the boundary, 0 outside."""

    RHO = 0.05

    def test_reads_one_well_inside_the_contact(self, seated):
        # The block's bottom face fills the neighbourhood, so the kernel sums
        # to the full-plane weight it is normalised by.
        c = seated.fields.coverage(1, np.array([0.0, 0.0, PLATE_TOP]), self.RHO, UP)
        assert c == pytest.approx(1.0, abs=0.05)

    def test_reads_one_half_on_the_contact_boundary(self, seated):
        # On the block's rim its bottom face covers a HALF plane, which
        # integrates to half the full-plane weight. This is what makes 1/2 the
        # boundary by construction rather than by tuning.
        c = seated.fields.coverage(1, np.array([0.2, 0.0, PLATE_TOP]), self.RHO, UP)
        assert c == pytest.approx(0.5, abs=0.05)

    def test_reads_zero_outside_the_contact(self, seated):
        # Further than rho beyond the rim, no block triangle is in range.
        c = seated.fields.coverage(1, np.array([0.32, 0.0, PLATE_TOP]), self.RHO, UP)
        assert c == pytest.approx(0.0, abs=1e-9)

    def test_crosses_one_half_exactly_once(self, seated):
        # What the bisection in slide_to_boundary actually needs. NOT global
        # monotonicity: on the saturated plateau the centroid-sampled integral
        # wobbles by a few parts in a thousand and does rise slightly. That is
        # harmless because the wobble sits entirely above 1/2, so it cannot
        # manufacture a second crossing - which is the property asserted here.
        xs = np.linspace(0.10, 0.30, 41)
        c = np.array([seated.fields.coverage(1, np.array([x, 0.0, PLATE_TOP]), self.RHO, UP)
                      for x in xs])
        crossings = np.count_nonzero(np.diff(np.signbit(c - 0.5)))
        assert crossings == 1

    def test_is_monotone_through_the_transition_band(self, seated):
        # Away from the plateau, where the answer is actually decided, the
        # field has to fall cleanly.
        xs = np.linspace(0.10, 0.30, 41)
        c = np.array([seated.fields.coverage(1, np.array([x, 0.0, PLATE_TOP]), self.RHO, UP)
                      for x in xs])
        band = c[(c < 0.95) & (c > 0.05)]
        assert all(later <= earlier + 1e-9
                   for earlier, later in zip(band, band[1:]))

    def test_facing_weight_is_what_makes_the_crossing_exist(self, seated):
        # The block's WALL rises from the rim, so it too covers a half plane
        # there and integrates to another 1/2. Unweighted the two sum to about
        # 1 on the boundary - the same value as deep inside - and no half
        # crossing exists to find.
        rim = np.array([0.2, 0.0, PLATE_TOP])
        assert seated.fields.coverage(1, rim, self.RHO, UP) == pytest.approx(
            0.5, abs=0.05)
        assert unweighted_coverage(seated, 1, rim, self.RHO) == pytest.approx(
            1.0, abs=0.1)


class TestSlideToBoundary:
    """Landing a point on the half level set, between vertices."""

    RHO = 0.05

    def test_lands_on_the_rim_from_inside(self, seated):
        landed = seated.fields.slide_to_boundary(
            np.array([0.17, 0.0, PLATE_TOP]), np.array([1.0, 0.0, 0.0]),
            1, self.RHO, UP)
        assert landed is not None
        assert landed[0] == pytest.approx(0.2, abs=0.005)

    def test_returns_none_when_already_outside(self, seated):
        # Coverage below 1/2 at the start: nothing to bracket.
        assert seated.fields.slide_to_boundary(
            np.array([0.30, 0.0, PLATE_TOP]), np.array([1.0, 0.0, 0.0]),
            1, self.RHO, UP) is None

    def test_returns_none_when_the_span_does_not_reach_the_boundary(
        self, seated
    ):
        # Deep inside, the far end of the span is still inside, so there is no
        # crossing in [0, rho] and the point must be left on its vertex.
        assert seated.fields.slide_to_boundary(
            np.array([0.0, 0.0, PLATE_TOP]), np.array([1.0, 0.0, 0.0]),
            1, self.RHO, UP) is None


class TestSharpEdgeDistance:
    """Point-to-segment, against brute force over every sharp edge."""

    def test_matches_brute_force(self, seated):
        rng = np.random.default_rng(0)
        points = rng.uniform(-0.3, 0.3, size=(25, 3))
        points[:, 2] = rng.uniform(0.15, 0.45, size=25)
        for side in (1, 2):
            got = seated.fields.sharp_edge_distance(side, points)
            for i, point in enumerate(points):
                assert got[i] == pytest.approx(
                    brute_force_edge_distance(seated, side, point), abs=1e-9)

    def test_is_zero_on_a_sharp_edge(self, seated):
        # The block's bottom rim is a 90 degree edge at z=PLATE_TOP, x=0.2.
        on_edge = np.array([[0.2, 0.05, PLATE_TOP]])
        assert seated.fields.sharp_edge_distance(2, on_edge)[0] == pytest.approx(
            0.0, abs=1e-9)


class TestRefinement:
    """What the refinement must and must not do to a chain."""

    def test_leaves_the_chain_alone_when_no_boundary_crosses_it(self, seated):
        # The block sits wholly on the plate, so around every point of its rim
        # the plate fills the neighbourhood: coverage is saturated, no half
        # crossing exists, and refinement has nothing to do. The seam is the
        # block's footprint perimeter, 4 x 0.4m.
        chains = seated.extract_chains()
        assert len(chains) == 1
        points, is_closed = chains[0]
        assert is_closed

        positions = np.array([p.position for p in points])
        assert np.allclose(positions[:, 2], PLATE_TOP, atol=1e-9)
        assert np.abs(positions[:, :2]).max() == pytest.approx(0.2, abs=1e-9)

    def test_recovers_a_corner_that_is_not_in_the_mesh(self, overhanging):
        # Seam is the perimeter of x in [0.4, 0.5], y in [-0.2, 0.2] -> 1.0m,
        # and neither corner at (0.5, +-0.2) is a vertex on either part.
        points, is_closed = overhanging.extract_chains()[0]
        assert is_closed

        positions = np.array([p.position for p in points])
        perimeter = float(np.sum(np.linalg.norm(np.diff(positions, axis=0),
                                                axis=1)))
        perimeter += float(np.linalg.norm(positions[0] - positions[-1]))

        # A quarter of the plate's rim spacing. Snapping to the nearest vertex
        # cannot reach this: measured, leaving the chain on its vertices gives
        # +16.3mm while the field gives +0.26mm.
        assert perimeter == pytest.approx(1.0, abs=0.25 * SPACING)

    @pytest.mark.parametrize('thickness', [0.2, 0.3, 0.4])
    def test_is_stable_against_geometry_that_does_not_touch_the_seam(
        self, thickness
    ):
        # The block's THICKNESS cannot move the seam - the contact region and
        # its boundary are identical for all of these.
        #
        # The corner point does read exactly one half by construction, so
        # whether it survives IS a coin flip settled by sampling noise (0.4840
        # at thickness 0.2, 0.5010 at 0.4, so 60 chain points against 61). This
        # asserts that the flip costs the CHAIN nothing, because the slide
        # reaches the same level set from either neighbour: measured 1000.52mm
        # against 1000.49mm. What the flip does cost is 12.5mm of emitted SEAM,
        # and that is the disjoint-sublist gap below, not this.
        plate = box(PLATE, (0.0, 0.0, 0.0))
        block = box((0.4, 0.4, thickness),
                    (0.6, 0.0, PLATE_TOP + thickness / 2.0))
        points, _ = SeamExtractorMesh(
            plate, block, PARAMS).extract_chains()[0]

        positions = np.array([p.position for p in points])
        perimeter = float(np.sum(np.linalg.norm(np.diff(positions, axis=0),
                                                axis=1)))
        perimeter += float(np.linalg.norm(positions[0] - positions[-1]))
        assert perimeter == pytest.approx(1.0, abs=0.25 * SPACING)

    def test_emitted_seams_span_the_whole_chain(self, overhanging):
        # Partitioning N points into k sublists emits only N-k segments: the
        # step across each boundary needs both endpoints and the split hands
        # them to different sublists. `_join_consecutive` extends each seam to
        # where the next begins, so the emitted seams cover the whole chain.
        # Without it this loses 12.5mm here and 0.77mm on the cylinder scene.
        seams = overhanging.extract_seams()
        assert seams
        total = sum(
            float(np.sum(np.linalg.norm(
                np.diff(s.config['smoothed_points'], axis=0), axis=1)))
            for s in seams
        )
        assert total == pytest.approx(1.0, abs=0.25 * SPACING)


class TestHoleGuard:
    """Telling a genuine crossing from a coverage failure mid-seam."""

    RHO = 0.005

    def _mask(self, n, dropped):
        inside = np.ones(n, dtype=bool)
        inside[dropped] = False
        return inside

    def test_keeps_a_crossing_where_the_ends_meet(self, seated):
        # At a real crossing the seam leaves and re-enters at the same place,
        # so the surviving points either side sit almost on top of each other.
        pos = np.zeros((9, 3))
        pos[2] = [0.0, 0.0, 0.0]
        pos[3:6] = [[0.0, 0.004, 0.0], [0.001, 0.005, 0.0], [0.002, 0.004, 0.0]]
        pos[6] = [0.0008, 0.0, 0.0]          # 0.8mm from pos[2], well under rho
        inside = reject_holes(
            self._mask(9, slice(3, 6)), pos, np.full(9, self.RHO))
        assert not inside[3:6].any(), 'a crossing must stay dropped'

    def test_restores_a_hole_where_the_ends_are_far_apart(self, seated):
        # A coverage failure mid-seam leaves the ends as far apart as the
        # stretch that went missing. Carving that out of a weld is the failure
        # this guard exists to refuse.
        pos = np.column_stack(
            [np.linspace(0.0, 0.040, 9), np.zeros(9), np.zeros(9)])
        inside = reject_holes(
            self._mask(9, slice(3, 6)), pos, np.full(9, self.RHO))
        assert inside.all(), 'a hole must be restored'

    def test_leaves_chain_ends_alone(self, seated):
        # A block touching either end is an ordinary trim, however long.
        pos = np.column_stack(
            [np.linspace(0.0, 0.040, 9), np.zeros(9), np.zeros(9)])
        for dropped in (slice(0, 3), slice(6, 9)):
            inside = reject_holes(
                self._mask(9, dropped), pos, np.full(9, self.RHO))
            assert not inside[dropped].any(), 'an end trim must stay dropped'


class TestStitch:
    """Joining polylines end to end without losing a point to closure."""

    CFG = SeamExtractorMeshParams.from_dict({'min_loop_points': 4})

    def square(self, side=1.0, per_side=9):
        """An OPEN square outline sampled at a uniform spacing.

        The last point stops one spacing short of the first, so the chain
        closes on itself with a gap exactly equal to every other step - and
        no point is a repeat of any other.
        """
        step = side / per_side
        corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)]
        points = []
        for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
            for i in range(per_side):
                t = i / per_side
                points.append([x0 + t * (x1 - x0), y0 + t * (y1 - y0), 0.0])
        return np.asarray(points), step

    def test_closing_a_chain_keeps_every_point(self):
        positions, step = self.square()
        assert len(positions) == 36

        result = stitch([(positions, False)], self.CFG)

        assert len(result) == 1
        closed_points, is_closed = result[0]
        assert is_closed, 'the ends are one spacing apart, so it closes'
        assert len(closed_points) == len(positions), (
            'closure does not make the last point a duplicate: it is a real '
            'sample, one spacing from the first'
        )
        np.testing.assert_allclose(closed_points, positions)

    def test_the_closing_gap_equals_the_sample_spacing(self):
        positions, step = self.square()
        closed_points, _ = stitch([(positions, False)], self.CFG)[0]

        gap = float(np.linalg.norm(closed_points[0] - closed_points[-1]))
        assert gap == pytest.approx(step, rel=1e-9)

    def test_a_repeated_last_point_is_still_dropped(self):
        # The case the strip exists for: the chain literally returns to its
        # first point, so the last one carries no new position.
        positions, _ = self.square()
        repeated = np.vstack([positions, positions[0]])

        closed_points, is_closed = stitch([(repeated, False)], self.CFG)[0]

        assert is_closed
        assert len(closed_points) == len(positions)
        np.testing.assert_allclose(closed_points, positions)

    def test_a_short_straight_chain_is_not_closed(self):
        # Its ends are exactly stitch_gap_factor spacings apart, so spacing alone would close it.
        positions = np.column_stack([np.arange(4) * 1e-3, np.zeros(4), np.zeros(4)])

        _, is_closed = stitch([(positions, False)], self.CFG)[0]

        assert not is_closed

    def test_a_short_turned_chain_still_closes(self):
        positions = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]]) * 1e-3

        _, is_closed = stitch([(positions, False)], self.CFG)[0]

        assert is_closed

    def test_an_open_chain_is_left_open_and_whole(self):
        # Ends far apart: no closure, and nothing trimmed either.
        positions = np.column_stack(
            [np.linspace(0.0, 1.0, 20), np.zeros(20), np.zeros(20)])

        result_points, is_closed = stitch([(positions, False)], self.CFG)[0]

        assert not is_closed
        np.testing.assert_allclose(result_points, positions)
