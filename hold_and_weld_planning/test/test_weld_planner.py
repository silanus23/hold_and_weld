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

"""Unit tests for WeldPlanner parameter validation and degenerate seam input."""

from hold_and_weld_planning.core.line_segment import LineSegment
from hold_and_weld_planning.core.seam import Seam
from hold_and_weld_planning.planning.weld_planner import WeldPlanner
import numpy as np
import pytest

PARAMS = {'work_angle_deg': 45.0, 'travel_angle_deg': 0.0, 'gap_mm': 1.0}
UP = np.array([0.0, 0.0, 1.0])
SIDE = np.array([1.0, 0.0, 0.0])


def line_seam(points, normals_main=None, normals_secondary=None):
    """Build a flat (edge-on-surface) seam over the given points."""
    points = np.asarray(points, dtype=float)
    n = len(points)
    seam = Seam(line_segment=LineSegment(start=points[0], end=points[-1]))
    seam.config.update({
        'smoothed_points': points,
        'normals_main': np.tile(UP, (n, 1)) if normals_main is None else normals_main,
        'normals_secondary': (
            np.tile(SIDE, (n, 1)) if normals_secondary is None else normals_secondary),
        'is_edge_joint': False,
        'geometry_type': 'line',
    })
    return seam


def along_y(n=11, length=0.1):
    y = np.linspace(0.0, length, n)
    return np.column_stack([np.zeros(n), y, np.zeros(n)])


class TestParameters:
    """Bad weld parameters are refused up front, by name, as ValueError."""

    @pytest.mark.parametrize('key,value', [
        ('gap_mm', float('nan')),
        ('gap_mm', 'wide'),
        ('gap_mm', -1.0),
        ('work_angle_deg', float('inf')),
        ('work_angle_deg', 90.0),
        ('work_angle_deg', -95.0),
        ('travel_angle_deg', 90.0),
        ('travel_angle_deg', 'steep'),
        ('waypoint_spacing_mm', 0.0),
    ])
    def test_a_bad_value_is_refused_by_name(self, key, value):
        with pytest.raises(ValueError, match=key):
            WeldPlanner(dict(PARAMS, **{key: value}))

    @pytest.mark.parametrize('key', sorted(PARAMS))
    def test_a_missing_required_key_is_refused_by_name(self, key):
        params = dict(PARAMS)
        del params[key]
        with pytest.raises(ValueError, match=key):
            WeldPlanner(params)

    def test_yaml_strings_of_numbers_are_accepted(self):
        planner = WeldPlanner(dict(PARAMS, gap_mm='2'))
        assert planner.gap_m == pytest.approx(0.002)


class TestDegenerateInput:
    """One bad sample must not cost the whole seam, nor turn the torch at random."""

    def test_a_point_with_no_base_normal_still_yields_finite_poses(self):
        points = along_y()
        normals = np.tile(UP, (len(points), 1))
        normals[5] = 0.0
        seam = line_seam(points, normals_main=normals)

        WeldPlanner(dict(PARAMS)).generate_seam(seam)

        assert seam.is_generated
        for pose in seam.poses:
            assert np.all(np.isfinite(pose['matrix']))

    def test_a_seam_with_no_usable_normal_at_all_is_refused(self):
        points = along_y()
        seam = line_seam(points, normals_main=np.zeros((len(points), 3)))
        with pytest.raises(ValueError, match='normal'):
            WeldPlanner(dict(PARAMS)).generate_seam(seam)

    def test_a_repeated_point_keeps_the_torch_along_the_seam(self):
        # A duplicate first sample gives a zero forward difference; the tangent there must still
        # follow the seam (+y), not an arbitrary world axis.
        points = along_y()
        points = np.vstack([points[:1], points])
        seam = line_seam(points)

        WeldPlanner(dict(PARAMS, waypoint_spacing_mm=1.0)).generate_seam(seam)

        first = np.array(seam.poses[0]['matrix'])
        np.testing.assert_allclose(first[:3, 0], [0.0, 1.0, 0.0], atol=1e-9)

    def test_a_seam_with_all_points_coincident_is_refused(self):
        points = np.zeros((5, 3))
        seam = line_seam(points)
        with pytest.raises(ValueError, match='tangent'):
            WeldPlanner(dict(PARAMS)).generate_seam(seam)
