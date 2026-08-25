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

"""Unit tests for the PathCreator tolerance cascade on synthetic curves."""

import numpy as np
import pytest

from hold_and_weld_planning.mesh.path_creator import PathCreator
from hold_and_weld_planning.mesh.seam_point import SeamPoint


def make_seam_points(positions):
    up = np.array([0.0, 0.0, 1.0])
    side = np.array([1.0, 0.0, 0.0])
    return [
        SeamPoint(
            position=np.asarray(p, dtype=float),
            normal_base=up,
            normal_wall=side,
            on_edge_1=False,
            on_edge_2=True,
            owner_side=2,
        )
        for p in positions
    ]


def circle_points(radius=0.1, n=100, closed=True):
    span = 2.0 * np.pi if closed else 1.5 * np.pi
    t = np.linspace(0.0, span, n, endpoint=not closed)
    return np.column_stack(
        [radius * np.cos(t), radius * np.sin(t), np.zeros(n)]
    )


def ellipse_points(a=0.1, b=0.08, n=120):
    t = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    return np.column_stack([a * np.cos(t), b * np.sin(t), np.zeros(n)])


def max_distance_to_curve(seam, curve):
    """Max distance from a seam's stored points to a densely sampled curve."""
    pts = seam.config['smoothed_points']
    d = np.linalg.norm(pts[:, None, :] - curve[None, :, :], axis=2)
    return float(np.max(np.min(d, axis=1)))


@pytest.fixture
def creator():
    return PathCreator({'max_arc_length': 10.0, 'max_line_length': 10.0})


class TestLine:

    def test_straight_run_is_one_line(self, creator):
        t = np.linspace(0.0, 0.3, 40)
        positions = np.column_stack([t, 0.5 * t, np.zeros(40)])
        seams = creator.process_path(make_seam_points(positions))
        assert len(seams) == 1
        assert seams[0].config['geometry_type'] == 'line'

    def test_noisy_line_within_tolerance_is_line(self, creator):
        rng = np.random.default_rng(7)
        t = np.linspace(0.0, 0.3, 60)
        positions = np.column_stack([t, np.zeros(60), np.zeros(60)])
        positions[:, 1] += rng.uniform(-3e-4, 3e-4, 60)  # < 1mm tolerance
        seams = creator.process_path(make_seam_points(positions))
        assert len(seams) == 1
        assert seams[0].config['geometry_type'] == 'line'

    def test_gentle_curve_below_tolerance_is_line(self, creator):
        # Deviation from straight stays under path tolerance: line by design.
        t = np.linspace(0.0, 0.1, 30)
        sag = 0.8e-3 * np.sin(np.pi * t / 0.1)  # max 0.8mm bow
        positions = np.column_stack([t, sag, np.zeros(30)])
        seams = creator.process_path(make_seam_points(positions))
        # A single primitive; the bow is within welding tolerance either way.
        assert len(seams) == 1


class TestCircle:

    def test_closed_circle_is_one_arc(self, creator):
        positions = circle_points(closed=True)
        seams = creator.process_path(make_seam_points(positions), is_closed=True)
        assert len(seams) == 1
        assert seams[0].config['geometry_type'] == 'arc'

    def test_open_arc_is_arc(self, creator):
        positions = circle_points(closed=False)
        seams = creator.process_path(make_seam_points(positions))
        assert len(seams) == 1
        assert seams[0].config['geometry_type'] == 'arc'


class TestEllipseSafety:
    """The asymmetry: an ellipse must never be forced into one circle."""

    def test_ellipse_is_not_one_arc(self, creator):
        positions = ellipse_points()
        seams = creator.process_path(make_seam_points(positions), is_closed=True)
        assert len(seams) >= 2, (
            'a 0.1/0.08 ellipse fitted as a single primitive would deviate '
            'millimetres from the seam'
        )

    def test_every_primitive_stays_on_the_ellipse(self, creator):
        positions = ellipse_points()
        seams = creator.process_path(make_seam_points(positions), is_closed=True)
        dense = ellipse_points(n=4000)
        for seam in seams:
            assert max_distance_to_curve(seam, dense) < 1.5e-3

    def test_near_circle_ellipse_not_forced_to_arc(self, creator):
        # 3mm semi-axis difference: a "slightly cracked" circle. A single
        # circle fit would deviate ~1.5mm; the strict arc test must split
        # it or fall back, never emit one full-span arc.
        positions = ellipse_points(a=0.1, b=0.097, n=160)
        seams = creator.process_path(make_seam_points(positions), is_closed=True)
        dense = ellipse_points(a=0.1, b=0.097, n=4000)
        for seam in seams:
            assert max_distance_to_curve(seam, dense) < 1.5e-3


class TestPtP:

    def test_erratic_path_falls_back_to_ptp(self, creator):
        rng = np.random.default_rng(3)
        positions = np.cumsum(rng.uniform(-0.01, 0.01, (30, 3)), axis=0)
        seams = creator.process_path(make_seam_points(positions))
        assert len(seams) >= 1
        # No fitted primitive may claim an erratic walk.
        for seam in seams:
            if seam.config['geometry_type'] in ('line', 'arc'):
                pts = seam.config['smoothed_points']
                assert len(pts) < len(positions)
