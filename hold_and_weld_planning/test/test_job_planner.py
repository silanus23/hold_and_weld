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

"""Unit tests for JobPlanner mode resolution and parameter handling."""

from hold_and_weld_planning.planning.job_planner import JobPlanner

import pytest

REQUIRED = {'work_angle_deg': 45.0, 'travel_angle_deg': 0.0, 'gap_mm': 0.0}


class TestParameterIsolation:
    """The caller's dict must survive being handed to a planner."""

    def test_construction_does_not_mutate_the_callers_dict(self):
        parameters = dict(REQUIRED)
        JobPlanner('a.step', 'b.step', parameters=parameters)
        assert parameters == REQUIRED, 'defaults leaked into the caller'

    def test_a_second_planner_still_gets_its_own_default_epsilon(self):
        # The bug: the first planner wrote mesh's 0.002 into the shared dict,
        # so the OCCT planner built from the same dict inherited it instead of
        # its own 1e-3 - a 2x tolerance, silently.
        parameters = dict(REQUIRED)

        mesh = JobPlanner('a.stl', 'b.stl', parameters=parameters, mode='mesh')
        occt = JobPlanner('a.step', 'b.step', parameters=parameters, mode='occt')

        assert mesh.parameters['epsilon'] == 0.002
        assert occt.parameters['epsilon'] == 1e-3

    def test_an_explicit_epsilon_is_never_overridden(self):
        parameters = dict(REQUIRED, epsilon=0.05)
        planner = JobPlanner('a.stl', 'b.stl', parameters=parameters)
        assert planner.parameters['epsilon'] == 0.05


class TestModeDetection:
    """Both inputs must belong to the pipeline that gets chosen."""

    @pytest.mark.parametrize('main,secondary,expected', [
        ('a.step', 'b.step', 'occt'),
        ('a.stp', 'b.urdf', 'occt'),
        ('a.stl', 'b.stl', 'mesh'),
        ('a.urdf', 'b.xacro', 'mesh'),
    ])
    def test_auto_detects_the_right_pipeline(self, main, secondary, expected):
        planner = JobPlanner(main, secondary, parameters=dict(REQUIRED))
        assert planner.mode == expected

    def test_a_mesh_input_paired_with_cad_is_refused_by_name(self):
        # Detection picks OCCT off the STEP; the STL would then reach the URDF
        # branch of the loader and fail inside xacro as malformed XML.
        with pytest.raises(ValueError, match=r'b\.stl'):
            JobPlanner('a.step', 'b.stl', parameters=dict(REQUIRED))

    def test_an_explicit_mode_is_validated_too(self):
        with pytest.raises(ValueError, match=r'a\.step'):
            JobPlanner('a.step', 'b.stl', parameters=dict(REQUIRED),
                       mode='mesh')

    def test_an_unknown_extension_is_refused(self):
        with pytest.raises(ValueError):
            JobPlanner('a.obj', 'b.obj', parameters=dict(REQUIRED))
