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

"""Tuning parameters for the mesh pipeline, as validated dataclasses.

One place to read every knob the mesh extractor and path classifier take. The
declaration carries the name, type and default; `__post_init__` carries the
constraints; PARAMS.md carries the measurements behind the defaults.

Lives in its own module because `seam_extractor_mesh` imports `path_creator`, so
anything both of them share has to sit upstream of the pair.
"""

from dataclasses import dataclass, fields
from typing import Any, Dict, Optional, Tuple

import numpy as np


class ParamsBase:
    """Build a params dataclass from a loosely typed config dict."""

    @classmethod
    def from_dict(cls, params: Optional[Dict[str, Any]]):
        """Build from a config dict, coercing types and ignoring foreign keys.

        Foreign keys are ignored rather than rejected because the pipeline
        hands ONE dict to both the extractor and PathCreator, so each
        necessarily sees the other's keys. Values arrive from YAML, so the
        declared field type does the coercion.

        Args:
            params: Config dict, or None for all defaults.

        Returns:
            An instance with declared defaults for everything not supplied.

        Raises:
            ValueError: If a value cannot be coerced, or fails a constraint.
        """
        given = params or {}
        taken = {}
        for spec in fields(cls):
            if spec.name not in given:
                continue
            taken[spec.name] = cls._coerce(spec.name, spec.type,
                                           given[spec.name])
        return cls(**taken)

    @staticmethod
    def _coerce(name: str, declared: Any, value: Any) -> Any:
        """Coerce one config value to its declared field type."""
        try:
            if declared is int or declared == 'int':
                return int(value)
            return float(value)
        except (TypeError, ValueError):
            raise ValueError(
                f'{name} must be a number, got {value!r}')


@dataclass
class SeamExtractorMeshParams(ParamsBase):
    """Tuning for SeamExtractorMesh; every field is optional."""

    # Contact region
    epsilon: float = 0.002                  # m, between fit-up gap and half
    #                                         the transverse face size
    edge_angle_min_deg: float = 0.0057      # dihedral above which an edge is
    #                                         a real part edge
    near_contact_edge_fraction: float = 0.1  # slack on epsilon, as a fraction
    #                                          of this mesh's median edge

    # Epsilon plateau check
    eps_stability_factors: Tuple[float, ...] = (0.75, 1.5)   # probe factors
    eps_stability_tolerance: float = 0.25   # relative boundary movement
    #                                         tolerated before "unstable"

    # Chaining and stitching
    min_loop_points: int = 4                # shortest chain kept
    stitch_gap_factor: float = 3.0          # gap allowed when joining two
    #                                         pieces, x local sampling density

    # Refinement onto the contact boundary
    kernel_radius_factor: float = 1.0       # coverage radius, x local edge
    coverage_bisection_steps: int = 20      # steps solving the half level set

    # Ownership
    ownership_radius_factor: float = 2.0    # turning radius, x coarser median
    ownership_tie_factor: float = 0.001     # x coarser median edge, the gap
    #                                         below which two sharp-edge
    #                                         distances count as tied
    edge_joint_floor_factor: float = 0.001  # x median edge, below which the
    #                                         losing mesh is also on the edge

    # Nearest-feature search widths, in candidates probed per point
    closest_face_candidates: int = 12
    closest_vertex_candidates: int = 4
    sharp_edge_candidates: int = 32

    # Overlap volume above which the parts are reported as interpenetrating
    # rather than touching. A WARNING, not a refusal - see
    # SeamExtractorMesh._report_interpenetration.
    interpenetration_volume_m3: float = 1e-12

    @property
    def edge_angle_min(self) -> float:
        """`edge_angle_min_deg` in radians, as the dihedral tables carry it."""
        return float(np.radians(self.edge_angle_min_deg))

    @staticmethod
    def _coerce(name: str, declared: Any, value: Any) -> Any:
        """Coerce, with the probe-factor sequence as the one special case."""
        if name == 'eps_stability_factors':
            try:
                return tuple(float(f) for f in value)
            except (TypeError, ValueError):
                raise ValueError(
                    'eps_stability_factors must be a list of probe factors, '
                    f'e.g. [0.75, 1.5]; got {value!r}'
                )
        return ParamsBase._coerce(name, declared, value)

    def __post_init__(self) -> None:
        """Reject values that fail late, or silently reassure.

        Raises:
            ValueError: If a parameter is out of range.
        """
        for key in (
            'epsilon', 'closest_face_candidates', 'closest_vertex_candidates',
            'kernel_radius_factor', 'edge_joint_floor_factor',
            'ownership_radius_factor', 'ownership_tie_factor',
        ):
            if not getattr(self, key) > 0:
                raise ValueError(f'{key} must be > 0, got {getattr(self, key)}')

        # Zero is a legitimate way to disable each of these, negative is not.
        # eps_stability_tolerance at 0 warns on any movement at all, which is
        # the behaviour this replaced.
        for key in (
            'edge_angle_min_deg', 'near_contact_edge_fraction',
            'stitch_gap_factor', 'interpenetration_volume_m3',
            'eps_stability_tolerance',
        ):
            if getattr(self, key) < 0.0:
                raise ValueError(
                    f'{key} must be >= 0, got {getattr(self, key)}')

        # One step halves the bracket; below a handful the level set is no
        # better located than the vertices it exists to escape.
        if self.coverage_bisection_steps < 4:
            raise ValueError(
                'coverage_bisection_steps must be >= 4, got '
                f'{self.coverage_bisection_steps}')

        # `oriented` reads loop[1] and loop[-1]; below 3 there is no loop.
        if self.min_loop_points < 3:
            raise ValueError(
                f'min_loop_points must be >= 3, got {self.min_loop_points}')

        # The nearest sharp edge is found among the nearest edge MIDPOINTS, so
        # one candidate would trust the midpoint ordering completely.
        if self.sharp_edge_candidates < 2:
            raise ValueError(
                'sharp_edge_candidates must be >= 2, got '
                f'{self.sharp_edge_candidates}')

        if not self.eps_stability_factors:
            raise ValueError(
                'eps_stability_factors must name at least one probe factor; '
                'with none, the plateau check has nothing to compare against '
                'and reports every epsilon as stable'
            )
        for factor in self.eps_stability_factors:
            if not factor > 0.0:
                raise ValueError(
                    f'eps_stability_factors must all be > 0, got {factor}')
            if factor == 1.0:
                raise ValueError(
                    'eps_stability_factors of exactly 1.0 probes epsilon '
                    'against itself and always reports a stable plateau'
                )


@dataclass
class PathCreatorParams(ParamsBase):
    """Tuning for PathCreator; every field is optional.

    Fields are the config keys as written in YAML, in the units they are
    written in. The derived quantities the cascade actually compares against -
    metres, radians, the arc tolerance - are the properties below, so a key
    and its unit never drift apart.
    """

    # Tolerance cascade
    path_tolerance_mm: float = 1.0     # deviation a LINE may hold
    arc_strictness: float = 0.5        # x path_tolerance_mm for an ARC;
    #                                    0 disables arcs
    min_arc_angle_deg: float = 15.0    # angle an arc must subtend to be worth
    #                                    calling an arc
    arc_gain: float = 1.5              # how much longer an arc run must be
    #                                    than the line run it displaces
    min_fit_points: int = 4            # points a fit needs

    # Segment length caps, in metres; hard splits, not geometric
    max_line_length: float = 0.5
    max_arc_length: float = 0.5
    max_ptp_length: float = 0.1

    # Gap `_join_consecutive` may close silently, as a multiple of the seam's
    # own sample spacing. Past it the join is recorded as a bridged hole.
    max_bridge_factor: float = 4.0

    # Shared with WeldPlanner: the process spacing, which doubles as the
    # shortest contact run kept when splitting on joint character. Not its own
    # key because a run too short to carry two weld poses is not a segment.
    waypoint_spacing_mm: float = 10.0

    @property
    def tolerance(self) -> float:
        """`path_tolerance_mm` in metres."""
        return self.path_tolerance_mm * 1e-3

    @property
    def arc_tolerance(self) -> float:
        """Return the stricter tolerance an arc fit must hold, in metres."""
        return self.arc_strictness * self.tolerance

    @property
    def min_arc_angle(self) -> float:
        """`min_arc_angle_deg` in radians."""
        return float(np.radians(self.min_arc_angle_deg))

    @property
    def min_contact_run(self) -> float:
        """Return the shortest contact run kept, in metres."""
        return self.waypoint_spacing_mm * 1e-3

    def __post_init__(self) -> None:
        """Reject config that silently collapses the cascade or divides by zero.

        Raises:
            ValueError: If a parameter is out of range.
        """
        for key in ('path_tolerance_mm', 'max_line_length', 'max_arc_length',
                    'max_ptp_length', 'arc_gain', 'waypoint_spacing_mm',
                    'max_bridge_factor'):
            if not getattr(self, key) > 0.0:
                raise ValueError(f'{key} must be > 0, got {getattr(self, key)}')

        # Zero is a legitimate way to disable each of these, negative is not.
        for key in ('arc_strictness', 'min_arc_angle_deg'):
            if getattr(self, key) < 0.0:
                raise ValueError(
                    f'{key} must be >= 0, got {getattr(self, key)}')

        if self.min_fit_points < 3:
            raise ValueError(
                f'min_fit_points must be >= 3, got {self.min_fit_points}')
