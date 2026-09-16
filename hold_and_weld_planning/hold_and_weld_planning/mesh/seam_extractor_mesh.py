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

"""SeamExtractorMesh, extract weld seams where two meshes touch.

A weld seam is the relation "one part's boundary edge lies on the other part's surface": mark the
faces within `epsilon` of the other mesh, take the edges bounding that set, keep those that are a
real part edge, chain them across both meshes, then decide per POINT which mesh carries the edge
there. Ownership is per point because it alternates wherever a part overhangs.
"""

import logging

import manifold3d
import numpy as np
from numpy.typing import NDArray
import trimesh

from .chaining import drop_coincident, loops, oriented, stitch
from .mesh_fields import MeshFields, reject_holes
from .params import SeamExtractorMeshParams
from .path_creator import PathCreator
from .seam_point import SeamPoint
from ..core.seam import Seam

logger = logging.getLogger(__name__)

MAX_PART_SIZE_M = 50.0


class SeamExtractorMesh:
    """Extract weld seams from the contact region with per-point ownership."""

    def __init__(
        self,
        mesh_1: trimesh.Trimesh,
        mesh_2: trimesh.Trimesh,
        params: dict | None = None,
    ) -> None:
        """Initialize the contact boundary extractor.

        Both meshes are read through caches built on first use, so they must not be modified after
        construction; build a new extractor instead.

        Args:
            mesh_1: World-frame, watertight, in metres.
            mesh_2: World-frame, watertight, in metres.
            params: Optional config dict, per SeamExtractorMeshParams and PathCreatorParams in
                params.py. Each ignores the other's keys.

        Raises:
            TypeError: If either mesh is not a trimesh.Trimesh.
            ValueError: If either mesh is not watertight, has a non-finite vertex, or a parameter
                is out of range.
        """
        for name, mesh in (('mesh_1', mesh_1), ('mesh_2', mesh_2)):
            # trimesh.load hands back a Scene for a multi-body file.
            if not isinstance(mesh, trimesh.Trimesh):
                raise TypeError(
                    f'{name} must be a trimesh.Trimesh, got {type(mesh).__name__}')
            if not mesh.is_watertight:
                raise ValueError(f'{name} is not watertight')
            if not np.isfinite(mesh.vertices).all():
                raise ValueError(f'{name} has a non-finite vertex')

        # A mesh carries no unit, so millimetres can't be rejected outright, only flagged; left
        # alone they run silently at 1000x scale.
        size = max(float(mesh_1.extents.max()), float(mesh_2.extents.max()))
        if size > MAX_PART_SIZE_M:
            logger.warning(
                f'Largest part is {size:.1f}m across; the meshes are probably '
                'in millimetres, while epsilon and every length here is in '
                'metres'
            )

        self.cfg = SeamExtractorMeshParams.from_dict(params)
        self.path_creator = PathCreator(params)
        self.mesh = {1: mesh_1, 2: mesh_2}
        self.fields = MeshFields(self.mesh, self.cfg)

    def extract_seams(self) -> list[Seam]:
        """Extract chains and classify each into line/arc/PTP Seam objects.

        Returns:
            List of Seam objects. Empty when the parts yield no chain.
        """
        seams: list[Seam] = []
        for points, is_closed in self.extract_chains():
            seams.extend(self.path_creator.process_path(points, is_closed=is_closed))
        return seams

    def extract_chains(self) -> list[tuple[list[SeamPoint], bool]]:
        """Build ordered SeamPoint chains with per-point edge ownership.

        Follows the pipeline in the module docstring, then refines the points off the vertex
        lattice onto the contact boundary before reading ownership and normals at them.

        Returns:
            List of (seam_points, is_closed) pairs. Empty when the parts do not touch.
        """
        self._check_interpenetration()

        marked: dict[int, tuple[NDArray, list[tuple[int, int]]]] = {}
        touching = False
        for side in (1, 2):
            contact, boundary = self.fields.contact_boundary(side, self.cfg.epsilon)
            touching = touching or bool(boundary)
            self._validate_epsilon(side, self.cfg.epsilon, boundary)
            marked[side] = (contact, self._sharp_boundary(side, boundary))
            if boundary and not marked[side][1]:
                logger.info(
                    'mesh_%d bounds contact region without a part edge; supports joint.',
                    side,
                )

        if not touching:
            logger.info(
                'No contact within epsilon=%.2fmm; parts do not touch.',
                self.cfg.epsilon * 1000,
            )
            return []

        pieces: list[tuple[NDArray, bool]] = []
        for side in (1, 2):
            contact, sharp = marked[side]
            if not sharp:
                continue
            for loop, is_closed in loops(side, sharp, self.cfg):
                if is_closed:
                    wall = self.fields.fan_normal(side, loop[0], ~contact)
                    self._warn_zero_normals(
                        side, wall[None], 'wall',
                        'the loop keeps the direction it was found in')
                    loop = oriented(self.mesh[side], loop, contact, wall)
                pieces.append((self.mesh[side].vertices[loop], is_closed))

        if not pieces:
            logger.info('Contact found but neither part has a sharp edge on it.')
            return []

        pieces = drop_coincident(pieces, self.cfg)

        chains: list[tuple[list[SeamPoint], bool]] = []
        for positions, is_closed in stitch(pieces, self.cfg):
            positions = self._refine_positions(positions, marked, is_closed)
            if len(positions) < 2:
                continue
            chains.append((self._seam_points(positions, marked), is_closed))

        if logger.isEnabledFor(logging.INFO):
            total_points = sum(len(pts) for pts, _ in chains)
            m1_count = sum(p.owner_side == 1 for pts, _ in chains for p in pts)
            switches = sum(
                a.owner_side != b.owner_side
                for pts, _ in chains
                for a, b in zip(pts, pts[1:])
            )
            logger.info(
                '%d seam chain(s), %d point(s); ownership mesh_1=%d mesh_2=%d, %d switches',
                len(chains), total_points, m1_count, total_points - m1_count, switches
            )

        return chains

    def _check_interpenetration(self) -> None:
        """Warn when the parts overlap instead of meeting at a surface, see PARAMS.md.

        Watertight meshes can still self-intersect, so a failed boolean is reported and skipped
        rather than fatal.
        """
        try:
            volume = float((self._manifold(1) ^ self._manifold(2)).volume())
        except Exception as error:
            logger.warning(
                f'Could not compute interpenetration volume: {error}. '
                'Skipping the overlap check; the seam is still extracted.'
            )
            return

        if volume > self.cfg.interpenetration_volume_m3:
            logger.warning(
                f'Parts interpenetrate by {volume * 1e9:.1f} mm^3, past the '
                f'{self.cfg.interpenetration_volume_m3 * 1e9:.3f} mm^3 '
                'threshold. Any seam found below follows the BURIED RIM '
                'rather than the surface, so check the part poses before '
                'trusting it; shallow overlap still produces a plausible '
                'closed chain at the wrong depth.'
            )

    def _sharp_boundary(self, side: int, boundary: list[tuple[int, int]]) -> list[tuple[int, int]]:
        """Contact-boundary edges that follow a real part edge of this mesh.

        Neither mesh is "the" seam mesh: where a part terminates its boundary runs along its own
        sharp rim, where it merely supports the other the boundary is an imprint across coplanar
        faces, and on an overhanging joint each mesh does both over partof the curve. Sharpness
        alone isn't enough either, since a part's edges away from the joint are just as sharp, so a
        candidate must sit on the other mesh too.
        """
        table = self.fields.edge_dihedrals(side)
        sharp = [edge for edge in boundary if table.get(edge, 0.0) > self.cfg.edge_angle_min]
        if not sharp:
            return []

        # A sharp edge is only seam where the parts actually meet. The plate's outer rim is sharp
        # along its whole length and is not a weld.
        vertices = self.mesh[side].vertices
        index = np.asarray(sharp)
        midpoints = 0.5 * (vertices[index[:, 0]] + vertices[index[:, 1]])

        distance = self.fields.surface_distance(3 - side, midpoints)
        slack = self.cfg.near_contact_edge_fraction * self.fields.median_edge(side)
        bound = self.cfg.epsilon + slack
        keep = distance <= bound

        dropped = int((~keep).sum())
        if dropped:
            logger.info(
                f'mesh_{side}: dropped {dropped} of {len(sharp)} sharp '
                'boundary edge(s) that are real part edges away from the joint'
            )

        # Overhanging rims or near-boundary candidates sit close to 'bound' rather than zero
        if keep.any() and logger.isEnabledFor(logging.DEBUG):
            admitted = distance[keep]
            logger.debug(
                'mesh_%d: admitted %d/%d edge(s) at bound %.2fmm; max dist=%.3fmm',
                side, int(keep.sum()), len(sharp), bound * 1000, admitted.max() * 1000
            )
        return [edge for edge, ok in zip(sharp, keep) if ok]

    def _validate_epsilon(
        self,
        side: int,
        epsilon: float,
        reference: list[tuple[int, int]],
    ) -> None:
        """Report how far the contact boundary moves when epsilon is nudged.

        Measured rather than modelled: the boundary is recomputed at neighbouring epsilon values
        and its size compared per direction, since too small reads the parts as apart and too large
        climbs the wall. Compared as a relative change, not vertex-set equality, which a single
        moved vertex would trip on every valid run.

        Args:
            reference: The boundary at `epsilon`, already computed by the caller.
        """
        base = {v for edge in reference for v in edge}
        if not base:
            logger.warning(
                f'mesh_{side}: no contact boundary at epsilon='
                f'{epsilon * 1000:.2f}mm; too small for the fit-up gap, or the '
                'parts do not meet'
            )
            return

        counts = [f'{epsilon * 1000:.2f}mm -> {len(base)}']
        moved = {False: 0.0, True: 0.0}      # keyed by "probe was upward"
        for factor in self.cfg.eps_stability_factors:
            _, probe = self.fields.contact_boundary(side, epsilon * factor)
            vertices = {v for edge in probe for v in edge}
            change = abs(len(vertices) - len(base)) / len(base)
            counts.append(
                f'{epsilon * factor * 1000:.2f}mm -> {len(vertices)} '
                f'({change * 100:+.0f}%)'
            )
            moved[factor > 1.0] = max(moved[factor > 1.0], change)

        message = f'mesh_{side} boundary vertices: ' + ', '.join(counts)
        limit = self.cfg.eps_stability_tolerance
        unstable = {up for up, change in moved.items() if change >= limit}

        if not unstable:
            logger.debug(f'epsilon is stable within {limit * 100:.0f}% ({message})')
        elif len(unstable) == 1:
            edge = 'upper' if unstable.pop() else 'lower'
            logger.debug(
                f'epsilon={epsilon * 1000:.2f}mm sits near the {edge} end of '
                f'its band ({message}); stable the other way, so this is '
                'headroom rather than a fault'
            )
        else:
            logger.warning(
                f'epsilon={epsilon * 1000:.2f}mm has no stable band; the '
                f'contact boundary moves either way it is nudged ({message}). '
                'Too small and the parts read as apart; too large and the '
                'boundary climbs the wall instead of following the joint.'
            )

    def _refine_positions(
        self,
        positions: NDArray,
        marked: dict[int, tuple[NDArray, list[tuple[int, int]]]],
        is_closed: bool = False,
    ) -> NDArray:
        """Move seam points off the vertex lattice onto the true contact boundary.

        Every seam point is an existing mesh vertex, so the seam is only as fine as the
        tessellation, and where a part edge is much longer than the contact region crossing it, the
        true boundary falls between vertices.
        """
        owner, _ = self._owner(positions)
        rho = np.zeros(len(positions))
        mating = np.zeros((len(positions), 3))
        for side in (1, 2):
            rows = np.nonzero(owner == side)[0]
            if not len(rows):
                continue
            other_side = 3 - side
            _, nearest = self.fields.vertex_tree(side).query(positions[rows])
            _, across = self.fields.vertex_tree(other_side).query(positions[rows])

            # rho must span the mesh being INTEGRATED (the OTHER one), not just the owner's: too
            # small relative to ITS triangle size and the centroid-sampled integral reads 0 or
            # overshoots on luck.
            rho[rows] = self.cfg.kernel_radius_factor * np.maximum(
                self.fields.vertex_edge_scale(side)[nearest],
                self.fields.vertex_edge_scale(other_side)[across],
            )
            contact = marked[side][0]
            mating[rows] = [self.fields.fan_normal(side, int(v), contact) for v in nearest]
            self._warn_zero_normals(
                side, mating[rows], 'contact',
                'coverage reads zero there, so those points are dropped')

        coverage = np.array([
            self.fields.coverage(int(owner[i]), positions[i], rho[i], mating[i])
            for i in range(len(positions))
        ])

        # A point exactly on the boundary reads 1/2 by construction; either side is fine, the slide
        # below lands on the same level set.
        inside = coverage >= 0.5

        if int(inside.sum()) < self.cfg.min_loop_points:
            logger.warning(
                f'Contact coverage puts only {int(inside.sum())} of '
                f'{len(positions)} seam point(s) inside the contact; leaving '
                'the chain on its mesh vertices. The mesh is probably too '
                'coarse at the joint for the half level set to exist.'
            )
            return positions

        inside = reject_holes(inside, positions, rho, is_closed)

        keep = np.nonzero(inside)[0]
        refined = positions[keep].copy()

        runs: list[tuple[int, int]] = []
        start = 0
        for j in range(1, len(keep) + 1):
            if j == len(keep) or keep[j] != keep[j - 1] + 1:
                runs.append((start, j - 1))
                start = j

        wraps = (is_closed and len(keep) > 0 and keep[0] == 0 and keep[-1] == len(positions) - 1)
        skip = {runs[0][0], runs[-1][1]} if wraps and runs else set()

        moved = 0
        for first, last in runs:
            if last <= first:
                continue
            for outer, inner in ((first, first + 1), (last, last - 1)):
                if outer in skip:
                    continue
                direction = refined[outer] - refined[inner]
                length = float(np.linalg.norm(direction))
                if length < 1e-12:
                    continue
                landed = self.fields.slide_to_boundary(
                    refined[outer], direction / length,
                    int(owner[keep[outer]]), rho[keep[outer]],
                    mating[keep[outer]],
                )
                if landed is not None:
                    moved += 1
                    refined[outer] = landed

        dropped = len(positions) - len(keep)
        logger.info(
            f'Refined {len(positions)} seam point(s) against the contact half '
            f'level set: dropped {dropped} lying past the boundary, slid '
            f'{moved} end point(s) onto it'
        )
        return refined

    def _seam_points(
        self,
        positions: NDArray,
        marked: dict[int, tuple[NDArray, list[tuple[int, int]]]],
    ) -> list[SeamPoint]:
        """Attach per-point ownership and normals to an ordered polyline."""
        owner, _ = self._owner(positions)
        on_edge = self._on_edge_mask(owner, positions)

        normal_base = np.zeros((len(positions), 3))
        normal_wall = np.zeros((len(positions), 3))
        for side in (1, 2):
            rows = np.nonzero(owner == side)[0]
            if not len(rows):
                continue
            other_mesh = self.mesh[3 - side]
            base = self.fields.closest_faces(3 - side, positions[rows])
            normal_base[rows] = other_mesh.face_normals[base]

            contact = marked[side][0]
            _, nearest = self.fields.vertex_tree(side).query(positions[rows])
            for idx, v in zip(rows, nearest):
                normal_wall[idx] = self.fields.fan_normal(side, int(v), ~contact)
            self._warn_zero_normals(
                side, normal_wall[rows], 'wall',
                'the weld pose there will be unusable'
            )

        return [
            SeamPoint(
                position=positions[i],
                normal_base=normal_base[i],
                normal_wall=normal_wall[i],
                on_edge_1=bool(on_edge[1][i]),
                on_edge_2=bool(on_edge[2][i]),
                owner_side=int(owner[i]),
            )
            for i in range(len(positions))
        ]

    def _warn_zero_normals(self, side: int, normals: NDArray, kind: str, consequence: str) -> None:
        """Warn once per batch for points fan_normal left with no normal."""
        zero = int((~normals.any(axis=1)).sum())
        if zero:
            logger.warning(
                'mesh_%d: %d of %d point(s) have no usable %s normal; %s',
                side, zero, len(normals), kind, consequence
            )

    def _owner(self, points: NDArray) -> tuple[NDArray, NDArray]:
        """Determine which mesh carries the geometric edge at each point."""
        points = np.atleast_2d(points)
        distance = {side: self.fields.sharp_edge_distance(side, points) for side in (1, 2)}
        owner = np.where(distance[1] <= distance[2], 1, 2)

        # Absolute gap scaled to the mesh: these distances are tiny refinement residuals, so a
        # relative tolerance only ties exact zeros.
        tolerance = self.cfg.ownership_tie_factor * max(
            self.fields.median_edge(1), self.fields.median_edge(2)
        )
        tied = (
            (np.abs(distance[1] - distance[2]) <= tolerance)
            & np.isfinite(distance[1]) & np.isfinite(distance[2])
        )
        if tied.any():
            owner[tied] = self._owner_by_turning(points[tied])

        won = np.where(owner == 1, distance[1], distance[2])
        return owner, won

    def _owner_by_turning(self, points: NDArray) -> NDArray:
        """Break a sharp-edge distance tie based on local surface curvature density."""
        radius = self._ownership_radius()
        density = {side: self.fields.turning_density(side, points, radius) for side in (1, 2)}

        flat = (density[1] <= 0.0) & (density[2] <= 0.0)
        if flat.any():
            logger.warning(
                'ownership: %d of %d tied point(s) read zero turning on BOTH meshes at '
                'radius %.6fm',
                int(flat.sum()), len(points), radius
            )

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                'ownership: broke %d tie(s) on turning density at radius %.6fm (m1 '
                'median=%.4f/m, m2 median=%.4f/m)',
                len(points), radius, np.median(density[1]), np.median(density[2])
            )

        return np.where(density[1] >= density[2], 1, 2)

    def _on_edge_mask(self, owner: NDArray, points: NDArray) -> dict[int, NDArray]:
        """Compute per-mesh, per-point edge-joint presence flags."""
        distance = {side: self.fields.sharp_edge_distance(side, points) for side in (1, 2)}

        if logger.isEnabledFor(logging.DEBUG):
            loser = np.where(owner == 1, distance[2], distance[1])
            if len(loser):
                logger.debug(
                    'is_edge_joint: loser distance over %d point(s), min=%.3fmm max=%.3fmm',
                    len(loser), loser.min() * 1000, loser.max() * 1000
                )

        return {
            side: (
                distance[side]
                <= self.cfg.edge_joint_floor_factor * self.fields.median_edge(side)
            ) | (owner == side)
            for side in (1, 2)
        }

    def _manifold(self, side: int) -> manifold3d.Manifold:
        """Convert one mesh to a manifold3d solid for CSG boolean operations."""
        mesh = self.mesh[side]
        manifold = manifold3d.Manifold(manifold3d.Mesh(
            vert_properties=np.asarray(mesh.vertices, dtype=np.float32),
            tri_verts=np.asarray(mesh.faces, dtype=np.uint32),
        ))

        status = manifold.status()
        if status != manifold3d.Error.NoError:
            raise ValueError(f'mesh_{side} failed manifold conversion: {status}')
        return manifold

    def _ownership_radius(self) -> float:
        """Neighborhood radius for the turning-density comparison."""
        return self.cfg.ownership_radius_factor * max(
            self.fields.median_edge(1), self.fields.median_edge(2)
        )
