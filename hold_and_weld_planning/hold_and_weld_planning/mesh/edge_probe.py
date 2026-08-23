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

"""Local geometric edge probe based on area-weighted normal statistics.

The probe answers, for a 3D point and one mesh: "is there a geometric edge
here, how sharp is it, and which way does it run?" It distinguishes a true
edge (a normal *jump*: most of the local normal variation concentrated in one
large step between two clusters) from smooth curvature (a normal *smear*:
variation distributed over many small steps), using two dimensionless,
insensitive criteria:

    is_edge  <=>  gap_ratio > gap_dominance  AND  dihedral >= edge_angle_min

where gap_ratio is the largest single angular step between projection-sorted
local normals divided by the total angular spread, and dihedral is the angle
between the two area-weighted cluster mean normals on either side of that
largest step.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import KDTree
import trimesh

DEFAULT_EDGE_ANGLE_MIN_DEG = 15.0
DEFAULT_GAP_DOMINANCE = 0.6

# Probe radius as a multiple of the local median edge length. Must span
# several faces so a single tessellation step on a coarse smooth surface
# cannot dominate the gap statistic.
PROBE_RADIUS_EDGE_FACTOR = 4.0

_MIN_FACES = 4
_FLAT_SPREAD = 1e-8


def _kernel(t: NDArray) -> NDArray:
    """Smooth compactly-supported radial weight, (1 - t^2)^3 on t in [0, 1].

    Both the value and its derivative vanish at t = 1, so a face entering the
    probe ball does so with zero weight *and* zero rate. That makes every
    probe statistic a smooth function of the probe centre rather than a
    staircase that only changes when a centroid crosses the radius, which is
    what allows the ridge walk to converge to a sub-triangle position.
    """
    u = np.clip(1.0 - t * t, 0.0, None)
    return u * u * u


@dataclass
class ProbeResult:
    """Result of probing one point against one mesh.

    Attributes:
        is_edge:      True if the local normal field shows a dominant jump
                      of at least the minimum dihedral angle.
        dihedral:     Angle (rad) between the two cluster mean normals.
        spread:       Total angular spread (rad): sum of consecutive angular
                      steps of the projection-sorted local normals.
        gap_ratio:    Largest single step divided by spread, in [0, 1].
        balance:      Area fraction of the dominant cluster, in [0.5, 1].
        response:     Walk objective, dihedral * 4 * w * (1 - w); peaks when
                      the probe ball straddles the edge evenly.
        n_dominant:   Area-weighted mean normal of the dominant cluster.
        n_secondary:  Area-weighted mean normal of the other cluster (equals
                      n_dominant when no split exists).
        edge_dir:     Unit direction of the edge line (n_dominant x
                      n_secondary), or None when there is no edge.
        num_faces:    Number of faces that contributed to the statistics.
    """

    is_edge: bool = False
    dihedral: float = 0.0
    spread: float = 0.0
    gap_ratio: float = 0.0
    balance: float = 1.0
    response: float = 0.0
    n_dominant: NDArray = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))
    n_secondary: NDArray = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))
    edge_dir: Optional[NDArray] = None
    num_faces: int = 0


class ProbeMesh:
    """Precomputed per-mesh lookup structures for the edge probe.

    Args:
        mesh: Watertight trimesh in world frame.
    """

    def __init__(self, mesh: trimesh.Trimesh) -> None:
        self.mesh = mesh
        self.face_normals = np.asarray(mesh.face_normals, dtype=np.float64)
        self.face_areas = np.asarray(mesh.area_faces, dtype=np.float64)
        self.face_centroids = np.asarray(mesh.triangles_center, dtype=np.float64)
        self.centroid_tree = KDTree(self.face_centroids)

        self.adjacency: Dict[int, List[int]] = {}
        for f1, f2 in mesh.face_adjacency:
            self.adjacency.setdefault(int(f1), []).append(int(f2))
            self.adjacency.setdefault(int(f2), []).append(int(f1))

        self.median_edge = float(np.median(mesh.edges_unique_length))
        self.probe_radius = PROBE_RADIUS_EDGE_FACTOR * self.median_edge


class EdgeProbe:
    """Probe 3D points against meshes for local geometric edges.

    Args:
        params: Configuration dict; reads 'edge_angle_min_deg' and
                'gap_dominance', both optional.
    """

    def __init__(self, params: Optional[Dict] = None) -> None:
        params = params or {}
        self.edge_angle_min = np.radians(
            params.get('edge_angle_min_deg', DEFAULT_EDGE_ANGLE_MIN_DEG)
        )
        self.gap_dominance = params.get('gap_dominance', DEFAULT_GAP_DOMINANCE)

    def collect_faces(
        self,
        probe_mesh: ProbeMesh,
        point: NDArray,
        radius: Optional[float] = None,
    ) -> List[int]:
        """Flood-fill face indices whose centroids lie within radius of point.

        Seeded at the face nearest to the point and grown over mesh adjacency,
        so the patch never jumps across free space to an unconnected part of
        the mesh (e.g. the far side of a thin plate).
        """
        if radius is None:
            radius = probe_mesh.probe_radius

        _, seed = probe_mesh.centroid_tree.query(point)
        seed = int(seed)

        visited = {seed}
        frontier = [seed]
        centroids = probe_mesh.face_centroids

        while frontier:
            f = frontier.pop()
            for nb in probe_mesh.adjacency.get(f, []):
                if nb in visited:
                    continue
                if np.linalg.norm(centroids[nb] - point) <= radius:
                    visited.add(nb)
                    frontier.append(nb)

        return sorted(visited)

    def probe(
        self,
        probe_mesh: ProbeMesh,
        point: NDArray,
        radius: Optional[float] = None,
    ) -> ProbeResult:
        """Probe one point against one mesh; see ProbeResult for semantics."""
        if radius is None:
            radius = probe_mesh.probe_radius

        faces = self.collect_faces(probe_mesh, point, radius)

        normals = probe_mesh.face_normals[faces]
        # Area weighted by a smooth radial kernel: hard in/out membership makes
        # every statistic a step function of the probe centre, which pins the
        # walk to the triangle lattice. See _kernel.
        distances = np.linalg.norm(
            probe_mesh.face_centroids[faces] - point, axis=1
        ) if len(faces) else np.zeros(0)
        areas = probe_mesh.face_areas[faces] * _kernel(distances / radius)
        total_area = float(np.sum(areas))

        if len(faces) < _MIN_FACES or total_area <= 0.0:
            # Too few faces to classify, but still report the best available
            # local normal so callers get a usable surface direction.
            n = _unit(np.sum(areas[:, None] * normals, axis=0)) if len(faces) \
                else np.array([0.0, 0.0, 1.0])
            return ProbeResult(n_dominant=n, n_secondary=n, num_faces=len(faces))

        weights = areas / total_area
        mean = weights @ normals
        centered = normals - mean
        cov = (weights[:, None] * centered).T @ centered

        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        principal = eigenvectors[:, -1]

        order = np.argsort(normals @ principal)
        sorted_normals = normals[order]
        sorted_areas = areas[order]

        dots = np.clip(
            np.einsum('ij,ij->i', sorted_normals[:-1], sorted_normals[1:]),
            -1.0, 1.0,
        )
        steps = np.arccos(dots)
        spread = float(np.sum(steps))

        if spread < _FLAT_SPREAD:
            # Flat neighbourhood: nothing to classify.
            n = _unit(weights @ normals)
            return ProbeResult(
                spread=spread, n_dominant=n, n_secondary=n, num_faces=len(faces)
            )

        split = int(np.argmax(steps))
        gap_ratio = float(steps[split] / spread)

        area_a = float(np.sum(sorted_areas[: split + 1]))
        area_b = float(np.sum(sorted_areas[split + 1:]))
        mean_a = _unit(sorted_areas[: split + 1] @ sorted_normals[: split + 1])
        mean_b = _unit(sorted_areas[split + 1:] @ sorted_normals[split + 1:])

        if area_a >= area_b:
            n_dominant, n_secondary = mean_a, mean_b
            balance = area_a / (area_a + area_b)
        else:
            n_dominant, n_secondary = mean_b, mean_a
            balance = area_b / (area_a + area_b)

        dihedral = float(
            np.arccos(np.clip(np.dot(n_dominant, n_secondary), -1.0, 1.0))
        )

        is_edge = bool(
            gap_ratio > self.gap_dominance and dihedral >= self.edge_angle_min
        )

        response = dihedral * 4.0 * balance * (1.0 - balance)

        edge_dir = None
        if is_edge:
            cross = np.cross(n_dominant, n_secondary)
            norm = np.linalg.norm(cross)
            if norm > 1e-10:
                edge_dir = cross / norm
            else:
                is_edge = False

        return ProbeResult(
            is_edge=is_edge,
            dihedral=dihedral,
            spread=spread,
            gap_ratio=gap_ratio,
            balance=balance,
            response=response,
            n_dominant=n_dominant,
            n_secondary=n_secondary,
            edge_dir=edge_dir,
            num_faces=len(faces),
        )


def _unit(v: NDArray) -> NDArray:
    """Return v normalized to unit length (unchanged if near-zero)."""
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v
