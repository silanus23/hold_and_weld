# Copyright 2026 Berkan Tali
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""SeamExtractorOCCT - Extract weld seams from OCCT shapes using exact geometry.

This module uses face-to-face proximity detection and BRepAlgoAPI_Common to find
kissing surfaces, then extracts exact seam curves and surface normals.
"""

# TODO(silanus23): complete pipe joint detection. The commented-out methods after
# _get_wall_surface_at_edge treat any face with inner holes (more than one wire) as a pipe joint
# and take normals from the outer shaft surfaces, found by a G1 continuity check. Untested on
# non-cylindrical shafts, several discontinuous faces, and stepped or flanged pipes. Before
# re-enabling them:
# - shaft selection takes the largest discontinuous face, an area heuristic
# - the hard-coded fallback normal [0, 0, 1] should fail instead
# - the G1 continuity check samples one point on the edge
# - they use `warnings` and `TopAbs_WIRE`, neither of which is imported

# TODO(silanus23): add a secondary check on whether a PTP seam could be an arc or a line.
from dataclasses import dataclass
import logging

import numpy as np
from numpy.typing import NDArray

from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.GeomAbs import GeomAbs_Circle, GeomAbs_Line, GeomAbs_Plane
from OCC.Core.GeomLProp import GeomLProp_SLProps
from OCC.Core.gp import gp_Pnt
from OCC.Core.GProp import GProp_GProps
from OCC.Core.ShapeAnalysis import ShapeAnalysis_Surface
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED
from OCC.Core.TopExp import topexp, TopExp_Explorer
from OCC.Core.TopoDS import topods, TopoDS_Shape
from OCC.Core.TopTools import (
    TopTools_IndexedDataMapOfShapeListOfShape,
    TopTools_ListIteratorOfListOfShape,
)

from ..core.arc_segment import ArcSegment
from ..core.line_segment import LineSegment
from ..core.ptp_segment import PtPSegment
from ..core.seam import Seam, SeamConfig
from ..utils.params import ParamsBase

logger = logging.getLogger(__name__)


@dataclass
class SeamExtractorOCCTParams(ParamsBase):
    """Tuning for SeamExtractorOCCT; every field is optional."""

    epsilon: float = 1e-3
    num_smooth_points: int = 100
    coincidence_samples: int = 5

    def __post_init__(self) -> None:
        """Reject parameter values that fail late, or silently produce nothing.

        Raises:
            ValueError: If a parameter is out of range.
        """
        # Both sample counts divide by (count - 1); caught here rather than as a failed edge.
        if self.coincidence_samples < 2:
            raise ValueError(
                'coincidence_samples must be >= 2 (a curve needs two ends), '
                f'got {self.coincidence_samples}'
            )

        if self.num_smooth_points < 2:
            raise ValueError(
                'num_smooth_points must be >= 2 (a segment needs two ends), '
                f'got {self.num_smooth_points}'
            )

        if not self.epsilon > 0.0:
            raise ValueError(f'epsilon must be > 0, got {self.epsilon}')


class SeamExtractorOCCT:
    """Extract weld seams from OCCT shapes using face-to-face proximity detection.

    Uses BRepExtrema_DistShapeShape to find kissing faces, then BRepAlgoAPI_Common
    to extract exact intersection geometry.
    """

    def __init__(self, shape_1: TopoDS_Shape, shape_2: TopoDS_Shape,
                 params: dict | None = None) -> None:
        """Initialize OCCT seam extractor."""
        if shape_1.IsNull():
            raise ValueError('shape_1 is null')
        if shape_2.IsNull():
            raise ValueError('shape_2 is null')
        self.shape_1 = shape_1
        self.shape_2 = shape_2

        cfg = SeamExtractorOCCTParams.from_dict(params)
        self.num_smooth_points = cfg.num_smooth_points
        self.tolerance = cfg.epsilon
        self.coincidence_samples = cfg.coincidence_samples

        self.centroid_1 = self._compute_shape_centroid(shape_1)
        self.centroid_2 = self._compute_shape_centroid(shape_2)

        self._edge_face_maps = {
            id(shape_1): self._build_edge_face_map(shape_1),
            id(shape_2): self._build_edge_face_map(shape_2),
        }

    @staticmethod
    def _build_edge_face_map(
        shape: TopoDS_Shape,
    ) -> TopTools_IndexedDataMapOfShapeListOfShape:
        """Map every edge of a shape to the faces incident on it."""
        edge_face_map = TopTools_IndexedDataMapOfShapeListOfShape()
        topexp.MapShapesAndAncestors(
            shape, TopAbs_EDGE, TopAbs_FACE, edge_face_map)
        return edge_face_map

    def extract_seams(self) -> list[Seam]:
        """Extract all weld seams from the two shapes.

        Returns:
            List of Seam objects. Empty when no face pair is within tolerance.

        Raises:
            RuntimeError: If any face pair or intersection edge cannot be processed. Every edge
                is attempted first so the error names them all; a job missing a seam is the
                wrong job, not a smaller one.
        """
        logger.info('Starting OCCT seam extraction')
        contact_candidates = self._find_contact_face_pairs()

        if not contact_candidates:
            logger.warning('No face pairs found within tolerance - shapes may not be touching')
            return []

        logger.info(f'Found {len(contact_candidates)} contact face pair(s)')
        intersection_data = self._extract_intersection_edges(contact_candidates)

        if not intersection_data:
            logger.warning('No intersection edges found - check tolerance and geometry')
            return []

        logger.info(f'Extracted {len(intersection_data)} intersection edge(s)')
        seams = []
        failures = []

        for idx, edge_data in enumerate(intersection_data):
            try:
                edge_seams = self._process_single_edge(edge_data)
                seams.extend(edge_seams)
                logger.debug(
                    f'Processed edge {idx + 1}/{len(intersection_data)}: {len(edge_seams)} seam(s)'
                )

            except Exception as e:
                failures.append(f'edge {idx + 1}/{len(intersection_data)}: {e}')
                logger.error(f'Failed to process {failures[-1]}')

        if failures:
            raise RuntimeError(
                f'{len(failures)} intersection edge(s) could not be processed: '
                + '; '.join(failures)
            )

        logger.info(f'Extracted {len(seams)} seam(s)')
        return seams

    def _process_single_edge(self, edge_data: dict) -> list[Seam]:
        """Process a single intersection edge into Seam objects."""
        edge = edge_data['edge']
        face_A = edge_data['face_A']
        face_B = edge_data['face_B']

        edge_adapted = BRepAdaptor_Curve(edge)
        u_min = edge_adapted.FirstParameter()
        u_max = edge_adapted.LastParameter()

        points = []
        for i in range(self.num_smooth_points):
            u = u_min + (u_max - u_min) * i / (self.num_smooth_points - 1)
            pnt = edge_adapted.Value(u)
            points.append([pnt.X(), pnt.Y(), pnt.Z()])

        points = np.array(points)

        boundary_A = self._get_matching_boundary_edge(edge, face_A)
        boundary_B = self._get_matching_boundary_edge(edge, face_B)

        # A part with a real boundary edge on the seam gives its wall's normal. Without one the
        # edge is synthetic, from a curved intersection, and only the kissing face is there.
        if boundary_A is not None:
            wall_A = self._get_wall_surface_at_edge(boundary_A, self.shape_1, face_A)
            normals_A = self._extract_normals_from_surface(points, wall_A)
        else:
            normals_A = self._extract_normals_from_surface(points, face_A)

        if boundary_B is not None:
            wall_B = self._get_wall_surface_at_edge(boundary_B, self.shape_2, face_B)
            normals_B = self._extract_normals_from_surface(points, wall_B)
        else:
            normals_B = self._extract_normals_from_surface(points, face_B)

        normals_main, normals_secondary = self._determine_main_secondary_normals(
            normals_A, normals_B, points,
            boundary_A is not None, boundary_B is not None,
        )

        on_edges = (boundary_A is not None, boundary_B is not None)

        geometry = self._detect_geometry(edge, points)
        edge_seams = self._wrap_in_seams(geometry, on_edges, normals_main, normals_secondary)

        return edge_seams

    def _find_contact_face_pairs(self) -> list[dict]:
        """Find all face pairs within tolerance using BRepExtrema_DistShapeShape proximity check.

        Raises:
            RuntimeError: If the distance of any face pair cannot be computed. Every pair is
                attempted first so the error names them all; a pair left out may be a seam.
        """
        faces_1 = []
        exp1 = TopExp_Explorer(self.shape_1, TopAbs_FACE)
        while exp1.More():
            faces_1.append(topods.Face(exp1.Current()))
            exp1.Next()

        faces_2 = []
        exp2 = TopExp_Explorer(self.shape_2, TopAbs_FACE)
        while exp2.More():
            faces_2.append(topods.Face(exp2.Current()))
            exp2.Next()

        contact_candidates = []
        failures = []

        # Skips the exact distance solve for pairs whose inflated boxes are apart.
        boxes_1 = [self._bounding_box(face) for face in faces_1]
        boxes_2 = [self._bounding_box(face) for face in faces_2]

        for i, (face_A, box_A) in enumerate(zip(faces_1, boxes_1)):
            for j, (face_B, box_B) in enumerate(zip(faces_2, boxes_2)):
                if box_A.IsOut(box_B):
                    continue
                try:
                    dist_checker = BRepExtrema_DistShapeShape(face_A, face_B)
                    if not dist_checker.IsDone():
                        raise RuntimeError('distance solve did not finish')
                except Exception as e:
                    failures.append(f'face pair ({i}, {j}): {e}')
                    logger.error(f'Failed to compute distance for {failures[-1]}')
                    continue

                min_dist = dist_checker.Value()
                if min_dist <= self.tolerance:
                    contact_candidates.append({
                        'face_A': face_A,
                        'face_B': face_B,
                        'distance': min_dist
                    })

        if failures:
            raise RuntimeError(
                f'{len(failures)} face pair distance(s) could not be computed: '
                + '; '.join(failures)
            )

        logger.debug(
            f'Face-pair prefilter: {len(contact_candidates)} candidate(s) from '
            f'{len(faces_1)}x{len(faces_2)} face pairs'
        )
        return contact_candidates

    def _bounding_box(self, shape: TopoDS_Shape) -> Bnd_Box:
        """Axis-aligned bounds of a shape, inflated by half the contact tolerance on each side."""
        box = Bnd_Box()
        brepbndlib.Add(shape, box)
        box.Enlarge(0.5 * self.tolerance)
        return box

    def _extract_intersection_edges(self, contact_candidates: list[dict]) -> list[dict]:
        """Extract intersection edges from contact face pairs using BRepAlgoAPI_Common.

        Raises:
            RuntimeError: If the intersection of any contact pair cannot be computed. Every pair
                is attempted first so the error names them all; a pair within tolerance that
                yields nothing may be a seam lost.
        """
        intersection_data = []
        failures = []

        for idx, pair in enumerate(contact_candidates):
            face_A = pair['face_A']
            face_B = pair['face_B']

            try:
                common = BRepAlgoAPI_Common(face_A, face_B)
                # Fuzzy value allows geometric tolerance in the boolean — faces touching within
                # tolerance are treated as coincident.
                common.SetFuzzyValue(self.tolerance)
                common.Build()

                if not common.IsDone() or common.Shape().IsNull():
                    raise RuntimeError('boolean common did not finish')
            except Exception as e:
                failures.append(f'contact pair {idx + 1}/{len(contact_candidates)}: {e}')
                logger.error(f'Failed to intersect {failures[-1]}')
                continue

            edge_exp = TopExp_Explorer(common.Shape(), TopAbs_EDGE)
            while edge_exp.More():
                edge = topods.Edge(edge_exp.Current())
                intersection_data.append({
                    'edge': edge,
                    'face_A': face_A,
                    'face_B': face_B
                })
                edge_exp.Next()

        if failures:
            raise RuntimeError(
                f'{len(failures)} contact pair(s) could not be intersected: '
                + '; '.join(failures)
            )

        return self._drop_coincident_edges(intersection_data)

    def _drop_coincident_edges(self, intersection_data: list[dict]) -> list[dict]:
        """Drop intersection curves another face pair already produced.

        One weld curve can come from more than one face pair, and must be welded once.

        Returns:
            The same records, less those coincident with an earlier one.
        """
        kept: list[dict] = []
        for record in intersection_data:
            if any(self._edges_coincide(record['edge'], other['edge']) for other in kept):
                continue
            kept.append(record)

        dropped = len(intersection_data) - len(kept)
        if dropped:
            logger.info(
                f'Dropped {dropped} of {len(intersection_data)} intersection '
                'edge(s) that repeat a curve another face pair already gives'
            )
        return kept

    def _edges_coincide(self, edge_1: TopoDS_Shape, edge_2: TopoDS_Shape) -> bool:
        """Test whether two edges trace the same curve over the same extent.

        Checked both ways, since a sub-stretch lies on the longer edge but does not cover it.
        """
        return self._edge_lies_on(edge_1, edge_2) and self._edge_lies_on(edge_2, edge_1)

    def _edge_lies_on(self, edge: TopoDS_Shape, other: TopoDS_Shape) -> bool:
        """Whether every sample along `edge` sits within tolerance of `other`."""
        try:
            adaptor = BRepAdaptor_Curve(edge)
            first, last = adaptor.FirstParameter(), adaptor.LastParameter()

            for i in range(self.coincidence_samples):
                t = i / (self.coincidence_samples - 1)
                point = adaptor.Value(first + t * (last - first))
                extrema = BRepExtrema_DistShapeShape(
                    BRepBuilderAPI_MakeVertex(point).Vertex(), other)
                if not extrema.IsDone() or extrema.Value() > self.tolerance:
                    return False
            return True
        except Exception as e:
            # Safe to treat as distinct: the cost is a duplicate weld, not a wrong normal.
            logger.debug(f'Could not compare edges for coincidence: {e}')
            return False

    def _get_matching_boundary_edge(self, seam_edge: TopoDS_Shape,
                                    face: TopoDS_Shape
                                    ) -> TopoDS_Shape | None:
        """Get boundary edge from face that geometrically matches seam edge, or None if synthetic.

        When several edges align, the nearest wins rather than the first in explorer order.
        """
        edge_exp = TopExp_Explorer(face, TopAbs_EDGE)

        best_edge = None
        best_distance = float('inf')

        while edge_exp.More():
            boundary_edge = topods.Edge(edge_exp.Current())

            if self._edges_are_geometrically_aligned(seam_edge, boundary_edge):
                extrema = BRepExtrema_DistShapeShape(seam_edge, boundary_edge)
                distance = extrema.Value() if extrema.IsDone() else float('inf')
                if distance < best_distance:
                    best_edge, best_distance = boundary_edge, distance

            edge_exp.Next()

        return best_edge

    def _edges_are_geometrically_aligned(self, edge_1: TopoDS_Shape, edge_2: TopoDS_Shape) -> bool:
        """Check if two edges lie on the same geometric curve within tolerance.

        Raises:
            RuntimeError: If the comparison fails. False would take the kissing face's normal
                instead of the wall's, which is 90 degrees off.
        """
        try:
            adaptor_1 = BRepAdaptor_Curve(edge_1)
            adaptor_2 = BRepAdaptor_Curve(edge_2)

            curve_type_1 = adaptor_1.GetType()
            curve_type_2 = adaptor_2.GetType()

            if curve_type_1 != curve_type_2:
                return False

            if curve_type_1 == GeomAbs_Line:
                line_1 = adaptor_1.Line()
                line_2 = adaptor_2.Line()

                dir_1 = line_1.Direction()
                dir_2 = line_2.Direction()

                dot = abs(dir_1.Dot(dir_2))
                if dot < 0.9999:
                    return False

                pnt_start = adaptor_1.Value(adaptor_1.FirstParameter())
                pnt_end = adaptor_1.Value(adaptor_1.LastParameter())

                dist_start = line_2.Distance(pnt_start)
                dist_end = line_2.Distance(pnt_end)

                if not (dist_start <= self.tolerance and
                        dist_end <= self.tolerance):
                    return False

                # `Distance` is to the infinite line, so collinear segments must also overlap.
                origin = line_2.Location()
                direction = line_2.Direction()

                def parameter(point) -> float:
                    """Signed position of a point along line_2, in metres."""
                    return ((point.X() - origin.X()) * direction.X()
                            + (point.Y() - origin.Y()) * direction.Y()
                            + (point.Z() - origin.Z()) * direction.Z())

                span_1 = sorted((parameter(pnt_start), parameter(pnt_end)))
                span_2 = sorted((
                    parameter(adaptor_2.Value(adaptor_2.FirstParameter())),
                    parameter(adaptor_2.Value(adaptor_2.LastParameter())),
                ))

                overlap = min(span_1[1], span_2[1]) - max(span_1[0], span_2[0])
                return overlap >= -self.tolerance

            elif curve_type_1 == GeomAbs_Circle:
                circle_1 = adaptor_1.Circle()
                circle_2 = adaptor_2.Circle()

                center_1 = circle_1.Location()
                center_2 = circle_2.Location()

                center_dist = center_1.Distance(center_2)
                if center_dist > self.tolerance:
                    return False

                radius_diff = abs(circle_1.Radius() - circle_2.Radius())
                if radius_diff > self.tolerance:
                    return False

                normal_1 = circle_1.Axis().Direction()
                normal_2 = circle_2.Axis().Direction()

                dot = abs(normal_1.Dot(normal_2))
                return dot > 0.9999

            else:
                for i in range(self.coincidence_samples):
                    t = i / (self.coincidence_samples - 1)
                    u = (
                        adaptor_1.FirstParameter() +
                        t * (adaptor_1.LastParameter() - adaptor_1.FirstParameter())
                    )
                    pnt = adaptor_1.Value(u)

                    extrema = BRepExtrema_DistShapeShape(
                        BRepBuilderAPI_MakeVertex(pnt).Vertex(),
                        edge_2
                    )

                    if not extrema.IsDone():
                        return False

                    if extrema.Value() > self.tolerance:
                        return False

                return True

        except Exception as e:
            raise RuntimeError(f'Could not compare edge alignment: {e}') from e

    def _get_wall_surface_at_edge(self,
                                  boundary_edge: TopoDS_Shape,
                                  shape: TopoDS_Shape,
                                  kissing_face: TopoDS_Shape
                                  ) -> TopoDS_Shape:
        """Get the face at the boundary edge that is not the kissing face; no angle is checked."""
        edge_face_map = self._edge_face_maps[id(shape)]

        if not edge_face_map.Contains(boundary_edge):
            raise RuntimeError('Boundary edge not found in shape topology')

        face_list = edge_face_map.FindFromKey(boundary_edge)

        faces_at_edge = []
        it = TopTools_ListIteratorOfListOfShape(face_list)
        while it.More():
            faces_at_edge.append(topods.Face(it.Value()))
            it.Next()

        wall_candidates = [f for f in faces_at_edge if not f.IsSame(kissing_face)]

        if not wall_candidates:
            raise RuntimeError('No wall surface found at edge')

        return wall_candidates[0]

    # Pipe joint detection, disabled; see the TODO at the top of this file.
    # def _has_inner_holes(self, face: TopoDS_Shape) -> bool:
    #     """Check if face has inner holes (multiple wire boundaries)."""
    #     wire_explorer = TopExp_Explorer(face, TopAbs_WIRE)
    #     num_wires = 0
    #     while wire_explorer.More():
    #         num_wires += 1
    #         wire_explorer.Next()
    #     return num_wires > 1

    # def _get_pipe_surfaces(self, edge: TopoDS_Shape, face_1: TopoDS_Shape,
    #                        face_2: TopoDS_Shape) -> dict:
    #     """Get outer shaft surfaces for pipe joint."""
    #     has_hole_1 = self._has_inner_holes(face_1)
    #     has_hole_2 = self._has_inner_holes(face_2)
    #     shaft_1 = None
    #     shaft_2 = None
    #     if has_hole_1:
    #         try:
    #             shaft_1 = self._get_outer_shaft_surface(edge, self.shape_1, face_1)
    #         except Exception as e:
    #             warnings.warn(f'Could not get shaft_1 surface: {e}')
    #     if has_hole_2:
    #         try:
    #             shaft_2 = self._get_outer_shaft_surface(edge, self.shape_2, face_2)
    #         except Exception as e:
    #             warnings.warn(f'Could not get shaft_2 surface: {e}')
    #     return {'shaft_1': shaft_1, 'shaft_2': shaft_2}

    # def _get_outer_shaft_surface(self, edge: TopoDS_Shape, shape: TopoDS_Shape,
    #                              kissing_face: TopoDS_Shape) -> TopoDS_Shape:
    #     """Get outer shaft surface for pipe using G1 continuity check."""
    #     neighbors = self._get_neighbor_faces(shape, kissing_face)
    #     if not neighbors:
    #         raise RuntimeError('No neighbor faces found for kissing surface')
    #     discontinuous_faces = []
    #     for neighbor in neighbors:
    #         if not self._are_faces_continuous(kissing_face, neighbor, edge):
    #             discontinuous_faces.append(neighbor)
    #     if not discontinuous_faces:
    #         return max(neighbors, key=lambda f: self._compute_face_area(f))
    #     shaft_surface = max(discontinuous_faces, key=lambda f: self._compute_face_area(f))
    #     return shaft_surface

    # def _are_faces_continuous(self, face_1: TopoDS_Shape, face_2: TopoDS_Shape,
    #                           shared_edge: TopoDS_Shape) -> bool:
    #     """Check if faces meet smoothly (G1 continuity)."""
    #     try:
    #         adaptor = BRepAdaptor_Curve(shared_edge)
    #         pnt = adaptor.Value(adaptor.FirstParameter())
    #         point = np.array([pnt.X(), pnt.Y(), pnt.Z()])
    #         n1 = self._evaluate_normal_at_point(point, face_1)
    #         n2 = self._evaluate_normal_at_point(point, face_2)
    #         return np.abs(np.dot(n1, n2)) > 0.98
    #     except Exception:
    #         return False

    # def _get_neighbor_faces(
    #         self, shape: TopoDS_Shape, target_face: TopoDS_Shape) -> list[TopoDS_Shape]:
    #     """Get faces that share edges with target face."""
    #     edge_face_map = TopTools_IndexedDataMapOfShapeListOfShape()
    #     topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_face_map)
    #     neighbors = set()
    #     edge_exp = TopExp_Explorer(target_face, TopAbs_EDGE)
    #     while edge_exp.More():
    #         edge = edge_exp.Current()
    #         if edge_face_map.Contains(edge):
    #             face_list = edge_face_map.FindFromKey(edge)
    #             it = TopTools_ListIteratorOfListOfShape(face_list)
    #             while it.More():
    #                 neighbor = topods.Face(it.Value())
    #                 if not neighbor.IsSame(target_face):
    #                     neighbors.add(neighbor)
    #                 it.Next()
    #         edge_exp.Next()
    #     return list(neighbors)

    # def _compute_face_area(self, face: TopoDS_Shape) -> float:
    #     """Compute surface area of a face."""
    #     props = GProp_GProps()
    #     brepgprop.SurfaceProperties(face, props)
    #     return props.Mass()

    # def _get_pipe_normals(
    #         self, points: NDArray, surfaces: dict) -> tuple[NDArray, NDArray]:
    #     """Get normals for pipe joint from outer shafts."""
    #     shaft_1 = surfaces['shaft_1']
    #     shaft_2 = surfaces['shaft_2']
    #     normals_1 = []
    #     normals_2 = []
    #     for point in points:
    #         if shaft_1:
    #             try:
    #                 n1 = self._evaluate_normal_at_point(point, shaft_1)
    #                 normals_1.append(n1)
    #             except Exception:
    #                 normals_1.append(np.array([0, 0, 1]))
    #         else:
    #             normals_1.append(np.array([0, 0, 1]))
    #         if shaft_2:
    #             try:
    #                 n2 = self._evaluate_normal_at_point(point, shaft_2)
    #                 normals_2.append(n2)
    #             except Exception:
    #                 normals_2.append(np.array([0, 0, -1]))
    #         else:
    #             normals_2.append(np.array([0, 0, -1]))
    #     normals_1 = np.array(normals_1)
    #     normals_2 = np.array(normals_2)
    #     normals_1 = self._make_normals_consistent(normals_1)
    #     normals_2 = self._make_normals_consistent(normals_2)
    #     return self._determine_main_secondary_normals(normals_1, normals_2)

    def _extract_normals_from_surface(self,
                                      points: NDArray,
                                      surface: TopoDS_Shape
                                      ) -> NDArray:
        """Extract normals at each point on a surface.

        Raises:
            RuntimeError: If a normal cannot be evaluated; no fallback direction is guessed.
        """
        normals = np.array([self._evaluate_normal_at_point(point, surface) for point in points])
        self._warn_on_opposing_normals(normals)
        return normals

    def _warn_on_opposing_normals(self, normals: NDArray) -> None:
        """Warn when adjacent normals oppose; they are never flipped.

        Face orientation is already applied, and no rule reading only the normals can tell a
        flipped normal from a curved surface sampled too coarsely.
        """
        adjacent = np.einsum('ij,ij->i', normals[:-1], normals[1:])
        opposing = int(np.sum(adjacent < 0.0))
        if opposing:
            logger.warning(
                f'{opposing} of {len(normals) - 1} adjacent normal pair(s) oppose each other on '
                'this edge: inconsistent OCCT orientation, or num_smooth_points too low for its '
                'curvature. Passed through unchanged.'
            )

    def _compute_shape_centroid(self, shape: TopoDS_Shape) -> NDArray:
        """Compute the volumetric centroid of an OCCT shape."""
        props = GProp_GProps()
        brepgprop.VolumeProperties(shape, props)
        cog = props.CentreOfMass()
        return np.array([cog.X(), cog.Y(), cog.Z()])

    def _determine_main_secondary_normals(self,
                                          normals_A: NDArray,
                                          normals_B: NDArray,
                                          seam_points: NDArray,
                                          has_boundary_A: bool,
                                          has_boundary_B: bool,
                                          ) -> tuple[NDArray, NDArray]:
        """Determine which normals are main (base) vs secondary (wall).

        The part with a boundary edge on the seam ends there and supplies the wall; the other
        carries the seam across its face. The centroid projection only decides when both or
        neither part has one.

        Args:
            normals_A: Normals per seam point taken from shape_1; the centroid test reads
                `centroid_1` for it, so the order matters.
            normals_B: Normals per seam point taken from shape_2.
            seam_points: The seam's sampled points (N, 3).
            has_boundary_A: Whether shape_1 has a real boundary edge on the seam.
            has_boundary_B: Whether shape_2 has a real boundary edge on the seam.

        Returns:
            (normals_main, normals_secondary).
        """
        if has_boundary_A != has_boundary_B:
            if has_boundary_A:
                return normals_B, normals_A
            return normals_A, normals_B

        avg_normal_A = np.mean(normals_A, axis=0)
        avg_normal_B = np.mean(normals_B, axis=0)

        norm_A = np.linalg.norm(avg_normal_A)
        norm_B = np.linalg.norm(avg_normal_B)

        if norm_A < 1e-10 or norm_B < 1e-10:
            logger.warning('Zero-length average normal detected, defaulting to A as main')
            return normals_A, normals_B

        avg_normal_A = avg_normal_A / norm_A
        avg_normal_B = avg_normal_B / norm_B

        seam_centroid = np.mean(seam_points, axis=0)
        vec_A = seam_centroid - self.centroid_1
        vec_B = seam_centroid - self.centroid_2

        norm_vA = np.linalg.norm(vec_A)
        norm_vB = np.linalg.norm(vec_B)

        if norm_vA < 1e-10 or norm_vB < 1e-10:
            logger.warning('Degenerate centroid-to-seam vector, defaulting to A as main')
            return normals_A, normals_B

        vec_A = vec_A / norm_vA
        vec_B = vec_B / norm_vB

        # The part whose average normal aligns more with its centroid-to-seam vector has the seam
        # on its outward face, so it is the base plate.
        dot_A = np.dot(avg_normal_A, vec_A)
        dot_B = np.dot(avg_normal_B, vec_B)

        logger.debug(f'Centroid-based main selection: dot_A={dot_A:.3f}, dot_B={dot_B:.3f}')

        if dot_A >= dot_B:
            return normals_A, normals_B
        return normals_B, normals_A

    def _evaluate_normal_at_point(self, point: NDArray, face: TopoDS_Shape) -> NDArray:
        """Evaluate unit surface normal at point using UV projection (fast path for planes)."""
        adaptor = BRepAdaptor_Surface(face)

        if adaptor.GetType() == GeomAbs_Plane:
            position = adaptor.Plane().Position()
            normal_dir = position.Direction()

            # The surface normal is XDirection x YDirection, which is the main direction only for
            # a right-handed frame. Mirrored parts can arrive with left-handed planes.
            if not position.Direct():
                normal_dir.Reverse()

            if face.Orientation() == TopAbs_REVERSED:
                normal_dir.Reverse()

            normal = np.array([normal_dir.X(), normal_dir.Y(), normal_dir.Z()])
            return normal / np.linalg.norm(normal)

        pnt = gp_Pnt(point[0], point[1], point[2])
        surface = BRep_Tool.Surface(face)

        sas = ShapeAnalysis_Surface(surface)
        uv = sas.ValueOfUV(pnt, self.tolerance)

        # 1 = compute up to first derivatives (required to evaluate the normal).
        props = GeomLProp_SLProps(surface, uv.X(), uv.Y(), 1, self.tolerance)

        if props.IsNormalDefined():
            normal_dir = props.Normal()
            if face.Orientation() == TopAbs_REVERSED:
                normal_dir.Reverse()
            normal = np.array([normal_dir.X(), normal_dir.Y(), normal_dir.Z()])
            return normal / np.linalg.norm(normal)

        raise RuntimeError(f'Normal not defined at point {point}')

    def _detect_geometry(self, edge: TopoDS_Shape, points: NDArray) -> dict:
        """Detect geometry type (line, arc, or ptp) and extract geometric parameters."""
        edge_adapted = BRepAdaptor_Curve(edge)
        curve_type = edge_adapted.GetType()

        if curve_type == GeomAbs_Line:
            return {
                'type': 'line',
                'points': points,
                'start': points[0],
                'end': points[-1]
            }

        elif curve_type == GeomAbs_Circle:
            circle = edge_adapted.Circle()
            center_gp = circle.Location()

            return {
                'type': 'arc',
                'points': points,
                'center': np.array([center_gp.X(), center_gp.Y(), center_gp.Z()]),
                'radius': circle.Radius(),
            }

        else:
            # A false line or arc misleads the planner; PtP keeps the sampled points.
            logger.debug(f'Curve type {curve_type} is neither line nor circle; kept as PtP')
            return {
                'type': 'ptp',
                'points': points,
            }

    def _wrap_in_seams(self,
                       geometry: dict,
                       on_edges: tuple[bool, bool],
                       normals_main: NDArray,
                       normals_secondary: NDArray
                       ) -> list[Seam]:
        """Wrap geometry and normals into Seam objects with metadata.

        A closed arc is split into two halves: the welder runs one Pilz CIRC per arc, from its
        first point to its last, and a full circle puts that goal on top of its start.

        `on_edges` says, for shape_1 and shape_2 in that order, whether the seam follows a real
        edge of that shape.
        """
        points = geometry['points']
        kind = geometry['type']

        if kind == 'arc' and np.linalg.norm(points[-1] - points[0]) <= self.tolerance:
            if len(points) < 5:
                raise RuntimeError(
                    f'closed arc sampled at {len(points)} points cannot be split into two arcs; '
                    'raise num_smooth_points'
                )
            mid = len(points) // 2
            halves = (slice(0, mid + 1), slice(mid, len(points)))
            return [
                self._wrap_one_seam(
                    dict(geometry, points=points[half]), on_edges,
                    normals_main[half], normals_secondary[half])
                for half in halves
            ]

        return [self._wrap_one_seam(geometry, on_edges, normals_main, normals_secondary)]

    def _wrap_one_seam(self,
                       geometry: dict,
                       on_edges: tuple[bool, bool],
                       normals_main: NDArray,
                       normals_secondary: NDArray
                       ) -> Seam:
        """Wrap geometry and normals into one Seam object with metadata."""
        points = geometry['points']
        kind = geometry['type']
        config = SeamConfig(
            smoothed_points=points,
            normals_main=normals_main,
            normals_secondary=normals_secondary,
            on_edge_1=on_edges[0],
            on_edge_2=on_edges[1],
        )

        if kind == 'line':
            return Seam(
                line_segment=LineSegment(start=geometry['start'], end=geometry['end']),
                config=config)
        if kind == 'arc':
            return Seam(arc_segment=ArcSegment(
                points=points, center=geometry['center'], radius=geometry['radius']),
                config=config)
        return Seam(ptp_segment=PtPSegment(points=points), config=config)
