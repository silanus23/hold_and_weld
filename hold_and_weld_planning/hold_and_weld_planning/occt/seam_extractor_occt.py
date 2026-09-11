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

"""SeamExtractorOCCT - Extract weld seams from OCCT shapes using exact geometry.

This module uses face-to-face proximity detection and BRepAlgoAPI_Common to find
kissing surfaces, then extracts exact seam curves and surface normals.
"""

# TODO: (@silanus23) Complete pipe logic
# Warning: Pipe joint detection is under construction. Current implementation
# treats any face with inner holes (>1 wire) as a pipe joint and attempts to
# extract normals from outer shaft surfaces using G1 continuity checks.
# This logic is not fully tested and may produce incorrect normals for:
# - Non-cylindrical shafts
# - Complex geometries with multiple discontinuous faces
# - Stepped or flanged pipes
# Known issues:
# - Shaft selection uses largest discontinuous face (area-based heuristic)
# - Hardcoded fallback normals [0,0,1] should fail instead
# - G1 continuity check samples only one point on edge

import logging
from typing import Dict, List, Tuple

import numpy as np

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
from ..core.seam import Seam

logger = logging.getLogger(__name__)


class SeamExtractorOCCT:
    """Extract weld seams from OCCT shapes using face-to-face proximity detection.

    Uses BRepExtrema_DistShapeShape to find kissing faces, then BRepAlgoAPI_Common
    to extract exact intersection geometry.
    """

    def __init__(self, shape_1: TopoDS_Shape, shape_2: TopoDS_Shape, params: Dict) -> None:
        """Initialize OCCT seam extractor.

        Args:
            shape_1: First OCCT shape (already transformed to world frame)
            shape_2: Second OCCT shape (already transformed to world frame)
            params: Dictionary with keys:
                    - num_smooth_points: Points per seam curve (default 100)
                    - epsilon: Distance tolerance for contact detection (default 1e-3)
        """
        if shape_1.IsNull():
            raise ValueError('shape_1 is null')
        if shape_2.IsNull():
            raise ValueError('shape_2 is null')
        self.shape_1 = shape_1
        self.shape_2 = shape_2
        self.params = params

        self.num_smooth_points = int(params.get('num_smooth_points', 100))
        self.tolerance = float(params.get('epsilon', 1e-3))
        # Samples per curve when testing two intersection edges for
        # coincidence. Endpoints plus interior: enough to separate two curves
        # that share their ends but not their middle.
        self.coincidence_samples = int(params.get('coincidence_samples', 5))
        self._validate_params()

        # Centroids for geometry-based main/secondary determination
        self.centroid_1 = self._compute_shape_centroid(shape_1)
        self.centroid_2 = self._compute_shape_centroid(shape_2)

        # Edge -> incident faces, once per shape. `_get_wall_surface_at_edge`
        # is called for every boundary edge of every intersection and used to
        # rebuild this map each time, which is a full topological traversal of
        # the whole shape per edge.
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

    def _validate_params(self) -> None:
        """Reject parameter values that fail late, or silently produce nothing.

        Raises:
            ValueError: If a parameter is out of range.
        """
        # `_process_single_edge` samples the curve at
        # `i / (num_smooth_points - 1)`, so 1 divides by zero. That used to be
        # swallowed by the per-edge handler in `extract_seams` and came out as
        # an EMPTY weld path rather than an error - measured: 0 and 1 both
        # yield zero seams, silently. Two points is the minimum that describes
        # a segment at all.
        # `_edge_lies_on` divides by `coincidence_samples - 1` in the same way.
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

        # Drives BRepExtrema face-pair acceptance and the Common fuzzy value;
        # at or below zero no face pair is ever admitted and the parts read as
        # not touching.
        if not self.tolerance > 0.0:
            raise ValueError(f'epsilon must be > 0, got {self.tolerance}')

    def extract_seams(self) -> List[Seam]:
        """Extract all weld seams from the two shapes.

        Pipeline:
        1. Find all face pairs within tolerance
        2. Extract intersection edges using Common
        3. Classify each edge and extract normals
        4. Detect geometry and wrap in Seam objects

        Returns:
            List of Seam objects with geometry and metadata.
            Empty list if no seams found. Individual seam failures are logged
            but don't stop extraction.
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
        failed_count = 0

        for idx, edge_data in enumerate(intersection_data):
            try:
                edge_seams = self._process_single_edge(edge_data)
                seams.extend(edge_seams)
                logger.debug(
                    f'Processed edge {idx + 1}/{len(intersection_data)}: {len(edge_seams)} seam(s)'
                )

            except Exception as e:
                failed_count += 1
                logger.warning(f'Failed to process edge {idx + 1}/{len(intersection_data)}: {e}')

        logger.info(f'Successfully extracted {len(seams)} seam(s), {failed_count} edge(s) failed')
        return seams

    def _process_single_edge(self, edge_data: Dict) -> List[Seam]:
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

        if len(points) < 2:
            raise RuntimeError('Edge sampling produced fewer than 2 points')

        # PIPE LOGIC DISABLED - under development (see commented methods below)
        # Pipe joints would check for inner holes and extract normals from outer shaft surfaces

        # REGULAR JOINT: Determine surfaces for normal extraction
        boundary_A = self._get_matching_boundary_edge(edge, face_A)
        boundary_B = self._get_matching_boundary_edge(edge, face_B)

        # Extract normals from appropriate surfaces (walls if boundaries
        # exist, kissing faces otherwise)
        if boundary_A is not None:
            # Shape_1 has real boundary - use wall surface
            wall_A = self._get_wall_surface_at_edge(boundary_A, self.shape_1, face_A)
            normals_A = self._extract_normals_from_surface(points, wall_A)
        else:
            # No boundary - use kissing face (synthetic edge from curved intersection)
            normals_A = self._extract_normals_from_surface(points, face_A)

        if boundary_B is not None:
            # Shape_2 has real boundary - use wall surface
            wall_B = self._get_wall_surface_at_edge(boundary_B, self.shape_2, face_B)
            normals_B = self._extract_normals_from_surface(points, wall_B)
        else:
            # No boundary - use kissing face (synthetic edge from curved intersection)
            normals_B = self._extract_normals_from_surface(points, face_B)

        normals_main, normals_secondary = self._determine_main_secondary_normals(
            normals_A, normals_B, points,
            boundary_A is not None, boundary_B is not None,
        )

        is_edge_joint = (boundary_A is not None) and (boundary_B is not None)

        geometry = self._detect_geometry(edge, points)
        edge_seams = self._wrap_in_seams(geometry, is_edge_joint, normals_main, normals_secondary)

        return edge_seams

    def _find_contact_face_pairs(self) -> List[Dict]:
        """Find all face pairs within tolerance using BRepExtrema_DistShapeShape proximity check.

        Uses BRepExtrema_DistShapeShape to find all face pairs within tolerance.
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

        # Every face pair otherwise gets an exact minimum-distance solve, which
        # is by far the most expensive call here and is wasted on the great
        # majority of pairs that are nowhere near each other. A bounding box
        # inflated by the tolerance cannot exclude a pair that is within it,
        # so this only removes work.
        boxes_1 = [self._bounding_box(face) for face in faces_1]
        boxes_2 = [self._bounding_box(face) for face in faces_2]

        for face_A, box_A in zip(faces_1, boxes_1):
            for face_B, box_B in zip(faces_2, boxes_2):
                if box_A.IsOut(box_B):
                    continue
                try:
                    dist_checker = BRepExtrema_DistShapeShape(face_A, face_B)

                    if dist_checker.IsDone():
                        min_dist = dist_checker.Value()

                        if min_dist <= self.tolerance:
                            contact_candidates.append({
                                'face_A': face_A,
                                'face_B': face_B,
                                'distance': min_dist
                            })
                except Exception as e:
                    logger.debug(f'Failed to compute distance for face pair: {e}')
                    continue

        logger.debug(
            f'Face-pair prefilter: {len(contact_candidates)} candidate(s) from '
            f'{len(faces_1)}x{len(faces_2)} face pairs'
        )
        return contact_candidates

    def _bounding_box(self, shape: TopoDS_Shape) -> Bnd_Box:
        """Axis-aligned bounds of a shape, inflated by the contact tolerance.

        Inflated by half the tolerance on each of the pair, so two boxes
        overlap whenever the shapes could be within `tolerance` of each other.
        """
        box = Bnd_Box()
        brepbndlib.Add(shape, box)
        box.Enlarge(0.5 * self.tolerance)
        return box

    def _extract_intersection_edges(self, contact_candidates: List[Dict]) -> List[Dict]:
        """Extract intersection edges from contact face pairs using BRepAlgoAPI_Common."""
        intersection_data = []

        for pair in contact_candidates:
            face_A = pair['face_A']
            face_B = pair['face_B']

            try:
                common = BRepAlgoAPI_Common(face_A, face_B)
                # Fuzzy value allows geometric tolerance in the boolean — faces touching
                # within tolerance are treated as coincident.
                common.SetFuzzyValue(self.tolerance)
                common.Build()

                if not common.IsDone() or common.Shape().IsNull():
                    continue

                result = common.Shape()

                edge_exp = TopExp_Explorer(result, TopAbs_EDGE)
                while edge_exp.More():
                    edge = topods.Edge(edge_exp.Current())
                    intersection_data.append({
                        'edge': edge,
                        'face_A': face_A,
                        'face_B': face_B
                    })
                    edge_exp.Next()

            except Exception as e:
                logger.debug(f'Failed to extract intersection edges from face pair: {e}')
                continue

        return self._drop_coincident_edges(intersection_data)

    def _drop_coincident_edges(self, intersection_data: List[Dict]) -> List[Dict]:
        """Drop intersection curves another accepted pair already covers.

        One weld curve is reached by more than one face pair: on a box
        standing on a plate, the box's underside against the plate's top
        yields the whole rim, and each of the box's four side faces against
        that same top yields one rim line again. Both are real intersections
        of real face pairs, so nothing upstream can tell them apart - but the
        curve is one weld and must be laid down once.

        The mesh pipeline settles this the same way, in
        `seam_extractor_mesh.drop_coincident`.

        Args:
            intersection_data: Edge records, in the order the face pairs were
                walked.

        Returns:
            The same records, less those coincident with an earlier one. The
            EARLIER record wins, so which face pair supplies the normals
            follows face-explorer order.
        """
        kept: List[Dict] = []
        for record in intersection_data:
            if any(self._edges_coincide(record['edge'], other['edge'])
                   for other in kept):
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

        Sampled in BOTH directions: a one-way test passes for an edge that is
        merely a sub-stretch of the other, and dropping that would lose the
        part of the weld the longer edge does not cover.
        """
        return (self._edge_lies_on(edge_1, edge_2)
                and self._edge_lies_on(edge_2, edge_1))

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
            # Unlike `_edges_are_geometrically_aligned`, a failure here is safe
            # to report as "not the same curve": the cost is a duplicate weld
            # pass, not a 90-degree normal error.
            logger.debug(f'Could not compare edges for coincidence: {e}')
            return False

    def _get_matching_boundary_edge(self, seam_edge: TopoDS_Shape,
                                    face: TopoDS_Shape
                                    ) -> TopoDS_Shape:
        """Get boundary edge from face that geometrically matches seam edge, or None if synthetic.

        Returns None if the edge is synthetic.

        Where several of the face's edges pass the alignment test — the four
        sides of a square face all lie on the same plane as the seam, and two
        opposite ones can both align with a seam that runs between them — the
        NEAREST is taken. Explorer order is a property of how the shape was
        built, not of the geometry, so letting it decide picks the wall on the
        far side of the face as often as the right one.
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

        Raises rather than returning False when the comparison cannot be made.
        False means "these are different curves", which routes the caller to
        the KISSING FACE for its normal instead of the wall - and those are
        perpendicular by construction, so reporting a failed comparison as
        False emits a seam whose every normal is 90 degrees wrong, silently.
        Measured at exactly 90.0 degrees mean error on a cube-on-plate joint.
        A raised error costs the one edge, which `extract_seams` already
        counts and reports.
        """
        try:
            adaptor_1 = BRepAdaptor_Curve(edge_1)
            adaptor_2 = BRepAdaptor_Curve(edge_2)

            curve_type_1 = adaptor_1.GetType()
            curve_type_2 = adaptor_2.GetType()

            if curve_type_1 != curve_type_2:
                return False

            # Line alignment: check parallel directions and endpoints lie on same infinite line
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

                # `Distance` is to the INFINITE line, so the test above is
                # satisfied by two collinear segments that do not touch: the
                # near and far edges of a long face lie on one line as surely
                # as two halves of the same edge do. Require the segments to
                # actually share some of that line before calling them the
                # same curve.
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

                overlap = (min(span_1[1], span_2[1])
                           - max(span_1[0], span_2[0]))
                return overlap >= -self.tolerance

            # Circle alignment: check centers, radii, and plane normals match
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

            # General curves: sample points along edge_1 and check distance to edge_2.
            # 5 samples balances cost vs coverage for typical short intersection edges;
            # increase if false positives occur on long spline edges.
            else:
                num_samples = 5
                for i in range(num_samples):
                    t = i / (num_samples - 1)
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
            raise RuntimeError(
                f'Could not compare edge alignment: {e}. Refusing to report '
                'this as "not aligned" - that would silently substitute the '
                'kissing-face normal for the wall normal, a 90 degree error.'
            )

    def _get_wall_surface_at_edge(self,
                                  boundary_edge: TopoDS_Shape,
                                  shape: TopoDS_Shape,
                                  kissing_face: TopoDS_Shape
                                  ) -> TopoDS_Shape:
        """Get perpendicular wall surface at boundary edge (excludes kissing face)."""
        edge_face_map = self._edge_face_maps.get(id(shape))
        if edge_face_map is None:
            edge_face_map = self._build_edge_face_map(shape)

        if not edge_face_map.Contains(boundary_edge):
            raise RuntimeError('Boundary edge not found in shape topology')

        face_list = edge_face_map.FindFromKey(boundary_edge)

        faces_at_edge = []
        it = TopTools_ListIteratorOfListOfShape(face_list)
        while it.More():
            faces_at_edge.append(topods.Face(it.Value()))
            it.Next()

        # Filter out kissing face to get wall
        wall_candidates = [f for f in faces_at_edge if not f.IsSame(kissing_face)]

        if not wall_candidates:
            raise RuntimeError('No wall surface found at edge')

        return wall_candidates[0]

    # PIPE LOGIC COMMENTED OUT - INCOMPLETE/UNDER DEVELOPMENT
    # def _has_inner_holes(self, face: TopoDS_Shape) -> bool:
    #     """Check if face has inner holes (multiple wire boundaries)."""
    #     wire_explorer = TopExp_Explorer(face, TopAbs_WIRE)
    #     num_wires = 0
    #     while wire_explorer.More():
    #         num_wires += 1
    #         wire_explorer.Next()
    #     return num_wires > 1

    # def _get_pipe_surfaces(self, edge: TopoDS_Shape, face_1: TopoDS_Shape,
    #                        face_2: TopoDS_Shape) -> Dict:
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
    #         self, shape: TopoDS_Shape, target_face: TopoDS_Shape) -> List[TopoDS_Shape]:
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
    #         self, points: np.ndarray, surfaces: Dict) -> Tuple[np.ndarray, np.ndarray]:
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
                                      points: np.ndarray,
                                      surface: TopoDS_Shape
                                      ) -> np.ndarray:
        """Extract normals at each point on a surface.

        Fails rather than guesses. The previous behaviour substituted the
        preceding point's normal, or a hardcoded [0,0,1] for the first point,
        and returned an array indistinguishable from a measured one - so a
        total failure to evaluate any normal produced a full set of
        straight-up normals and a seam that looked perfectly well formed.
        Nothing downstream can catch that: WeldPlanner validates normals for
        presence and length only, never direction.

        The ONE fallback kept is the previous normal ON A PLANAR FACE, where
        it is not a guess: a plane's normal is constant, so the preceding
        value is exactly the right answer. On any curved surface consecutive
        normals genuinely differ and reusing one is wrong, so that raises.

        Raises:
            RuntimeError: If a normal cannot be evaluated and cannot be
                          recovered exactly.
        """
        planar = BRepAdaptor_Surface(surface).GetType() == GeomAbs_Plane
        normals = []

        for index, point in enumerate(points):
            try:
                normal = self._evaluate_normal_at_point(point, surface)
                normals.append(normal)
            except Exception as e:
                if normals and planar:
                    normals.append(normals[-1])
                    logger.debug(
                        f'Normal failed at point {index} on a planar face; '
                        f'reusing the previous normal, exact here: {e}'
                    )
                else:
                    raise RuntimeError(
                        f'Could not evaluate the surface normal at point '
                        f'{index} of {len(points)}'
                        + ('' if normals else ' (the first point)')
                        + f' on a {"planar" if planar else "curved"} face: {e}'
                    )

        normals = np.array(normals)
        normals = self._make_normals_consistent(normals)

        return normals

    def _make_normals_consistent(self, normals: np.ndarray) -> np.ndarray:
        """Report inconsistent normal orientation along an edge; change nothing.

        Returns its input untouched. `_evaluate_normal_at_point` already
        respects face orientation (TopAbs_REVERSED), so the normals arrive
        correctly oriented, and measurement agrees: across a cube on a plate,
        cylinder sectors from 90 to 360 degrees, and the shipped workpiece
        scene, ZERO raw normals were ever wrongly signed. This has never had
        anything to correct.

        It used to rewrite them anyway, and every version of that rule
        damaged correct data as soon as its premise failed:

        - Comparing each normal against the MEAN assumes the whole seam fits
          inside one hemisphere. True of a flat wall, false of a curved one:
          0 normals wrongly reversed at 180 degrees, 2 of 40 at 181, 14 at
          270, 20 of 40 at 360 - half a round seam pointing into the part.
        - Comparing each normal against its NEIGHBOUR assumes adjacent
          samples never truly oppose. False once the sample step passes 90
          degrees: 1 of 3 and 2 of 4 on a full circle. At exactly 180 degrees
          a turned normal and a flipped one are the same vector, so no rule
          reading only the normals can separate them.

        Nothing downstream can catch a reversed normal either - WeldPlanner
        validates normals for presence and array length, never direction - so
        the safe move is to measure and say so rather than guess. If this
        warning ever fires there is a real case to design against; until then
        there is not.
        """
        if len(normals) < 2:
            return normals

        adjacent = np.einsum('ij,ij->i', normals[:-1], normals[1:])
        opposing = int(np.sum(adjacent < 0.0))
        if opposing:
            logger.warning(
                f'{opposing} of {len(normals) - 1} adjacent normal pair(s) '
                'oppose each other on this edge. Either OCCT reported '
                'inconsistent orientation, or the edge is sampled too '
                'coarsely for its curvature (check num_smooth_points). The '
                'normals are passed through unchanged - verify them before '
                'trusting the weld poses from this seam.'
            )
        return normals

    def _compute_shape_centroid(self, shape: TopoDS_Shape) -> np.ndarray:
        """Compute the volumetric centroid of an OCCT shape."""
        props = GProp_GProps()
        brepgprop.VolumeProperties(shape, props)
        cog = props.CentreOfMass()
        return np.array([cog.X(), cog.Y(), cog.Z()])

    def _determine_main_secondary_normals(self,
                                          normals_A: np.ndarray,
                                          normals_B: np.ndarray,
                                          seam_points: np.ndarray,
                                          has_boundary_A: bool,
                                          has_boundary_B: bool,
                                          ) -> Tuple[np.ndarray, np.ndarray]:
        """Determine which normals are main (base) vs secondary (wall).

        MAIN is the base surface the seam lies ON, SECONDARY the wall of the
        part that terminates there. This is the convention the mesh extractor
        uses (`SeamPoint.normal_base` -> main, `normal_wall` -> secondary) and
        the one WeldPlanner assumes for both extractors.

        Decided by which part has a real boundary edge on the seam, which
        `_process_single_edge` has already established: a part with a matching
        boundary edge TERMINATES on the curve, so it supplies the wall; the
        part without one carries the seam across its face, so it is the base.
        That is the same relation the mesh extractor calls ownership.

        The centroid projection below is the fallback for the two cases the
        boundary test cannot separate - both parts terminating (an edge-to-edge
        joint) or neither (both curves synthetic). It used to decide EVERY
        case, and it decided them backwards: on a cube-on-plate joint it
        returned the cube's wall as main and the plate as secondary, the
        opposite of both this docstring's own stated intent and the mesh
        extractor's output for the identical scene.
        """
        if has_boundary_A != has_boundary_B:
            if has_boundary_A:
                # A terminates here, so A is the wall and B carries the base.
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

        # Vector from each part centroid to the seam centroid
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

        # The part whose average normal aligns MORE with its centroid-to-seam vector
        # has the seam on its outward face -> that is the base plate (main)
        dot_A = np.dot(avg_normal_A, vec_A)
        dot_B = np.dot(avg_normal_B, vec_B)

        logger.debug(f'Centroid-based main selection: dot_A={dot_A:.3f}, dot_B={dot_B:.3f}')

        if dot_A >= dot_B:
            return normals_A, normals_B
        else:
            return normals_B, normals_A

    def _evaluate_normal_at_point(self, point: np.ndarray, face: TopoDS_Shape) -> np.ndarray:
        """Evaluate unit surface normal at point using UV projection (fast path for planes)."""
        adaptor = BRepAdaptor_Surface(face)

        # Fast path for planes - normal is constant everywhere
        if adaptor.GetType() == GeomAbs_Plane:
            normal_dir = adaptor.Plane().Axis().Direction()

            # Respect face orientation (REVERSED means normal points inward)
            if face.Orientation() == TopAbs_REVERSED:
                normal_dir.Reverse()

            normal = np.array([normal_dir.X(), normal_dir.Y(), normal_dir.Z()])
            return normal / np.linalg.norm(normal)

        # General surfaces: project 3D point to UV parameter space, then evaluate normal
        pnt = gp_Pnt(point[0], point[1], point[2])
        surface = BRep_Tool.Surface(face)

        # Find UV coordinates of nearest point on surface
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

    def _detect_geometry(self, edge: TopoDS_Shape, points: np.ndarray) -> Dict:
        """Detect geometry type (line, arc, or polyline) and extract geometric parameters."""
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
            normal_gp = circle.Axis().Direction()

            return {
                'type': 'arc',
                'points': points,
                'center': np.array([center_gp.X(), center_gp.Y(), center_gp.Z()]),
                'radius': circle.Radius(),
                'normal': np.array([normal_gp.X(), normal_gp.Y(), normal_gp.Z()]),
                'is_closed': edge_adapted.IsClosed()
            }

        else:
            return {
                'type': 'polyline',
                'points': points,
                'curve_type': str(curve_type),
                'description': (
                    'Complex curve — emitted as a PtP segment carrying its '
                    'sampled points'
                )
            }

    def _wrap_in_seams(self,
                       geometry: Dict,
                       is_edge_joint: bool,
                       normals_main: np.ndarray,
                       normals_secondary: np.ndarray
                       ) -> List[Seam]:
        """Wrap geometry and normals into a Seam object with metadata.

        The segment type is the only thing that varies per branch; the five
        shared config keys are written once below, followed by whatever that
        type adds. An unrecognised type yields no seam rather than a guess.
        """
        points = geometry['points']
        kind = geometry['type']
        extra: Dict = {}

        if kind == 'line':
            seam = Seam(line_segment=LineSegment(
                start=geometry['start'], end=geometry['end']))

        elif kind == 'arc':
            seam = Seam(arc_segment=ArcSegment(
                points=points,
                center=geometry['center'],
                radius=geometry['radius'],
            ))
            extra = {
                'arc_normal': geometry['normal'],
                'is_closed': geometry['is_closed'],
            }

        elif kind == 'polyline':
            # A curve OCCT could not identify as a line or a circle - an
            # ellipse, spline, parabola. This used to be wrapped in a
            # LineSegment from first point to last, which claims the strongest
            # possible geometry for the curve least understood: `to_dict` then
            # exported only start/end, `length_m` became the CHORD, and on a
            # closed curve (an ellipse, the generic tilted circular feature)
            # start == end, so a 169mm weld exported as a zero-length line
            # with its geometry absent entirely.
            #
            # PtPSegment is what this case has needed: its `length()` sums the
            # polyline and `to_dict` exports the points themselves. It is also
            # the demotion the mesh extractor already makes for an unfittable
            # run - a false PTP is safe, a false line or arc is not.
            kind = 'ptp'
            seam = Seam(ptp_segment=PtPSegment(points=points))
            # `is_complex_curve` retired: it existed to flag exactly this case
            # from behind a LineSegment, and `segment_type` now says 'ptp'.
            extra = {'curve_type': geometry.get('curve_type', 'unknown')}

        else:
            logger.warning(
                f"Unrecognised geometry type '{kind}'; emitting no seam for "
                'this edge rather than guessing at its shape'
            )
            return []

        seam.config.update({
            'is_edge_joint': is_edge_joint,
            'geometry_type': kind,
            'normals_main': normals_main,
            'normals_secondary': normals_secondary,
            'smoothed_points': points,
            **extra,
        })
        return [seam]
