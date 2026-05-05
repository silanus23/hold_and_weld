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

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Common
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCC.Core.BRepExtrema import BRepExtrema_DistShapeShape
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.GeomAbs import GeomAbs_Circle, GeomAbs_Line, GeomAbs_Plane
from OCC.Core.GeomLProp import GeomLProp_SLProps
from OCC.Core.gp import gp_Pnt
from OCC.Core.GProp import GProp_GProps
from OCC.Core.ShapeAnalysis import ShapeAnalysis_Surface
from OCC.Core.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED, TopAbs_WIRE
from OCC.Core.TopExp import topexp, TopExp_Explorer
from OCC.Core.TopoDS import topods, TopoDS_Shape
from OCC.Core.TopTools import (
    TopTools_IndexedDataMapOfShapeListOfShape,
    TopTools_ListIteratorOfListOfShape,
)

from ..core.arc_segment import ArcSegment
from ..core.line_segment import LineSegment
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
                    - tolerance: Distance tolerance for contact detection (default 1e-3)
        """
        if shape_1.IsNull():
            raise ValueError('shape_1 is null')
        if shape_2.IsNull():
            raise ValueError('shape_2 is null')
        self.shape_1 = shape_1
        self.shape_2 = shape_2
        self.params = params

        self.num_smooth_points = params.get('num_smooth_points', 100)
        self.tolerance = params.get('tolerance', 1e-3)

        # Centroids for geometry-based main/secondary determination
        self.centroid_1 = self._compute_shape_centroid(shape_1)
        self.centroid_2 = self._compute_shape_centroid(shape_2)

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
                logger.debug(f'Processed edge {idx + 1}/{len(intersection_data)}: {len(edge_seams)} seam(s)')

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
            normals_A, normals_B, points
        )

        is_edge_joint = (boundary_A is not None) and (boundary_B is not None)

        geometry = self._detect_geometry(edge, points)
        edge_seams = self._wrap_in_seams(geometry, is_edge_joint, normals_main, normals_secondary)

        return edge_seams

    def _find_contact_face_pairs(self) -> List[Dict]:
        """Find all face pairs within tolerance using BRepExtrema_DistShapeShape proximity check."""
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

        for face_A in faces_1:
            for face_B in faces_2:
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


        return contact_candidates

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

        return intersection_data

    def _get_matching_boundary_edge(self, seam_edge: TopoDS_Shape,
                                    face: TopoDS_Shape
                                    ) -> TopoDS_Shape:
        """Get boundary edge from face that geometrically matches seam edge, or None if synthetic."""
        edge_exp = TopExp_Explorer(face, TopAbs_EDGE)

        while edge_exp.More():
            boundary_edge = topods.Edge(edge_exp.Current())

            if self._edges_are_geometrically_aligned(seam_edge, boundary_edge):
                return boundary_edge

            edge_exp.Next()

        return None

    def _edges_are_geometrically_aligned(self, edge_1: TopoDS_Shape, edge_2: TopoDS_Shape) -> bool:
        """Check if two edges lie on the same geometric curve within tolerance."""
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

                return (dist_start <= self.tolerance and
                        dist_end <= self.tolerance)

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
            logger.debug(f'Failed to check edge alignment: {e}')
            return False

    def _get_wall_surface_at_edge(self,
                                  boundary_edge: TopoDS_Shape,
                                  shape: TopoDS_Shape,
                                  kissing_face: TopoDS_Shape
                                  ) -> TopoDS_Shape:
        """Get perpendicular wall surface at boundary edge (excludes kissing face)."""
        edge_face_map = TopTools_IndexedDataMapOfShapeListOfShape()
        topexp.MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_face_map)

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

    # def _get_neighbor_faces(self, shape: TopoDS_Shape, target_face: TopoDS_Shape) -> List[TopoDS_Shape]:
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

    # def _get_pipe_normals(self, points: np.ndarray, surfaces: Dict) -> Tuple[np.ndarray, np.ndarray]:
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
        """Extract normals at each point on a surface."""
        normals = []

        for point in points:
            try:
                normal = self._evaluate_normal_at_point(point, surface)
                normals.append(normal)
            except Exception as e:
                if normals:
                    normals.append(normals[-1])
                    logger.debug(f'Failed to get normal from surface, using previous normal as fallback: {e}')
                else:
                    normals.append(np.array([0, 0, 1]))
                    logger.warning(f'Failed to get normal for first point on surface, using hardcoded fallback [0,0,1]: {e}')

        normals = np.array(normals)
        normals = self._make_normals_consistent(normals)

        return normals

    def _make_normals_consistent(self, normals: np.ndarray) -> np.ndarray:
        """Ensure all normals point in consistent direction by flipping those opposite to average."""
        if len(normals) == 0:
            return normals

        avg_normal = np.mean(normals, axis=0)
        norm = np.linalg.norm(avg_normal)

        if norm < 1e-10:
            return normals

        avg_normal = avg_normal / norm

        fixed_normals = []
        for normal in normals:
            if np.dot(normal, avg_normal) < 0:
                fixed_normals.append(-normal)
            else:
                fixed_normals.append(normal)

        return np.array(fixed_normals)

    def _compute_shape_centroid(self, shape: TopoDS_Shape) -> np.ndarray:
        """Compute the volumetric centroid of an OCCT shape."""
        props = GProp_GProps()
        brepgprop.VolumeProperties(shape, props)
        cog = props.CentreOfMass()
        return np.array([cog.X(), cog.Y(), cog.Z()])

    def _determine_main_secondary_normals(self,
                                          normals_A: np.ndarray,
                                          normals_B: np.ndarray,
                                          seam_points: np.ndarray
                                          ) -> Tuple[np.ndarray, np.ndarray]:
        """Determine which normals are main vs secondary based on part centroids.

        The main surface is the base plate — the one whose normals point *away* from its
        own part centroid relative to the seam. Computed by projecting the centroid-to-seam
        vector onto each part's average normal: the higher dot product means the seam is
        on the outward-facing side of that part → that part is the base (main).
        """
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
        # has the seam on its outward face → that is the base plate (main)
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
            # Respect face orientation
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
                'description': 'Complex curve (polyline) — use smoothed_points for waypoint planning'
            }

    def _wrap_in_seams(self,
                       geometry: Dict,
                       is_edge_joint: bool,
                       normals_main: np.ndarray,
                       normals_secondary: np.ndarray
                       ) -> List[Seam]:
        """Wrap geometry and normals into Seam object with metadata."""
        seams = []
        points = geometry['points']

        if geometry['type'] == 'line':
            line_segment = LineSegment(start=geometry['start'], end=geometry['end'])
            seam = Seam(line_segment=line_segment)

            seam.config['is_edge_joint'] = is_edge_joint
            seam.config['geometry_type'] = 'line'
            seam.config['normals_main'] = normals_main
            seam.config['normals_secondary'] = normals_secondary
            seam.config['smoothed_points'] = points

            seams.append(seam)

        elif geometry['type'] == 'arc':
            arc_segment = ArcSegment(
                points=points,
                center=geometry['center'],
                radius=geometry['radius']
            )
            seam = Seam(arc_segment=arc_segment)

            seam.config['is_edge_joint'] = is_edge_joint
            seam.config['geometry_type'] = 'arc'
            seam.config['normals_main'] = normals_main
            seam.config['normals_secondary'] = normals_secondary
            seam.config['smoothed_points'] = points
            seam.config['arc_normal'] = geometry['normal']
            seam.config['is_closed'] = geometry['is_closed']

            seams.append(seam)

        elif geometry['type'] == 'polyline':
            line_segment = LineSegment(start=points[0], end=points[-1])
            seam = Seam(line_segment=line_segment)

            seam.config['is_edge_joint'] = is_edge_joint
            seam.config['geometry_type'] = 'polyline'
            seam.config['normals_main'] = normals_main
            seam.config['normals_secondary'] = normals_secondary
            seam.config['smoothed_points'] = points
            seam.config['is_complex_curve'] = True
            seam.config['curve_type'] = geometry.get('curve_type', 'unknown')

            seams.append(seam)

        return seams
