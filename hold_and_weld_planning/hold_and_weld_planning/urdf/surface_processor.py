# Copyright 2025 Berkan Tali
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

"""Surface processor - extracts and analyzes geometric surfaces from URDF."""

import numpy as np


class SurfaceProcessor:
    """Extract and analyze planar surfaces from URDF collision geometries."""

    def __init__(self, urdf_processor):
        """Initialize surface processor.

        Args:
        urdf_processor : URDFProcessor
            Instance used to retrieve link and transformation data.

        """
        self.urdf_processor = urdf_processor

    def extract_all_surfaces_from_links(self, link_names):
        """Extract all planar surfaces from multiple links.

        Args:
        link_names : list
            List of link names to process.

        Returns:
        list
            List of surface dictionaries from all links.

        """
        all_surfaces = []

        for link_name in link_names:
            surfaces = self.extract_all_surfaces(link_name)
            all_surfaces.extend(surfaces)

        return all_surfaces

    def extract_all_surfaces(self, link_name):
        """Extract all planar surfaces from a link's collision geometries.

        Args:
        link_name : str
            Name of the link to analyze.

        Returns:
        list
            List of surface dictionaries, each containing:
            - center: np.array [x, y, z] in world frame
            - normal: np.array [x, y, z] unit vector in world frame
            - u_axis: np.array [x, y, z] first bound direction
            - v_axis: np.array [x, y, z] second bound direction
            - area: surface area in m²
            - bounds: [dim1, dim2] dimensions along u_axis and v_axis
            - face_id: identifier string (e.g., 'link:0:top')

        Raises
        ValueError
            If link not found or geometry not supported.

        """
        link = self.urdf_processor.get_link(link_name)

        if not link.collisions and not link.collision:
            raise ValueError(f"Link '{link_name}' has no collision geometry")

        # Handle both single collision and multiple collisions
        collisions = link.collisions if link.collisions else [link.collision]

        all_surfaces = []

        for collision_idx, collision in enumerate(collisions):
            if collision is None:
                continue

            geometry = collision.geometry

            local_transform = self.urdf_processor.get_collision_transform(collision)
            world_transform = self.urdf_processor.world_transform @ local_transform

            if hasattr(geometry, 'size'):
                surfaces = self._extract_box_surfaces(
                    geometry, world_transform, link_name, collision_idx
                )
            elif hasattr(geometry, 'radius') and hasattr(geometry, 'length'):
                surfaces = self._extract_cylinder_surfaces(
                    geometry, world_transform, link_name, collision_idx
                )
            else:
                raise ValueError(
                    f"Link '{link_name}' collision {collision_idx}: "
                    f'unsupported geometry type (only box and cylinder supported)'
                )

            all_surfaces.extend(surfaces)

        return all_surfaces

    def find_surface_for_seam(self, link_name, seam_point, tolerance_m=0.05):
        """Find the surface closest to a seam point.

        Args:
        link_name : str
            Name of the link to search.
        seam_point : list
            [x, y, z] point on or near the seam.
        tolerance_m : float
            Maximum distance from point to surface (default 50mm).

        Returns
        dict
            Surface info with 'center' and 'normal' as lists.

        """
        seam_point = np.array(seam_point, dtype=float)
        surfaces = self.extract_all_surfaces(link_name)

        if not surfaces:
            raise ValueError(f"No surfaces found for link '{link_name}'")

        min_distance = float('inf')
        closest_surface = None

        for surface in surfaces:
            distance = self._point_to_plane_distance(
                seam_point, surface['center'], surface['normal']
            )

            if distance < min_distance:
                min_distance = distance
                closest_surface = surface

        if min_distance > tolerance_m:
            raise ValueError(
                f'No surface within {tolerance_m*1000:.1f}mm of seam point '
                f'(closest: {min_distance*1000:.1f}mm)'
            )

        return {
            'center': closest_surface['center'].tolist(),
            'normal': closest_surface['normal'].tolist()
        }

    def find_touching_pairs(self, main_surfaces, extra_surfaces,
                            distance_tol_m=0.015, angle_tol_deg=5.0):
        """Find pairs of surfaces that are touching between two parts.

        Args:
        main_surfaces : list
            List of surfaces from main part.
        extra_surfaces : list
            List of surfaces from extra part.
        distance_tol_m : float
            Maximum distance between planes (default 10mm).
        angle_tol_deg : float
            Maximum angle difference for parallel normals (default 5°).

        Returns:
        list
            List of touching pairs, each is (main_surf, extra_surf) tuple.

        """
        touching_pairs = []
        cos_tol = np.cos(np.radians(angle_tol_deg))

        for main_surf in main_surfaces:
            for extra_surf in extra_surfaces:
                dot = np.dot(main_surf['normal'], extra_surf['normal'])

                # Normals must be anti-parallel (facing each other), not parallel
                if dot > -cos_tol:
                    continue  # Normals not facing each other

                vec_between = extra_surf['center'] - main_surf['center']
                distance = abs(np.dot(vec_between, main_surf['normal']))

                if distance <= distance_tol_m:
                    touching_pairs.append((main_surf, extra_surf))

        return touching_pairs

    def find_closest_pair_to_seam(self, touching_pairs, seam_line, tolerance_m=0.05):
        """Find which touching surface pair is closest to the weld seam.

        Args:
        touching_pairs : list
            List of (main_surf, extra_surf) tuples.
        seam_line : LineSegment
            LineSegment object representing the weld seam.
        tolerance_m : float
            Maximum distance from seam to surface (default 50mm).

        Returns:
        tuple
            (main_surf, extra_surf) closest to seam.

        Raises
        ValueError
            If no touching pair is within tolerance of seam.

        """
        if not touching_pairs:
            raise ValueError('No touching surface pairs found')

        min_distance = float('inf')
        closest_pair = None

        for main_surf, extra_surf in touching_pairs:
            dist_start = self._point_to_plane_distance(
                seam_line.start, main_surf['center'], main_surf['normal']
            )
            dist_end = self._point_to_plane_distance(
                seam_line.end, main_surf['center'], main_surf['normal']
            )

            avg_distance = (dist_start + dist_end) / 2.0

            if avg_distance < min_distance:
                min_distance = avg_distance
                closest_pair = (main_surf, extra_surf)

        if min_distance > tolerance_m:
            raise ValueError(
                f'No touching surface within {tolerance_m*1000:.1f}mm of seam '
                f'(closest: {min_distance*1000:.1f}mm)'
            )

        return closest_pair

    def get_surface_corners(self, surface):
        """Get the 4 corner points of a rectangular surface.

        Args:
        surface : dict
            Surface dictionary with center, u_axis, v_axis, bounds.

        Returns:
        list
            4 corner points as np.arrays.

        """
        center = surface['center']
        u_axis = surface['u_axis']
        v_axis = surface['v_axis']
        half_u = surface['bounds'][0] / 2.0
        half_v = surface['bounds'][1] / 2.0

        corners = [
            center + half_u * u_axis + half_v * v_axis,
            center + half_u * u_axis - half_v * v_axis,
            center - half_u * u_axis + half_v * v_axis,
            center - half_u * u_axis - half_v * v_axis,
        ]

        return corners

    def project_point_to_surface(self, point, surface):
        """Project a point onto a surface plane and get UV coordinates.

        Args:
        ----
        point : list
            [x, y, z] point to project.
        surface : dict
            Surface dictionary.

        Returns:
        -------
        tuple
            (u_coord, v_coord) in surface local coordinates.

        """
        point = np.array(point)
        center = surface['center']
        u_axis = surface['u_axis']
        v_axis = surface['v_axis']

        vec = point - center
        u_coord = np.dot(vec, u_axis)
        v_coord = np.dot(vec, v_axis)

        return u_coord, v_coord

    def check_containment(self, inner_surface, outer_surface, tolerance_m=0.001):
        """Check if inner_surface is completely contained within outer_surface.

        Args:
        inner_surface : dict
            Surface that might be inside.
        outer_surface : dict
            Surface that might contain the other.
        tolerance_m : float
            Edge tolerance (default 1mm).

        Returns:
        bool
            True if inner_surface fits completely inside outer_surface.

        """
        inner_corners = self.get_surface_corners(inner_surface)
        outer_half_u = outer_surface['bounds'][0] / 2.0
        outer_half_v = outer_surface['bounds'][1] / 2.0

        for corner in inner_corners:
            u, v = self.project_point_to_surface(corner, outer_surface)
            if abs(u) > outer_half_u + tolerance_m:
                return False
            if abs(v) > outer_half_v + tolerance_m:
                return False

        return True

    def check_edge_alignment(self, surface_a, surface_b, tolerance_m=0.01):
        """Check if surface edges align or one extends past the other.

        Args:
        surface_a : dict
            First surface.
        surface_b : dict
            Second surface.
        tolerance_m : float
            Edge alignment tolerance (default 10mm).

        Returns:
        str
            'aligned' or 'partial'.

        """
        corners_a = self.get_surface_corners(surface_a)
        corners_b = self.get_surface_corners(surface_b)

        half_u_b = surface_b['bounds'][0] / 2.0
        half_v_b = surface_b['bounds'][1] / 2.0

        a_on_b_coords = []
        for corner in corners_a:
            u, v = self.project_point_to_surface(corner, surface_b)
            a_on_b_coords.append((u, v))

        reaches_u_edge = any(abs(u) >= half_u_b - tolerance_m for u, v in a_on_b_coords)
        reaches_v_edge = any(abs(v) >= half_v_b - tolerance_m for u, v in a_on_b_coords)

        half_u_a = surface_a['bounds'][0] / 2.0
        half_v_a = surface_a['bounds'][1] / 2.0

        b_on_a_coords = []
        for corner in corners_b:
            u, v = self.project_point_to_surface(corner, surface_a)
            b_on_a_coords.append((u, v))

        b_reaches_u_edge = any(abs(u) >= half_u_a - tolerance_m for u, v in b_on_a_coords)
        b_reaches_v_edge = any(abs(v) >= half_v_a - tolerance_m for u, v in b_on_a_coords)

        if reaches_u_edge or reaches_v_edge or b_reaches_u_edge or b_reaches_v_edge:
            return 'aligned'
        return 'partial'

    def get_overlap_surface(self, main_surface, extra_surface):
        """Compute the overlapping region between two touching surfaces.

        Args:
        ----
        main_surface : dict
            Surface dict from main part.
        extra_surface : dict
            Surface dict from extra part.

        Returns:
        -------
        dict
            Overlap surface dictionary.

        Raises:
        ------
        ValueError
            If surfaces don't overlap.

        """
        extra_corners = self.get_surface_corners(extra_surface)

        u_coords = []
        v_coords = []
        for corner in extra_corners:
            u, v = self.project_point_to_surface(corner, main_surface)
            u_coords.append(u)
            v_coords.append(v)

        extra_u_min, extra_u_max = min(u_coords), max(u_coords)
        extra_v_min, extra_v_max = min(v_coords), max(v_coords)

        main_half_u = main_surface['bounds'][0] / 2.0
        main_half_v = main_surface['bounds'][1] / 2.0

        overlap_u_min = max(extra_u_min, -main_half_u)
        overlap_u_max = min(extra_u_max, main_half_u)
        overlap_v_min = max(extra_v_min, -main_half_v)
        overlap_v_max = min(extra_v_max, main_half_v)

        if overlap_u_min >= overlap_u_max or overlap_v_min >= overlap_v_max:
            raise ValueError('Surfaces do not overlap')

        overlap_dim_u = overlap_u_max - overlap_u_min
        overlap_dim_v = overlap_v_max - overlap_v_min
        overlap_center_u = (overlap_u_min + overlap_u_max) / 2.0
        overlap_center_v = (overlap_v_min + overlap_v_max) / 2.0

        overlap_center = (
            main_surface['center']
            + overlap_center_u * main_surface['u_axis']
            + overlap_center_v * main_surface['v_axis']
        )

        return {
            'center': overlap_center,
            'normal': main_surface['normal'].copy(),
            'u_axis': main_surface['u_axis'].copy(),
            'v_axis': main_surface['v_axis'].copy(),
            'bounds': [overlap_dim_u, overlap_dim_v],
            'area': overlap_dim_u * overlap_dim_v
        }

    def determine_joint_type(self, main_surface, extra_surface):
        """Determine joint type based on surface containment.

        Args:
            main_surface: Surface dict from main part.
            extra_surface: Surface dict from extra part.

        Returns:
            't_joint', 'corner_joint', or 'lap_joint'.
        """
        extra_in_main = self.check_containment(extra_surface, main_surface)
        if extra_in_main:
            return 't_joint'

        main_in_extra = self.check_containment(main_surface, extra_surface)
        if main_in_extra:
            return 'corner_joint'

        alignment = self.check_edge_alignment(main_surface, extra_surface)
        if alignment == 'aligned':
            return 'corner_joint'
        return 'lap_joint'

    def detect_joint_type(self, main_link, extra_link, seam_point,
                          extra_processor=None, tolerance_m=0.05):
        """Auto-detect joint type based on geometry analysis.

        Args:
            main_link: Link name in main part.
            extra_link: Link name in extra part.
            seam_point: [x, y, z] point on or near the seam.
            extra_processor: SurfaceProcessor for extra part.
            tolerance_m: Distance tolerance for finding surfaces.

        Returns:
            Detected joint type.
        """
        if extra_processor is None:
            extra_processor = self

        main_surfaces = self.extract_all_surfaces(main_link)
        extra_surfaces = extra_processor.extract_all_surfaces(extra_link)
        touching_pairs = self.find_touching_pairs(main_surfaces, extra_surfaces)

        if not touching_pairs:
            seam_point_arr = np.array(seam_point, dtype=float)
            main_surface = self._find_closest_surface(main_surfaces, seam_point_arr)
            extra_surface = self._find_closest_surface(extra_surfaces, seam_point_arr)

            if main_surface is None or extra_surface is None:
                return 't_joint'

            return self.determine_joint_type(main_surface, extra_surface)

        from ..core.line_segment import LineSegment
        seam_point_arr = np.array(seam_point, dtype=float)
        dummy_segment = LineSegment(seam_point_arr, seam_point_arr + np.array([0.001, 0, 0]))

        try:
            main_surf, extra_surf = self.find_closest_pair_to_seam(
                touching_pairs, dummy_segment, tolerance_m
            )
        except ValueError:
            main_surf, extra_surf = touching_pairs[0]

        return self.determine_joint_type(main_surf, extra_surf)

    def _point_to_plane_distance(self, point, plane_center, plane_normal):
        """Calculate perpendicular distance from point to plane."""
        vec = np.array(point) - np.array(plane_center)
        return abs(np.dot(vec, plane_normal))

    def _find_closest_surface(self, surfaces, point):
        """Find the closest surface to a point."""
        if not surfaces:
            return None

        min_distance = float('inf')
        closest = None

        for surface in surfaces:
            distance = self._point_to_plane_distance(
                point, surface['center'], surface['normal']
            )
            if distance < min_distance:
                min_distance = distance
                closest = surface

        return closest

    def _extract_box_surfaces(self, geometry, world_transform, link_name, collision_idx):
        """Extract 6 planar surfaces from a box geometry.

        Args:
        geometry : Box
            Box geometry with 'size' attribute.
        world_transform : np.ndarray
            4x4 transformation matrix to world frame.
        link_name : str
            Name of the parent link.
        collision_idx : int
            Index of collision within link.

        Returns:
        list
            List of 6 surface dictionaries.

        """
        l_dim, w_dim, h_dim = geometry.size

        local_faces = {
            'top': {
                'offset': [0, 0, h_dim/2], 'normal': [0, 0, 1],
                'u_axis': [1, 0, 0], 'v_axis': [0, 1, 0], 'dims': [l_dim, w_dim]
            },
            'bottom': {
                'offset': [0, 0, -h_dim/2], 'normal': [0, 0, -1],
                'u_axis': [1, 0, 0], 'v_axis': [0, -1, 0], 'dims': [l_dim, w_dim]
            },
            'front': {
                'offset': [l_dim/2, 0, 0], 'normal': [1, 0, 0],
                'u_axis': [0, 1, 0], 'v_axis': [0, 0, 1], 'dims': [w_dim, h_dim]
            },
            'back': {
                'offset': [-l_dim/2, 0, 0], 'normal': [-1, 0, 0],
                'u_axis': [0, -1, 0], 'v_axis': [0, 0, 1], 'dims': [w_dim, h_dim]
            },
            'right': {
                'offset': [0, w_dim/2, 0], 'normal': [0, 1, 0],
                'u_axis': [1, 0, 0], 'v_axis': [0, 0, 1], 'dims': [l_dim, h_dim]
            },
            'left': {
                'offset': [0, -w_dim/2, 0], 'normal': [0, -1, 0],
                'u_axis': [-1, 0, 0], 'v_axis': [0, 0, 1], 'dims': [l_dim, h_dim]
            },
        }

        return self._transform_faces_to_world(
            local_faces, world_transform, link_name, collision_idx
        )

    def _extract_cylinder_surfaces(self, geometry, world_transform, link_name, collision_idx):
        """Extract 2 planar cap surfaces from a cylinder geometry.

        Args:
        geometry : Cylinder
            Cylinder geometry with radius and length.
        world_transform : np.ndarray
            4x4 transformation matrix to world frame.
        link_name : str
            Name of the parent link.
        collision_idx : int
            Index of collision within link.

        Returns:
        list
            List of 2 surface dictionaries.

        """
        radius = geometry.radius
        length = geometry.length
        cap_area = np.pi * radius * radius
        diameter = radius * 2

        local_faces = {
            'cap_top': {
                'offset': [0, 0, length/2], 'normal': [0, 0, 1],
                'u_axis': [1, 0, 0], 'v_axis': [0, 1, 0], 'dims': [diameter, diameter]
            },
            'cap_bottom': {
                'offset': [0, 0, -length/2], 'normal': [0, 0, -1],
                'u_axis': [1, 0, 0], 'v_axis': [0, -1, 0], 'dims': [diameter, diameter]
            },
        }

        surfaces = self._transform_faces_to_world(
            local_faces, world_transform, link_name, collision_idx
        )

        for surface in surfaces:
            surface['area'] = cap_area
            surface['is_circular'] = True

        return surfaces

    def _transform_faces_to_world(self, local_faces, world_transform, link_name, collision_idx):
        """Transform local face data to world frame surfaces.

        Args:
        local_faces : dict
            Dict of face data with offsets and normals.
        world_transform : np.ndarray
            4x4 transformation matrix.
        link_name : str
            Name of the parent link.
        collision_idx : int
            Index of collision within link.

        Returns:
        list
            List of surface dictionaries in world frame.

        """
        surfaces = []
        rot_mat = world_transform[:3, :3]

        for face_name, face in local_faces.items():
            local_center = np.array(face['offset'])
            local_center_homogeneous = np.append(local_center, 1.0)
            world_center = (world_transform @ local_center_homogeneous)[:3]

            world_normal = rot_mat @ np.array(face['normal'])
            world_normal = world_normal / np.linalg.norm(world_normal)

            world_u = rot_mat @ np.array(face['u_axis'])
            world_u = world_u / np.linalg.norm(world_u)

            world_v = rot_mat @ np.array(face['v_axis'])
            world_v = world_v / np.linalg.norm(world_v)

            area = face['dims'][0] * face['dims'][1]
            face_id = f'{link_name}:{collision_idx}:{face_name}'

            surfaces.append({
                'center': world_center,
                'normal': world_normal,
                'u_axis': world_u,
                'v_axis': world_v,
                'area': area,
                'bounds': face['dims'],
                'face_id': face_id,
                'is_circular': False
            })

        return surfaces
