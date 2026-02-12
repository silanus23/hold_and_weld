
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

"""URDF processor - handles package paths, xacro processing, and URDF parsing.

This module provides utilities for loading URDF/xacro files, resolving ROS package
paths, and managing coordinate frame transformations for workpiece positioning.
"""

from pathlib import Path
from typing import Any, List

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
from urdf_parser_py.urdf import URDF


class URDFProcessor:
    """Process URDF and xacro files.

    Handles package:// path resolution, xacro processing, and world pose
    transformations.
    """

    def __init__(self, urdf_path: str | Path) -> None:
        """Initialize URDF processor with URDF or xacro file.

        Args:
            urdf_path: Path to URDF or xacro file (accepts package:// URIs).

        Raises:
            FileNotFoundError: If file doesn't exist.
            ValueError: If URDF/xacro parsing fails.
        """
        resolved_path = self._resolve_package_path(urdf_path)
        urdf_string = self._process_xacro(resolved_path)

        try:
            self.robot = URDF.from_xml_string(urdf_string)
        except Exception as e:
            raise ValueError(f'Failed to parse URDF: {e}')

        self.world_xyz = np.array([0.0, 0.0, 0.0])
        self.world_rpy = np.array([0.0, 0.0, 0.0])
        self.world_transform = np.eye(4)

    def set_world_pose(
        self, xyz: List[float] | NDArray, rpy: List[float] | NDArray
    ) -> None:
        """Set world pose of the workpiece.

        Args:
            xyz: [x, y, z] position in meters.
            rpy: [roll, pitch, yaw] rotation in radians.
        """
        self.world_xyz = np.array(xyz, dtype=float)
        self.world_rpy = np.array(rpy, dtype=float)
        self.world_transform = self._build_transform_matrix(xyz, rpy)

    def get_link(self, link_name: str) -> Any:
        """Get link by name from the URDF model.

        Args:
            link_name: Name of the link to retrieve.

        Returns:
            Link object from URDF.

        Raises:
            ValueError: If link not found in URDF.
        """
        for link in self.robot.links:
            if link.name == link_name:
                return link
        raise ValueError(f"Link '{link_name}' not found in URDF")

    def get_collision_transform(self, collision: Any) -> NDArray:
        """Get transformation matrix from collision origin.

        Args:
            collision: Collision object from URDF.

        Returns:
            4x4 transformation matrix (homogeneous transform).
        """
        if collision.origin is not None:
            xyz = collision.origin.xyz if collision.origin.xyz else [0, 0, 0]
            rpy = collision.origin.rpy if collision.origin.rpy else [0, 0, 0]
        else:
            xyz = [0, 0, 0]
            rpy = [0, 0, 0]

        return self._build_transform_matrix(xyz, rpy)

    def _resolve_package_path(self, path_str: str | Path) -> Path:
        """Resolve package:// URI to absolute path.

        Args:
            path_str: Path string, either absolute or package:// URI.

        Returns:
            Resolved absolute path as Path object.

        Raises:
            FileNotFoundError: If package not found or file doesn't exist.
            ValueError: If package path format is invalid.
        """
        path_str = str(path_str)

        if path_str.startswith('package://'):
            without_prefix = path_str[len('package://'):]
            parts = without_prefix.split('/', 1)

            if len(parts) != 2:
                raise ValueError(f'Invalid package path: {path_str}')

            package_name = parts[0]
            relative_path = parts[1]

            try:
                from ament_index_python.packages import (
                    get_package_share_directory,
                )
                package_dir = get_package_share_directory(package_name)
            except Exception as e:
                raise FileNotFoundError(
                    f"Package '{package_name}' not found: {e}"
                )

            resolved = Path(package_dir) / relative_path

        else:
            resolved = Path(path_str)

        if not resolved.exists():
            raise FileNotFoundError(f'File not found: {resolved}')

        return resolved

    def _process_xacro(self, xacro_path: Path) -> str:
        """Process xacro file to URDF string.

        Args:
            xacro_path: Path to xacro file.

        Returns:
            URDF XML string.

        Raises:
            ValueError: If xacro processing fails.
        """
        try:
            import xacro
            doc = xacro.process_file(str(xacro_path))
            return doc.toxml()
        except Exception as e:
            raise ValueError(f"Failed to process xacro '{xacro_path}': {e}")

    def _build_transform_matrix(
        self, xyz: List[float] | NDArray, rpy: List[float] | NDArray
    ) -> NDArray:
        """Build 4x4 transformation matrix from position and orientation.

        Args:
            xyz: [x, y, z] position vector.
            rpy: [roll, pitch, yaw] orientation in radians.

        Returns:
            4x4 homogeneous transformation matrix.
        """
        R = Rotation.from_euler('xyz', rpy).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz
        return T
