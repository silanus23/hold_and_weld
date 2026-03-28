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

import logging
from pathlib import Path
from typing import Any, List

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
from urdf_parser_py.urdf import URDF

logger = logging.getLogger(__name__)


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
        logger.info(f'Loading URDF from: {resolved_path}')
        
        urdf_string = self._process_xacro(resolved_path)

        try:
            self.robot = URDF.from_xml_string(urdf_string)
            logger.info(f'Parsed URDF: {self.robot.name}')
        except Exception as e:
            logger.error(f'Failed to parse URDF: {e}')
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

        Raises:
            ValueError: If xyz or rpy don't have exactly 3 elements.
        """
        xyz_array = np.array(xyz, dtype=float)
        rpy_array = np.array(rpy, dtype=float)

        if len(xyz_array) != 3:
            raise ValueError(f'xyz must have 3 elements, got {len(xyz_array)}')
        if len(rpy_array) != 3:
            raise ValueError(f'rpy must have 3 elements, got {len(rpy_array)}')

        self.world_xyz = xyz_array
        self.world_rpy = rpy_array
        self.world_transform = self._build_transform_matrix(xyz_array, rpy_array)
        
        logger.debug(f'Set world pose: xyz={xyz_array}, rpy={rpy_array}')

    def get_collision_transform(self, collision: Any) -> NDArray:
        """Get transformation matrix from collision geometry origin.

        Extracts xyz/rpy from collision.origin and builds the local transform
        for the collision geometry relative to its link frame.

        Args:
            collision: Collision object from URDF link.

        Returns:
            4x4 homogeneous transformation matrix.
        """
        if collision.origin is not None:
            xyz = collision.origin.xyz if collision.origin.xyz else [0, 0, 0]
            rpy = collision.origin.rpy if collision.origin.rpy else [0, 0, 0]
        else:
            xyz = [0, 0, 0]
            rpy = [0, 0, 0]

        return self._build_transform_matrix(xyz, rpy)

    def _resolve_package_path(self, path_str: str | Path) -> Path:
        """Resolve package:// URI to absolute path."""
        path_str = str(path_str)

        if path_str.startswith('package://'):
            logger.debug(f'Resolving package path: {path_str}')
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
                logger.debug(f'Resolved package {package_name} to: {package_dir}')
            except ModuleNotFoundError:
                raise FileNotFoundError(
                    f"ament_index_python not installed - cannot resolve package://"
                )
            except Exception as e:
                raise FileNotFoundError(f"Package '{package_name}' not found: {e}")

            resolved = Path(package_dir) / relative_path

        else:
            resolved = Path(path_str)

        if not resolved.exists():
            raise FileNotFoundError(f'File not found: {resolved}')

        return resolved

    def _process_xacro(self, xacro_path: Path) -> str:
        """Process file through xacro (handles both .xacro and .urdf with macros)."""
        logger.debug(f'Processing file through xacro: {xacro_path}')
        try:
            import xacro

            doc = xacro.process_file(str(xacro_path))
            return doc.toxml()
        except ModuleNotFoundError:
            raise ValueError(
                f"xacro module not installed - cannot process URDF/xacro files"
            )
        except Exception as e:
            logger.error(f'Xacro processing failed: {e}')
            raise ValueError(f"Failed to process xacro '{xacro_path}': {e}")

    def _build_transform_matrix(
        self, xyz: List[float] | NDArray, rpy: List[float] | NDArray
    ) -> NDArray:
        """Build 4x4 homogeneous transform from xyz position and rpy rotation."""
        xyz_array = np.array(xyz, dtype=float)
        rpy_array = np.array(rpy, dtype=float)

        if len(xyz_array) != 3:
            raise ValueError(f'xyz must have 3 elements, got {len(xyz_array)}')
        if len(rpy_array) != 3:
            raise ValueError(f'rpy must have 3 elements, got {len(rpy_array)}')

        R = Rotation.from_euler('xyz', rpy_array).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz_array
        return T
