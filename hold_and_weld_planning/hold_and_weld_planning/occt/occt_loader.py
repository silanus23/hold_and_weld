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

"""OCCTLoader - Load CAD files (STEP/IGES) and convert to OCCT shapes.

Handles package:// URI resolution, applies world transforms, and provides
OCCT TopoDS_Shape objects for exact geometric seam extraction.
"""

from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.gp import gp_Trsf
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IGESControl import IGESControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.TopoDS import TopoDS_Shape
from scipy.spatial.transform import Rotation

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


class OCCTLoader:
    """Load CAD files and convert to OCCT shapes for weld planning pipeline.

    Handles package:// URI resolution and world pose transformation.
    Supports STEP (.step, .stp) and IGES (.iges, .igs) file formats.
    """

    def __init__(
        self,
        cad_path: str | Path,
        world_transform: NDArray = np.eye(4),
    ) -> None:
        """Initialize OCCT loader and build shape.

        Args:
            cad_path: Path to CAD file (supports package:// URIs)
            world_transform: Global pose matrix (4x4) to apply after loading

        Raises:
            ValueError: If file format unsupported or loading fails
            FileNotFoundError: If file doesn't exist
        """
        if world_transform.shape != (4, 4):
            raise ValueError(
                f'world_transform must be 4x4, got {world_transform.shape}'
            )

        self.world_transform = world_transform

        resolved_path = self._resolve_package_path(cad_path)

        try:
            shape = self._load_cad_file(resolved_path)
        except Exception as e:
            raise ValueError(f'Failed to load CAD file: {e}')

        self.shape = self._apply_transform(shape, world_transform)

    def _resolve_package_path(self, path_str: str | Path) -> Path:
        """Resolve package:// URI to absolute path.

        Args:
            path_str: Path string, either absolute or package:// URI

        Returns:
            Resolved absolute path as Path object

        Raises:
            FileNotFoundError: If package not found or file doesn't exist
            ValueError: If package path format is invalid
        """
        path_str = str(path_str)

        if path_str.startswith('package://'):
            if get_package_share_directory is None:
                raise ImportError(
                    'ament_index_python not available for package:// resolution'
                )

            without_prefix = path_str[len('package://'):]
            parts = without_prefix.split('/', 1)

            if len(parts) != 2:
                raise ValueError(f'Invalid package path: {path_str}')

            package_name = parts[0]
            relative_path = parts[1]

            try:
                package_dir = get_package_share_directory(package_name)
            except Exception as e:
                raise FileNotFoundError(f"Package '{package_name}' not found: {e}")

            resolved = Path(package_dir) / relative_path
        else:
            resolved = Path(path_str)

        if not resolved.exists():
            raise FileNotFoundError(f'File not found: {resolved}')

        return resolved

    def _load_cad_file(self, file_path: Path) -> TopoDS_Shape:
        """Load CAD file and return OCCT shape.

        Args:
            file_path: Path to STEP or IGES file

        Returns:
            TopoDS_Shape loaded from file

        Raises:
            ValueError: If file format unsupported or loading fails
        """
        suffix = file_path.suffix.lower()

        if suffix in ['.step', '.stp']:
            return self._load_step(file_path)
        elif suffix in ['.iges', '.igs']:
            return self._load_iges(file_path)
        else:
            raise ValueError(
                f'Unsupported file format: {suffix}. '
                'Supported formats: .step, .stp, .iges, .igs'
            )

    def _load_step(self, file_path: Path) -> TopoDS_Shape:
        """Load STEP file.

        Args:
            file_path: Path to STEP file

        Returns:
            TopoDS_Shape from STEP file

        Raises:
            RuntimeError: If STEP loading fails
        """
        reader = STEPControl_Reader()
        status = reader.ReadFile(str(file_path))

        if status != IFSelect_RetDone:
            raise RuntimeError(f'Failed to read STEP file: {file_path}')

        reader.TransferRoots()
        shape = reader.OneShape()

        if shape.IsNull():
            raise RuntimeError(f'STEP file contains no valid shapes: {file_path}')

        return shape

    def _load_iges(self, file_path: Path) -> TopoDS_Shape:
        """Load IGES file.

        Args:
            file_path: Path to IGES file

        Returns:
            TopoDS_Shape from IGES file

        Raises:
            RuntimeError: If IGES loading fails
        """
        reader = IGESControl_Reader()
        status = reader.ReadFile(str(file_path))

        if status != IFSelect_RetDone:
            raise RuntimeError(f'Failed to read IGES file: {file_path}')

        reader.TransferRoots()
        shape = reader.OneShape()

        if shape.IsNull():
            raise RuntimeError(f'IGES file contains no valid shapes: {file_path}')

        return shape

    def _apply_transform(
        self, shape: TopoDS_Shape, transform: NDArray
    ) -> TopoDS_Shape:
        """Apply 4x4 transformation matrix to OCCT shape.

        Args:
            shape: OCCT shape to transform
            transform: 4x4 homogeneous transformation matrix

        Returns:
            Transformed TopoDS_Shape
        """
        trsf = gp_Trsf()

        # Set transformation matrix (row-major)
        trsf.SetValues(
            transform[0, 0], transform[0, 1], transform[0, 2], transform[0, 3],
            transform[1, 0], transform[1, 1], transform[1, 2], transform[1, 3],
            transform[2, 0], transform[2, 1], transform[2, 2], transform[2, 3],
        )

        transformed_shape = BRepBuilderAPI_Transform(shape, trsf).Shape()

        return transformed_shape
