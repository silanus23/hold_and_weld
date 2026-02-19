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

from pathlib import Path

import manifold3d
import numpy as np
from numpy.typing import NDArray
import trimesh


class MeshLoader:
    """Load STL mesh files and convert to manifold for weld planning pipeline.

    Handles package:// URI resolution, mesh refinement to increase vertex
    density, and conversion to manifold3d format for boolean operations.
    """

    def __init__(
        self,
        mesh_path: str | Path,
        world_transform: NDArray = np.eye(4),
        refine_iterations: int = 32,
    ) -> None:
        """Initialize mesh loader and build manifold.

        Args:
            mesh_path: Path to mesh file (supports package:// URIs)
            world_transform: Global pose matrix (4x4) to apply after loading
            refine_iterations: Number of mesh subdivision iterations (default: 32)
        Raises:
            ValueError: If mesh loading or conversion fails
            FileNotFoundError: If file doesn't exist
        """
        if world_transform.shape != (4, 4):
            raise ValueError(
                f'world_transform must be 4x4, got {world_transform.shape}'
            )

        self.world_transform = world_transform
        self.refine_iterations = refine_iterations

        resolved_path = self._resolve_package_path(mesh_path)

        try:
            mesh = trimesh.load(resolved_path)
        except Exception as e:
            raise ValueError(f'Failed to load mesh: {e}')

        self.manifold = self._build_manifold(mesh)

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
            without_prefix = path_str[len('package://'):]
            parts = without_prefix.split('/', 1)

            if len(parts) != 2:
                raise ValueError(f'Invalid package path: {path_str}')

            package_name = parts[0]
            relative_path = parts[1]

            try:
                from ament_index_python.packages import get_package_share_directory

                package_dir = get_package_share_directory(package_name)
            except Exception as e:
                raise FileNotFoundError(f"Package '{package_name}' not found: {e}")

            resolved = Path(package_dir) / relative_path
        else:
            resolved = Path(path_str)

        if not resolved.exists():
            raise FileNotFoundError(f'File not found: {resolved}')

        return resolved

    def _build_manifold(self, mesh: trimesh.Trimesh) -> manifold3d.Manifold:
        """Convert trimesh to manifold, refine, and apply world transform.

        Args:
            mesh: Loaded trimesh object
        Returns:
            Transformed and refined manifold
        Raises:
            ValueError: If conversion fails
        """
        try:
            manifold_obj = manifold3d.Manifold(
                manifold3d.Mesh(
                    vert_properties=np.array(mesh.vertices, dtype=np.float64),
                    tri_verts=np.array(mesh.faces, dtype=np.int32),
                )
            )
        except Exception as e:
            raise ValueError(f'Failed to convert mesh to manifold: {e}')

        # Subdivide to increase vertex density
        if self.refine_iterations > 0:
            manifold_obj = manifold_obj.refine(self.refine_iterations)

        mat_3x4 = self.world_transform[:3, :].tolist()
        manifold_obj = manifold_obj.transform(mat_3x4)

        return manifold_obj
