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

"""OCCTGenerator - Convert URDF collision geometry to OCCT shapes.

Uses pythonocc-core to create exact geometric representations from URDF
primitives (Box, Cylinder, Sphere). Provides deterministic geometry for
precise seam extraction without mesh approximation.
"""

from typing import Any

import numpy as np
from numpy.typing import NDArray
from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepPrimAPI import (
    BRepPrimAPI_MakeBox,
    BRepPrimAPI_MakeCylinder,
    BRepPrimAPI_MakeSphere,
)
from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf
from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Core.TopoDS import TopoDS_Shape
from scipy.spatial.transform import Rotation
from urdf_parser_py.urdf import Box, Cylinder, Mesh, Sphere


class OCCTGenerator:
    """Generate OCCT shapes from URDF collision geometry.

    Converts URDF primitives to exact OCCT geometric representations,
    applies transformations, and fuses into single compound shape.
    """

    def __init__(
        self,
        robot_object: Any,
        world_transform: NDArray = np.eye(4),
    ) -> None:
        """Initialize OCCT generator.

        Args:
            robot_object: The self.robot object from URDFProcessor
            world_transform: Global starting pose matrix (4x4)

        Raises:
            ValueError: If world_transform is not 4x4
        """
        if world_transform.shape != (4, 4):
            raise ValueError(
                f'world_transform must be 4x4, got {world_transform.shape}'
            )

        self.robot = robot_object
        self.world_transform = world_transform

    def create_shape_for_all_links(self) -> TopoDS_Compound:
        """Combine all link geometries into single OCCT compound.

        Iterates through all links in the URDF, converts collision geometry
        to OCCT primitives, applies transforms, and fuses them together.

        Returns:
            Combined TopoDS_Compound representing entire robot
        """
        compound = TopoDS_Compound()
        builder = BRep_Builder()
        builder.MakeCompound(compound)

        for link in self.robot.links:
            collisions = (
                link.collisions
                if link.collisions
                else ([link.collision] if link.collision else [])
            )

            if not collisions:
                continue

            link_shape = self.create_link_shape(link)
            builder.Add(compound, link_shape)

        return compound

    def create_link_shape(self, link: Any) -> TopoDS_Shape:
        """Create OCCT shape for all collision elements in a link.

        Args:
            link: URDF link object

        Returns:
            TopoDS_Shape representing union of all collision geometries

        Raises:
            ValueError: If geometry type is unsupported or has invalid dimensions
        """
        collisions = (
            link.collisions
            if link.collisions
            else ([link.collision] if link.collision else [])
        )

        shapes = []

        for idx, collision in enumerate(collisions):
            geom = collision.geometry

            if isinstance(geom, Box):
                if len(geom.size) != 3:
                    raise ValueError(
                        f"Link '{link.name}' collision {idx}: "
                        f'Box size must be [x, y, z], got {geom.size}'
                    )

                dx, dy, dz = geom.size
                pnt = gp_Pnt(-dx/2, -dy/2, -dz/2)
                shape = BRepPrimAPI_MakeBox(pnt, dx, dy, dz).Shape()

            elif isinstance(geom, Cylinder):
                if geom.radius <= 0 or geom.length <= 0:
                    raise ValueError(
                        f"Link '{link.name}' collision {idx}: "
                        f'Cylinder dimensions must be positive: '
                        f'radius={geom.radius}, length={geom.length}'
                    )

                # Cylinder centered along Z-axis
                ax = gp_Ax2(gp_Pnt(0, 0, -geom.length/2), gp_Dir(0, 0, 1))
                shape = BRepPrimAPI_MakeCylinder(ax, geom.radius, geom.length).Shape()

            elif isinstance(geom, Sphere):
                if geom.radius <= 0:
                    raise ValueError(
                        f"Link '{link.name}' collision {idx}: "
                        f'Sphere radius must be positive: {geom.radius}'
                    )

                shape = BRepPrimAPI_MakeSphere(geom.radius).Shape()

            elif isinstance(geom, Mesh):
                raise ValueError(
                    f"Link '{link.name}' collision {idx}: "
                    f'Mesh geometry is not supported'
                )

            else:
                raise ValueError(
                    f"Link '{link.name}' collision {idx}: "
                    f'Unsupported geometry type: {type(geom).__name__}'
                )

            # Apply collision transform
            local_T = self._get_collision_transform(collision)
            absolute_T = self.world_transform @ local_T

            trsf = self._numpy_to_gp_trsf(absolute_T)
            transformed_shape = BRepBuilderAPI_Transform(shape, trsf).Shape()
            shapes.append(transformed_shape)

        # Fuse all collision shapes in this link
        if len(shapes) == 1:
            return shapes[0]

        result = shapes[0]
        for shape in shapes[1:]:
            result = BRepAlgoAPI_Fuse(result, shape).Shape()

        return result

    def _get_collision_transform(self, collision) -> np.ndarray:
        """Extract 4x4 transform from URDF collision origin."""
        origin = collision.origin if collision.origin else None

        if origin is None:
            return np.eye(4)

        xyz = origin.xyz if origin.xyz else [0, 0, 0]
        rpy = origin.rpy if origin.rpy else [0, 0, 0]

        rot_matrix = Rotation.from_euler('xyz', rpy).as_matrix()

        T = np.eye(4)
        T[:3, :3] = rot_matrix
        T[:3, 3] = xyz

        return T

    def _numpy_to_gp_trsf(self, matrix: np.ndarray) -> gp_Trsf:
        """Convert numpy 4x4 matrix to OCCT gp_Trsf."""
        trsf = gp_Trsf()

        trsf.SetValues(
            matrix[0, 0], matrix[0, 1], matrix[0, 2], matrix[0, 3],
            matrix[1, 0], matrix[1, 1], matrix[1, 2], matrix[1, 3],
            matrix[2, 0], matrix[2, 1], matrix[2, 2], matrix[2, 3],
        )

        return trsf
