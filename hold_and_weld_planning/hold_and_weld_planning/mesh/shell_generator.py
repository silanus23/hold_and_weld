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

"""ShellGenerator - Generate watertight mesh shells from URDF collision geometry.

Uses manifold3d for robust boolean operations, combining every collision
primitive of a part into a single watertight shell ready for contact
extraction.
"""

import logging
from typing import Any, Optional

import manifold3d
import numpy as np
from numpy.typing import NDArray
from urdf_parser_py.urdf import Box, Cylinder, Mesh, Sphere

from ..utils.transforms import link_poses, origin_to_matrix

logger = logging.getLogger(__name__)


class ShellGenerator:
    """Generate watertight mesh shells from URDF collision geometry.

    Uses manifold3d for robust boolean operations to combine multiple
    collision primitives into a single watertight shell per part.
    """

    def __init__(
        self,
        robot_object: Any,
        world_transform: Optional[NDArray] = None,
        refine_iterations: int = 32,
    ) -> None:
        """Initialize shell generator.

        Args:
            robot_object: The self.robot object from URDFProcessor
            world_transform: The global starting pose matrix (4x4). Defaults
                to identity.
            refine_iterations: Number of mesh subdivision iterations (default: 32)

        Raises:
            ValueError: If world_transform is not 4x4, or the URDF's joint
                tree does not place every link.
        """
        # Built here rather than in the signature: a default argument is one
        # array shared by every caller, and a caller that transforms it in
        # place moves every later part that took the default with it.
        if world_transform is None:
            world_transform = np.eye(4)

        if world_transform.shape != (4, 4):
            raise ValueError(
                f'world_transform must be 4x4, got {world_transform.shape}'
            )

        if refine_iterations < 0:
            raise ValueError(f'refine_iterations must be non-negative, got {refine_iterations}')

        self.robot = robot_object
        self.world_transform = world_transform
        self.refine_iterations = refine_iterations
        self.total_manifold = manifold3d.Manifold()
        # A collision origin is stated relative to its LINK, not to the model
        # root, so a multi-link part needs the joint tree walked before any of
        # its geometry can be placed.
        self.link_poses = link_poses(robot_object)

        logger.debug(f'ShellGenerator initialized with {refine_iterations} refine iterations')

    def create_shells_for_all_links(self) -> manifold3d.Manifold:
        """Combine all link geometries into single watertight shell.

        Iterates through all links in the URDF, converts collision geometry
        to manifold primitives, applies transforms, and unions them together.

        Returns:
            Combined manifold representing entire robot shell
        Raises:
            RuntimeError: If any link's geometry cannot be built. A shell
                missing a link is not a smaller workpiece, it is the wrong
                one: seam extraction would go on to weld the hole the
                missing link left, so this cannot be downgraded to a skip.
        """
        logger.info(f'Creating shells for {len(self.robot.links)} links')

        processed_count = 0
        for link in self.robot.links:
            collisions = (
                link.collisions
                if link.collisions
                else ([link.collision] if link.collision else [])
            )

            if not collisions:
                logger.debug(f"Link '{link.name}' has no collision geometry, skipping")
                continue

            logger.debug(
                f"Processing link '{link.name}' with {len(collisions)} collision element(s)"
            )

            try:
                link_manifold = self.create_link_shell(link)
            except Exception as e:
                logger.error(f"Failed to create shell for link '{link.name}': {e}")
                raise RuntimeError(
                    f"Failed to create shell for link '{link.name}': {e}"
                )

            self.total_manifold += link_manifold
            processed_count += 1

        logger.info(f'Successfully created shells for {processed_count} link(s)')
        return self.total_manifold

    def create_link_shell(self, link: Any) -> manifold3d.Manifold:
        """Create combined manifold for all collision elements in a link.

        Args:
            link: URDF link object
        Returns:
            Manifold representing union of all collision geometries
        Raises:
            ValueError: If geometry type is unsupported
            RuntimeError: If mesh loading or transform fails
        """
        link_combined = manifold3d.Manifold()

        collisions = (
            link.collisions
            if link.collisions
            else ([link.collision] if link.collision else [])
        )

        for idx, collision in enumerate(collisions):
            geom = collision.geometry
            manifold_obj = None

            logger.debug(f'Processing collision element {idx}: {type(geom).__name__}')

            try:
                if isinstance(geom, Box):
                    if len(geom.size) != 3:
                        raise ValueError(f'Box size must be [x, y, z], got {geom.size}')
                    manifold_obj = manifold3d.Manifold.cube(geom.size, center=True)
                    logger.debug(f'Created box: size={geom.size}')

                elif isinstance(geom, Cylinder):
                    if geom.radius <= 0 or geom.length <= 0:
                        raise ValueError(
                            f'Cylinder dimensions must be positive: '
                            f'radius={geom.radius}, length={geom.length}'
                        )
                    manifold_obj = manifold3d.Manifold.cylinder(
                        geom.length, geom.radius, circular_segments=128, center=True
                    )
                    logger.debug(f'Created cylinder: radius={geom.radius}, length={geom.length}')

                elif isinstance(geom, Sphere):
                    if geom.radius <= 0:
                        raise ValueError(
                            f'Sphere radius must be positive: {geom.radius}'
                        )
                    manifold_obj = manifold3d.Manifold.sphere(
                        geom.radius, circular_segments=128
                    )
                    logger.debug(f'Created sphere: radius={geom.radius}')

                elif isinstance(geom, Mesh):
                    raise ValueError(
                        f'Mesh geometry is not supported in collision '
                        f"element {idx} of link '{link.name}' — "
                        f'convert collision geometry to primitives (Box/Cylinder/Sphere) '
                        f'or use the STL pipeline instead'
                    )

                else:
                    raise ValueError(
                        f'Unsupported geometry type: {type(geom).__name__} '
                        f"in collision element {idx} of link '{link.name}'"
                    )

                if manifold_obj is not None:
                    if self.refine_iterations > 0:
                        manifold_obj = manifold_obj.refine(self.refine_iterations)

                    link_T = self.link_poses.get(link.name, np.eye(4))
                    local_T = origin_to_matrix(collision.origin)
                    absolute_T = self.world_transform @ link_T @ local_T

                    mat_3x4 = absolute_T[:3, :].tolist()  # manifold3d takes [R|t] not 4x4
                    transformed_obj = manifold_obj.transform(mat_3x4)

                    link_combined += transformed_obj

            except Exception as e:
                logger.error(f"Failed to process collision {idx} in link '{link.name}': {e}")
                raise RuntimeError(
                    f"Failed to process collision {idx} in link '{link.name}': {e}"
                )

        return link_combined
