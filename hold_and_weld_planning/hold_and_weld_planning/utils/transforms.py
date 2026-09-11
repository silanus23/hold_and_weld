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

"""Homogeneous transform helpers shared by the mesh and OCCT pipelines.

URDF states every pose as an `<origin xyz rpy>`, and both pipelines have to
turn one into a 4x4 before they can place anything. Kept here so the two
agree on the Euler convention rather than each restating it.
"""

import logging
from typing import Any, List, Optional

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)


def xyz_rpy_to_matrix(
    xyz: List[float] | NDArray,
    rpy: List[float] | NDArray,
) -> NDArray:
    """Build a 4x4 homogeneous transform from a position and fixed-axis RPY.

    Args:
        xyz: [x, y, z] translation in metres.
        rpy: [roll, pitch, yaw] in radians, URDF's fixed-axis convention.

    Returns:
        4x4 homogeneous transformation matrix.

    Raises:
        ValueError: If either argument does not have exactly 3 elements.
    """
    xyz_array = np.asarray(xyz, dtype=float).ravel()
    rpy_array = np.asarray(rpy, dtype=float).ravel()

    if xyz_array.shape != (3,):
        raise ValueError(f'xyz must have 3 elements, got {xyz_array.size}')
    if rpy_array.shape != (3,):
        raise ValueError(f'rpy must have 3 elements, got {rpy_array.size}')

    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', rpy_array).as_matrix()
    matrix[:3, 3] = xyz_array
    return matrix


def origin_to_matrix(origin: Optional[Any]) -> NDArray:
    """Convert a URDF `<origin>` element to a 4x4 homogeneous transform.

    A missing origin, or a missing xyz or rpy on one, means the identity for
    that part — the same default URDF itself gives them.

    Args:
        origin: The `origin` attribute of a URDF collision, joint or visual,
            or None.

    Returns:
        4x4 homogeneous transformation matrix.
    """
    if origin is None:
        return np.eye(4)

    xyz = origin.xyz if origin.xyz is not None else [0.0, 0.0, 0.0]
    rpy = origin.rpy if origin.rpy is not None else [0.0, 0.0, 0.0]

    return xyz_rpy_to_matrix(xyz, rpy)


def numpy_to_gp_trsf(matrix: NDArray) -> Any:
    """Convert a 4x4 homogeneous matrix to an OCCT gp_Trsf.

    gp_Trsf models a rigid motion with a uniform scale, so shear or
    non-uniform scale cannot survive the conversion. That is reported rather
    than refused: a slightly non-unit determinant is usually accumulated
    round-off in a pose composed from several origins.

    OCCT is imported inside the body so that the mesh pipeline, which shares
    this module for the URDF helpers above, does not pull in pythonocc.

    Returns:
        Equivalent gp_Trsf.

    Raises:
        ValueError: If the matrix is not 4x4.
    """
    from OCC.Core.gp import gp_Trsf

    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f'transform must be 4x4, got {matrix.shape}')

    determinant = np.linalg.det(matrix[:3, :3])
    if not np.isclose(determinant, 1.0, atol=1e-3):
        logger.warning(
            f'Transform has non-unit determinant {determinant:.6f}, '
            'may contain scaling/shear'
        )

    trsf = gp_Trsf()
    trsf.SetValues(
        matrix[0, 0], matrix[0, 1], matrix[0, 2], matrix[0, 3],
        matrix[1, 0], matrix[1, 1], matrix[1, 2], matrix[1, 3],
        matrix[2, 0], matrix[2, 1], matrix[2, 2], matrix[2, 3],
    )
    return trsf


def link_poses(robot: Any) -> dict:
    """Resolve every link's pose in the model's root frame.

    URDF states a link's placement only relative to its parent, through the
    joint between them, so a part whose links are not all at the origin is
    assembled wrong by anything that reads collision origins alone. Walks the
    joint tree from the root, accumulating parent pose @ joint origin.

    Joint variables are taken at zero: these are workpieces, whose links are
    held together by fixed joints. A movable joint is placed at its zero
    position, which is what a URDF drawn as a static assembly means.

    Args:
        robot: The parsed URDF model (urdf_parser_py `URDF`).

    Returns:
        Dict mapping link name to its 4x4 pose in the root frame. Links that
        no joint reaches — a disconnected model — are left at the identity.

    Raises:
        ValueError: If two links share a name, or a joint names a parent or
            child link that was never declared, or the joint tree contains a
            cycle.
    """
    link_names = [link.name for link in robot.links]
    duplicates = {name for name in link_names if link_names.count(name) > 1}
    if duplicates:
        raise ValueError(f'URDF has duplicate link name(s): {sorted(duplicates)}')

    poses = {name: np.eye(4) for name in link_names}

    # Caught here rather than left to surface downstream: a joint naming an
    # undeclared link would otherwise inject a spurious entry into `poses`
    # during the walk below, since that dict is keyed by whatever a joint
    # says rather than only by declared links.
    unknown = {
        (joint.parent if joint.parent not in poses else joint.child): joint
        for joint in robot.joints
        if joint.parent not in poses or joint.child not in poses
    }
    if unknown:
        raise ValueError(
            'URDF joint(s) reference undeclared link name(s): '
            f'{sorted(unknown)}'
        )

    children: dict = {}
    child_names = set()
    for joint in robot.joints:
        children.setdefault(joint.parent, []).append(joint)
        child_names.add(joint.child)

    roots = [name for name in poses if name not in child_names]

    visited = set()
    stack = [(name, np.eye(4)) for name in roots]
    while stack:
        name, pose = stack.pop()
        if name in visited:
            raise ValueError(
                f"URDF joint tree revisits link '{name}': the model is not a "
                'tree, so its links have no single well-defined pose'
            )
        visited.add(name)
        poses[name] = pose

        for joint in children.get(name, []):
            stack.append((joint.child, pose @ origin_to_matrix(joint.origin)))

    unreached = set(poses) - visited
    if unreached:
        # Not fatal on its own: a single-link part has no joints at all and a
        # root is still reached. Only a link no root reaches lands here.
        raise ValueError(
            'URDF links are not connected to the model root and have no '
            f'defined pose: {sorted(unreached)}'
        )

    return poses
