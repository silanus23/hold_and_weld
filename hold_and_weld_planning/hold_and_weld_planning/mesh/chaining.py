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

"""Turn loose contact-boundary edges into ordered, oriented seam polylines.

Four steps, in the order `SeamExtractorMesh.extract_chains` applies them: `loops`, `oriented`,
`drop_coincident`, `stitch`.

Plain functions of their arguments, reading only vertex positions and `SeamExtractorMeshParams`
tuning - kept separate from `SeamExtractorMesh` so they can be tested against plain arrays, no mesh
or extractor required.
"""

import logging

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import KDTree
import trimesh

from .params import SeamExtractorMeshParams

logger = logging.getLogger(__name__)


def loops(
    side: int,
    boundary: list[tuple[int, int]],
    cfg: SeamExtractorMeshParams,
) -> list[tuple[list[int], bool]]:
    """Walk boundary edges into ordered chains of vertex indices.

    A boundary vertex normally joins exactly two edges, so the walk has one way to continue. Graph
    degree above two means two boundary curves cross, and nothing local says which branch continues
    which seam, so the walk stops there rather than guessing. The curves through it come out as
    open chains instead of one spliced loop.

    Args:
        side: Naming only, for logs.

    Returns:
        (vertex_indices, is_closed) per chain, shorter chains discarded. A closed chain does
        not repeat its first vertex at the end.
    """
    adjacency: dict[int, list[int]] = {}
    for a, b in boundary:
        adjacency.setdefault(a, []).append(b)
        adjacency.setdefault(b, []).append(a)

    junctions = {v for v, n in adjacency.items() if len(n) != 2}
    crossings = sum(len(n) > 2 for n in adjacency.values())
    if crossings:
        logger.info(
            f'mesh_{side} contact boundary crosses itself at {crossings} vertex(es); '
            'cutting into open chains there')

    chains: list[tuple[list[int], bool]] = []
    used: set = set()

    def walk(start: int, first: int) -> None:
        chain = [start, first]
        used.add((min(start, first), max(start, first)))
        previous, current = start, first
        while current not in junctions:
            nxt = next(v for v in adjacency[current] if v != previous)
            key = (min(current, nxt), max(current, nxt))
            if key in used:
                break
            used.add(key)
            chain.append(nxt)
            previous, current = current, nxt
        # A branch can come back to the junction it left, which closes it just like a plain loop.
        closed = chain[-1] == chain[0]
        chains.append((chain[:-1] if closed else chain, closed))

    # Junctions first, so every walk after them runs on a plain cycle.
    for start in [*junctions, *adjacency]:
        for neighbour in adjacency[start]:
            if (min(start, neighbour), max(start, neighbour)) not in used:
                walk(start, neighbour)

    return [(chain, closed) for chain, closed in chains if len(chain) >= cfg.min_loop_points]


def oriented(
    mesh: trimesh.Trimesh,
    loop: list[int],
    contact: NDArray,
    wall_normal: NDArray,
) -> list[int]:
    """Orient a loop so the contact region lies consistently to one side.

    Judged at the loop's first edge against the one contact face on it, not the centroid of the
    whole contact set: an inner rim has its contact outside the loop, and parts touching in
    separate places put that centroid between them.

    Args:
        contact: Boolean contact mask over the mesh faces.
        wall_normal: Unit normal of the wall rising out of the joint at `loop[0]`.

    Returns:
        The loop, reversed after its first vertex when it ran the wrong way.
    """
    start, end = mesh.vertices[loop[0]], mesh.vertices[loop[1]]
    shared = np.intersect1d(mesh.vertex_faces[loop[0]], mesh.vertex_faces[loop[1]])
    shared = shared[shared >= 0]
    face = shared[contact[shared]][0]

    inward = np.cross(end - start, wall_normal)
    if np.dot(inward, mesh.triangles_center[face] - (start + end) / 2.0) < 0.0:
        return [loop[0], *loop[1:][::-1]]
    return loop


def drop_coincident(
    pieces: list[tuple[NDArray, bool]], cfg: SeamExtractorMeshParams
) -> list[tuple[NDArray, bool]]:
    """Drop a piece that lies on top of a better-sampled one.

    On an edge-to-edge joint both parts terminate on the same curve and it would be welded twice.
    Coincident pieces sit within epsilon along their whole length - the same condition that defined
    contact, so no new tolerance is introduced.

    Returns:
        The same list less the pieces another piece already covers.
    """
    trees = [KDTree(positions) for positions, _ in pieces]

    keep: list[tuple[NDArray, bool]] = []
    for index, (positions, is_closed) in enumerate(pieces):
        redundant = False
        n_pos = len(positions)
        for other_index, (others, _) in enumerate(pieces):
            n_other = len(others)
            if other_index == index or n_other < n_pos:
                continue
            if n_other == n_pos and other_index > index:
                continue
            max_dist = trees[other_index].query(positions)[0].max()
            if float(max_dist) <= cfg.epsilon:
                redundant = True
                break
        if redundant:
            logger.info(
                f'Dropping {len(positions)}-point piece: another piece '
                'samples the same curve more finely'
            )
        else:
            keep.append((positions, is_closed))
    return keep


def stitch(
    pieces: list[tuple[NDArray, bool]], cfg: SeamExtractorMeshParams
) -> list[tuple[NDArray, bool]]:
    """Join polylines from different meshes end to end into one chain.

    Each mesh contributes only the portion of the seam where it terminates, and the two share no
    vertices, so the pieces meet with a gap of roughly one segment. Closing those gaps gives one
    chain whose ownership alternates along its length.

    Returns:
        Closed chains pass through untouched; an open chain that closes on itself is returned
        closed, with its repeated last point removed.
    """
    open_pieces = [list(p) for p, closed in pieces if not closed]
    result = [(p, True) for p, closed in pieces if closed]

    def span(piece: list[NDArray]) -> float:
        """Measure the sample spacing local to the ENDS of one piece.

        At the ends, not pooled over the whole piece: a stitched chain crosses meshes sampled at
        very different densities, so a pooled median would be dominated by the finer mesh and judge
        a coarse end's gap far too tight.
        """
        array = np.asarray(piece)
        window = min(cfg.min_loop_points, len(array))
        head_diffs = np.diff(array[:window], axis=0)
        tail_diffs = np.diff(array[-window:], axis=0)

        head_med = np.median(np.linalg.norm(head_diffs, axis=1))
        tail_med = np.median(np.linalg.norm(tail_diffs, axis=1))
        return float(max(head_med, tail_med))

    merged = True
    while merged and len(open_pieces) > 1:
        merged = False
        for i in range(len(open_pieces)):
            for j in range(len(open_pieces)):
                if i == j:
                    continue
                a, b = open_pieces[i], open_pieces[j]
                endpoints = [
                    (np.linalg.norm(a[-1] - b[0]), False, False),
                    (np.linalg.norm(a[-1] - b[-1]), False, True),
                    (np.linalg.norm(a[0] - b[0]), True, False),
                    (np.linalg.norm(a[0] - b[-1]), True, True),
                ]
                gap, flip_a, flip_b = min(endpoints, key=lambda x: x[0])
                # Pieces from different meshes never share a position exactly; ends that do meet
                # at a crossing `loops` cut on purpose, and joining them would guess the branch.
                if gap == 0.0:
                    continue
                tolerance = cfg.stitch_gap_factor * max(span(a), span(b))
                if gap > tolerance:
                    continue
                logger.info(
                    f'Stitched a {len(a)}-point and a {len(b)}-point piece; '
                    f'gap {gap * 1000:.2f}mm within tolerance '
                    f'{tolerance * 1000:.2f}mm'
                )
                head = a[::-1] if flip_a else a
                tail = b[::-1] if flip_b else b
                open_pieces[i] = head + tail
                open_pieces.pop(j)
                merged = True
                break
            if merged:
                break

    for piece in open_pieces:
        array = np.asarray(piece)
        gap = float(np.linalg.norm(array[0] - array[-1]))
        length = float(np.linalg.norm(np.diff(array, axis=0), axis=1).sum())
        # A straight run's ends sit its whole length apart, so a chain only a few samples long
        # passes the spacing test alone; a loop has to turn back toward its start.
        closed = gap <= cfg.stitch_gap_factor * span(piece) and gap <= length / 2.0
        # Closing doesn't make the last point redundant - the gap that closed it is a real step.
        # Only a last point landed exactly ON the first is a repeat.
        if gap == 0.0:
            array = array[:-1]
        result.append((array, closed))
    return result
