// Copyright 2026 Berkan Tali
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__TOPOLOGY_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__TOPOLOGY_HPP_

#include <utility>
#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

/**
 * @brief Represents a corner (vertex) in the object topology
 *
 * Stores position and connectivity to adjacent edges and surfaces.
 * All data is in world frame, after the load-time transform is applied.
 */
struct Corner
{
  /** Corner position in the world frame [m]. */
  gp_Pnt position;

  /** IDs of edges connected to this corner. */
  std::vector<int> connected_edges;

  /** IDs of surfaces that share this corner. */
  std::vector<int> connected_surfaces;

  Corner() = default;
};

/**
 * @brief Represents an edge in the object topology
 *
 * Stores the two corner endpoints and connectivity to adjacent surfaces.
 */
struct Edge
{
  TopoDS_Edge edge;

  /** IDs of the two end corners; the edge is undirected, so the order is arbitrary. */
  std::pair<int, int> corner_ids;

  /** IDs of surfaces that share this edge: 1 on a boundary, 2 inside, more if non-manifold. */
  std::vector<int> connected_surfaces;

  Edge() = default;
};

/**
 * @brief Represents a surface (face) in the object topology
 *
 * Stores OCCT face handle, geometric properties, and connectivity.
 * All geometric data is in world frame after transformation.
 */
struct Surface
{
  TopoDS_Face face;

  /**
   * Outward unit normal in the world frame, already flipped for TopAbs_REVERSED faces.
   * Evaluated at the UV centre of the face; a fallback is stored where it is undefined there.
   */
  gp_Vec normal;

  /** Area centroid in the world frame [m]. */
  gp_Pnt center;

  /** IDs of edges bounding this surface. */
  std::vector<int> edge_ids;

  /** IDs of corners belonging to this surface. */
  std::vector<int> corner_ids;

  /** True if the face has inner wires (holes), e.g. a washer's flat face. */
  bool has_inner_holes;

  Surface()
  : has_inner_holes(false)
  {}
};

/**
 * @brief Container for object topology extracted from a loaded shape
 *        (STEP, URDF or a raw OCCT shape).
 *
 * Holds corners, edges, and surfaces with their connectivity
 * relationships. All elements are 0-indexed.
 *
 * Built by GeometryMapper when a shape is loaded.
 */
class Topology
{
public:
  /**
   * @brief Default constructor - creates empty topology
   */
  Topology() = default;

  /**
   * @brief Get corner by ID (const access)
   *
   * @param id Corner ID (0-indexed)
   * @return Reference to Corner struct
   */
  const Corner & get_corner(int id) const;

  /**
   * @brief Get edge by ID (const access)
   *
   * @param id Edge ID (0-indexed)
   * @return Reference to Edge struct
   */
  const Edge & get_edge(int id) const;

  /**
   * @brief Get surface by ID (const access)
   *
   * @param id Surface ID (0-indexed)
   * @return Reference to Surface struct
   */
  const Surface & get_surface(int id) const;

  /**
   * @brief Get all surface IDs
   *
   * @deprecated Use num_surfaces() + index loop or get_all_surfaces() instead.
   * This method allocates a vector of [0..N-1] which is redundant.
   *
   * @return Vector of surface IDs (0-indexed, sorted)
   */
  [[deprecated("Use num_surfaces() + index loop or get_all_surfaces() instead")]]
  std::vector<int> get_all_surface_ids() const;

  /**
   * @brief Get direct access to all corners
   *
   * @return Const reference to corners vector
   */
  const std::vector<Corner> & get_all_corners() const;

  /**
   * @brief Get direct access to all edges
   *
   * @return Const reference to edges vector
   */
  const std::vector<Edge> & get_all_edges() const;

  /**
   * @brief Get direct access to all surfaces (const)
   *
   * @return Const reference to surfaces vector
   */
  const std::vector<Surface> & get_all_surfaces() const;

  /**
   * @brief Get total number of corners
   *
   * @return Corner count
   */
  size_t num_corners() const {return corners_.size();}

  /**
   * @brief Get total number of edges
   *
   * @return Edge count
   */
  size_t num_edges() const {return edges_.size();}

  /**
   * @brief Get total number of surfaces
   *
   * @return Surface count
   */
  size_t num_surfaces() const {return surfaces_.size();}

  /**
   * @brief Add a corner to the topology
   *
   * @param corner Corner data
   * @return Assigned corner ID (0-indexed)
   */
  int add_corner(const Corner & corner);

  /**
   * @brief Add an edge to the topology
   *
   * @param edge Edge data
   * @return Assigned edge ID (0-indexed)
   */
  int add_edge(const Edge & edge);

  /**
   * @brief Add a surface to the topology
   *
   * @param surface Surface data
   * @return Assigned surface ID (0-indexed)
   */
  int add_surface(const Surface & surface);

  /**
   * @brief Clear all topology data
   */
  void clear();

private:
  std::vector<Corner> corners_;
  std::vector<Edge> edges_;
  std::vector<Surface> surfaces_;
};

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__TOPOLOGY_HPP_
