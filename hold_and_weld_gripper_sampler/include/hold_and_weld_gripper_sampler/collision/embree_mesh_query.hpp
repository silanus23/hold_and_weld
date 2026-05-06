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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__COLLISION__EMBREE_MESH_QUERY_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__COLLISION__EMBREE_MESH_QUERY_HPP_

#include <embree4/rtcore.h>

#include <optional>
#include <vector>

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <TopoDS_Shape.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

/**
 * @brief Embree 4 backed mesh query engine — watertight ray intersection and
 *        point-in-solid parity tests on a single triangulated mesh.
 *
 * This class owns the Embree device and scene lifetime.  Build it once from a
 * triangulated OCCT shape and reuse it for every query in the hot loop.
 *
 * Two primary operations are exposed:
 *
 *   ray_intersect — shoot a single ray and return the nearest hit point.
 *     Uses Embree's watertight Möller–Trumbore intersector, so flat faces
 *     represented by only 2 triangles are handled correctly regardless of
 *     grazing angles.  Replaces bvh_ray_cast inside FCLCollisionChecker.
 *
 *   point_inside — parity test: shoot a ray in a fixed direction from the
 *     query point and count all forward intersections.  An odd count means
 *     the point is inside the closed mesh.  Embree's watertight intersector
 *     guarantees no missed or double-counted triangles on edges/vertices,
 *     so a single direction suffices (no majority-vote hack needed).
 *     Replaces the FCL containment blind-spot in check_gripper_collision.
 *
 * Thread safety: after construction (i.e. after rtcCommitScene) the scene is
 * read-only and all query methods are safe to call concurrently.
 */
class EmbreeMeshQuery
{
public:
  /**
   * @brief Build the Embree scene directly from an OCCT shape.
   *
   * The shape is triangulated using BRepMesh_IncrementalMesh with the
   * supplied linear deflection.  All faces are collected into a single
   * rtcNewGeometry(RTC_GEOMETRY_TYPE_TRIANGLE) geometry.
   *
   * @param shape             OCCT shape to triangulate and load.
   * @param linear_deflection Triangulation chord-height tolerance in metres.
   *                          Smaller values produce denser meshes.
   *                          Typical value: 0.0001 (0.1 mm).
   *
   * @throws std::runtime_error if the shape is null, produces no triangles,
   *         or if Embree device/scene creation fails.
   */
  explicit EmbreeMeshQuery(
    const TopoDS_Shape & shape,
    double linear_deflection = 0.0001);

  /**
   * @brief Build the Embree scene from pre-extracted vertex and index arrays.
   *
   * Useful when the caller has already run shape_to_bvh and wants to reuse
   * the same vertex/triangle data without re-meshing.
   *
   * @param vertices  Flat vertex array — each element is {x, y, z} in metres.
   * @param triangles Flat triangle index array — each element holds three
   *                  zero-based indices into @p vertices.
   *
   * @throws std::runtime_error if either array is empty or Embree fails.
   */
  EmbreeMeshQuery(
    const std::vector<std::array<float, 3>> & vertices,
    const std::vector<std::array<unsigned int, 3>> & triangles);

  // Non-copyable — owns RTCDevice/RTCScene handles.
  EmbreeMeshQuery(const EmbreeMeshQuery &) = delete;
  EmbreeMeshQuery & operator=(const EmbreeMeshQuery &) = delete;

  // Movable.
  EmbreeMeshQuery(EmbreeMeshQuery &&) noexcept;
  EmbreeMeshQuery & operator=(EmbreeMeshQuery &&) noexcept;

  ~EmbreeMeshQuery();

  /**
   * @brief Shoot a single ray and return the nearest intersection point.
   *
   * Internally calls rtcIntersect1.  The direction need not be normalised;
   * it is normalised internally before the query.
   *
   * @param origin       Ray origin in world frame (metres).
   * @param direction    Ray direction (any non-zero length).
   * @param max_distance Maximum ray length to consider (metres).
   *
   * @return The nearest hit point on the mesh surface, or std::nullopt if the
   *         ray misses within @p max_distance.
   */
  std::optional<gp_Pnt> ray_intersect(
    const gp_Pnt & origin,
    const gp_Dir & direction,
    double max_distance) const;

  /**
   * @brief Test whether a point lies inside the closed mesh (parity test).
   *
   * Shoots a ray in the +X direction from @p point and counts all forward
   * intersections using a custom intersection filter.  An odd count means
   * the point is inside.
   *
   * Because Embree's watertight intersector guarantees no gaps at triangle
   * edges or vertices, a single ray direction is sufficient — unlike the
   * three-direction majority-vote required when using the hand-rolled FCL
   * BVH traversal.
   *
   * The ray origin is offset by a small epsilon (1 µm) in the shoot direction
   * to avoid self-intersection at the surface when the query point sits
   * exactly on a triangle.
   *
   * @param point Query point in world frame (metres).
   *
   * @return true  if the point is strictly inside the mesh.
   * @return false if the point is outside or on the surface.
   */
  bool point_inside(const gp_Pnt & point) const;

  /**
   * @brief Returns true if the scene was built successfully and is ready to query.
   */
  bool is_valid() const;

  /**
   * @brief Number of triangles loaded into the Embree scene.
   */
  unsigned int num_triangles() const;

  /**
   * @brief Number of vertices loaded into the Embree scene.
   */
  unsigned int num_vertices() const;

private:
  /**
   * @brief Shared initialisation: create device, build scene from vertex/index
   *        buffers that have already been populated into vertex_buf_ / index_buf_.
   */
  void commit_scene();

  RTCDevice device_{nullptr};
  RTCScene  scene_{nullptr};

  // Host-side copies kept alive because Embree may reference them (shared
  // buffer mode).  In this implementation we use RTC_BUFFER_TYPE_VERTEX /
  // RTC_BUFFER_TYPE_INDEX with rtcSetSharedGeometryBuffer so the data must
  // remain valid for the lifetime of the scene.
  std::vector<std::array<float, 3>> vertex_buf_;
  std::vector<std::array<unsigned int, 3>> index_buf_;

  unsigned int num_triangles_{0};
  unsigned int num_vertices_{0};
  bool valid_{false};
};

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__COLLISION__EMBREE_MESH_QUERY_HPP_
