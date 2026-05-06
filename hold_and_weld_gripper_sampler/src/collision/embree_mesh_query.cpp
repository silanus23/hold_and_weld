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

#include "hold_and_weld_gripper_sampler/collision/embree_mesh_query.hpp"

#include <embree4/rtcore.h>

#include <stdexcept>
#include <string>
#include <cstring>

#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

static void check_embree_error(RTCDevice device, const char * context)
{
  RTCError err = rtcGetDeviceError(device);
  if (err != RTC_ERROR_NONE) {
    std::string msg = std::string("Embree error in ") + context + ": ";
    switch (err) {
      case RTC_ERROR_UNKNOWN:           msg += "RTC_ERROR_UNKNOWN"; break;
      case RTC_ERROR_INVALID_ARGUMENT:  msg += "RTC_ERROR_INVALID_ARGUMENT"; break;
      case RTC_ERROR_INVALID_OPERATION: msg += "RTC_ERROR_INVALID_OPERATION"; break;
      case RTC_ERROR_OUT_OF_MEMORY:     msg += "RTC_ERROR_OUT_OF_MEMORY"; break;
      case RTC_ERROR_UNSUPPORTED_CPU:   msg += "RTC_ERROR_UNSUPPORTED_CPU"; break;
      case RTC_ERROR_CANCELLED:         msg += "RTC_ERROR_CANCELLED"; break;
      default:                          msg += "unknown code " +
          std::to_string(static_cast<int>(err)); break;
    }
    throw std::runtime_error(msg);
  }
}

EmbreeMeshQuery::EmbreeMeshQuery(
  const TopoDS_Shape & shape,
  double linear_deflection)
{
  if (shape.IsNull()) {
    throw std::runtime_error("EmbreeMeshQuery: input shape is null");
  }

  IMeshTools_Parameters mesh_params;
  mesh_params.Deflection = linear_deflection;
  mesh_params.Angle = 0.5;
  mesh_params.DeflectionInterior = linear_deflection * 10.0;
  mesh_params.InParallel = false;  // single-threaded to avoid OCCT mesh race
  BRepMesh_IncrementalMesh mesher(shape, mesh_params);

  for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
    TopoDS_Face face = TopoDS::Face(exp.Current());
    TopLoc_Location loc;
    Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
    if (tri.IsNull()) {continue;}

    const gp_Trsf & trsf = loc.Transformation();
    const bool is_identity = loc.IsIdentity();

    unsigned int offset = static_cast<unsigned int>(vertex_buf_.size());

    for (int i = 1; i <= tri->NbNodes(); ++i) {
      gp_Pnt p = is_identity ? tri->Node(i) : tri->Node(i).Transformed(trsf);
      vertex_buf_.push_back({
            static_cast<float>(p.X()),
            static_cast<float>(p.Y()),
            static_cast<float>(p.Z())});
    }

    const bool reversed = (exp.Current().Orientation() == TopAbs_REVERSED);
    for (int i = 1; i <= tri->NbTriangles(); ++i) {
      int n1, n2, n3;
      tri->Triangle(i).Get(n1, n2, n3);
      if (reversed) {std::swap(n2, n3);}
      index_buf_.push_back({
            offset + static_cast<unsigned int>(n1 - 1),
            offset + static_cast<unsigned int>(n2 - 1),
            offset + static_cast<unsigned int>(n3 - 1)});
    }
  }

  if (vertex_buf_.empty() || index_buf_.empty()) {
    throw std::runtime_error(
      "EmbreeMeshQuery: shape produced no triangles — check meshing parameters");
  }

  num_vertices_ = static_cast<unsigned int>(vertex_buf_.size());
  num_triangles_ = static_cast<unsigned int>(index_buf_.size());

  commit_scene();
}

EmbreeMeshQuery::EmbreeMeshQuery(
  const std::vector<std::array<float, 3>> & vertices,
  const std::vector<std::array<unsigned int, 3>> & triangles)
: vertex_buf_(vertices),
  index_buf_(triangles),
  num_triangles_(static_cast<unsigned int>(triangles.size())),
  num_vertices_(static_cast<unsigned int>(vertices.size()))
{
  if (vertex_buf_.empty() || index_buf_.empty()) {
    throw std::runtime_error("EmbreeMeshQuery: empty vertex or index array");
  }
  commit_scene();
}

EmbreeMeshQuery::EmbreeMeshQuery(EmbreeMeshQuery && other) noexcept
: device_(other.device_),
  scene_(other.scene_),
  vertex_buf_(std::move(other.vertex_buf_)),
  index_buf_(std::move(other.index_buf_)),
  num_triangles_(other.num_triangles_),
  num_vertices_(other.num_vertices_),
  valid_(other.valid_)
{
  other.device_ = nullptr;
  other.scene_ = nullptr;
  other.valid_ = false;
}

EmbreeMeshQuery & EmbreeMeshQuery::operator=(EmbreeMeshQuery && other) noexcept
{
  if (this != &other) {
    // Release owned handles
    if (scene_) {rtcReleaseScene(scene_);   scene_ = nullptr;}
    if (device_) {rtcReleaseDevice(device_); device_ = nullptr;}

    device_ = other.device_;
    scene_ = other.scene_;
    vertex_buf_ = std::move(other.vertex_buf_);
    index_buf_ = std::move(other.index_buf_);
    num_triangles_ = other.num_triangles_;
    num_vertices_ = other.num_vertices_;
    valid_ = other.valid_;

    other.device_ = nullptr;
    other.scene_ = nullptr;
    other.valid_ = false;
  }
  return *this;
}

EmbreeMeshQuery::~EmbreeMeshQuery()
{
  if (scene_) {rtcReleaseScene(scene_);   scene_ = nullptr;}
  if (device_) {rtcReleaseDevice(device_); device_ = nullptr;}
}

void EmbreeMeshQuery::commit_scene()
{
  valid_ = false;

  RTCGeometry geom = nullptr;
  try {
    device_ = rtcNewDevice(nullptr);
    if (!device_) {throw std::runtime_error("EmbreeMeshQuery: failed to create RTCDevice");}
    check_embree_error(device_, "rtcNewDevice");

    scene_ = rtcNewScene(device_);
    if (!scene_) {throw std::runtime_error("EmbreeMeshQuery: failed to create RTCScene");}

    geom = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_TRIANGLE);
    check_embree_error(device_, "rtcNewGeometry");

    // Share the host buffers with Embree — vertex_buf_ and index_buf_ must
    // remain alive for the scene lifetime (they are members of this class).
    rtcSetSharedGeometryBuffer(
      geom,
      RTC_BUFFER_TYPE_VERTEX,
      0,
      RTC_FORMAT_FLOAT3,
      vertex_buf_.data(),
      0,
      sizeof(std::array<float, 3>),
      num_vertices_);
    check_embree_error(device_, "rtcSetSharedGeometryBuffer(vertex)");

    rtcSetSharedGeometryBuffer(
      geom,
      RTC_BUFFER_TYPE_INDEX,
      0,
      RTC_FORMAT_UINT3,
      index_buf_.data(),
      0,
      sizeof(std::array<unsigned int, 3>),
      num_triangles_);
    check_embree_error(device_, "rtcSetSharedGeometryBuffer(index)");

    rtcCommitGeometry(geom);
    check_embree_error(device_, "rtcCommitGeometry");

    rtcAttachGeometry(scene_, geom);
    check_embree_error(device_, "rtcAttachGeometry");

    rtcReleaseGeometry(geom);  // scene holds its own reference
    geom = nullptr;

    rtcCommitScene(scene_);
    check_embree_error(device_, "rtcCommitScene");
  } catch (const std::runtime_error & e) {
    if (geom) {rtcReleaseGeometry(geom);}
    if (scene_) {rtcReleaseScene(scene_);   scene_ = nullptr;}
    if (device_) {rtcReleaseDevice(device_); device_ = nullptr;}
    throw;
  }

  valid_ = true;
}

std::optional<gp_Pnt> EmbreeMeshQuery::ray_intersect(
  const gp_Pnt & origin,
  const gp_Dir & direction,
  double max_distance) const
{
  if (!valid_) {return std::nullopt;}
  if (max_distance <= 1e-6) {return std::nullopt;}

  RTCRayHit rayhit;
  std::memset(&rayhit, 0, sizeof(rayhit));

  rayhit.ray.org_x = static_cast<float>(origin.X());
  rayhit.ray.org_y = static_cast<float>(origin.Y());
  rayhit.ray.org_z = static_cast<float>(origin.Z());

  rayhit.ray.dir_x = static_cast<float>(direction.X());
  rayhit.ray.dir_y = static_cast<float>(direction.Y());
  rayhit.ray.dir_z = static_cast<float>(direction.Z());

  rayhit.ray.tnear = 1e-6f;                               // small epsilon to skip self-hit
  rayhit.ray.tfar = static_cast<float>(max_distance);
  rayhit.ray.mask = 0xFFFFFFFF;
  rayhit.ray.flags = 0;
  rayhit.ray.time = 0.0f;
  rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);

  rtcIntersect1(scene_, &rayhit, &args);

  if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
    return std::nullopt;  // miss
  }

  const float t = rayhit.ray.tfar;
  return gp_Pnt(
    origin.X() + t * direction.X(),
    origin.Y() + t * direction.Y(),
    origin.Z() + t * direction.Z());
}

// Hits are counted via repeated rtcIntersect1 because RTCFilterFunctionNArguments
// uses a packet type; casting it to a scalar hit struct is UB.  Odd count = inside.
bool EmbreeMeshQuery::point_inside(const gp_Pnt & point) const
{
  if (!valid_) {return false;}

  // +X shoot direction — any fixed direction works with Embree's watertight intersector.
  constexpr float kEps = 1e-4f;   // advance past each hit by this amount (0.1 mm)
  constexpr int   kMaxIter = 64;  // safety cap — no real mesh needs more

  float tnear = kEps;
  int hit_count = 0;

  RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);

  for (int iter = 0; iter < kMaxIter; ++iter) {
    RTCRayHit rayhit;
    std::memset(&rayhit, 0, sizeof(rayhit));

    rayhit.ray.org_x = static_cast<float>(point.X());
    rayhit.ray.org_y = static_cast<float>(point.Y());
    rayhit.ray.org_z = static_cast<float>(point.Z());
    rayhit.ray.dir_x = 1.0f;
    rayhit.ray.dir_y = 0.0f;
    rayhit.ray.dir_z = 0.0f;
    rayhit.ray.tnear = tnear;
    rayhit.ray.tfar = 1e30f;
    rayhit.ray.mask = 0xFFFFFFFF;
    rayhit.ray.flags = 0;
    rayhit.ray.time = 0.0f;
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(scene_, &rayhit, &iargs);

    if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
      break;  // no more hits — done
    }

    ++hit_count;
    // Advance tnear just past this hit so the next call skips it.
    tnear = rayhit.ray.tfar + kEps;
  }

  // Odd intersection count → point is inside the closed mesh (Jordan theorem).
  return (hit_count % 2) == 1;
}

bool EmbreeMeshQuery::is_valid() const
{
  return valid_;
}

unsigned int EmbreeMeshQuery::num_triangles() const
{
  return num_triangles_;
}

unsigned int EmbreeMeshQuery::num_vertices() const
{
  return num_vertices_;
}

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler
