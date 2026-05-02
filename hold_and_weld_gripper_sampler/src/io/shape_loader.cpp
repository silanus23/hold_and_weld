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

#include "hold_and_weld_gripper_sampler/io/shape_loader.hpp"

#include <tinyxml2.h>

#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <gp_Ax2.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Poly_Triangulation.hxx>
#include <Precision.hxx>
#include <RWStl.hxx>
#include <Standard_Failure.hxx>
#include <STEPControl_Reader.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hold_and_weld_gripper_sampler/geometry/occt_utils.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace io
{

static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

namespace
{

void validate_vector3d(const Eigen::Vector3d & vec, const std::string & context)
{
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(vec[i])) {
      throw std::invalid_argument(
        context + ": invalid vector component[" + std::to_string(i) + "] = " +
        std::to_string(vec[i]));
    }
  }
}

Eigen::Quaterniond validate_and_normalize_quaternion(
  const Eigen::Quaterniond & q,
  const std::string & context)
{
  if (!std::isfinite(q.x()) || !std::isfinite(q.y()) ||
    !std::isfinite(q.z()) || !std::isfinite(q.w()))
  {
    throw std::invalid_argument(context + ": quaternion contains non-finite values");
  }

  double norm = q.norm();
  if (norm < 1e-10) {
    throw std::invalid_argument(
      context + ": quaternion has near-zero norm: " + std::to_string(norm));
  }

  return q.normalized();
}

void validate_positive_dimension(
  double value,
  const std::string & name,
  const std::string & context)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(
      context + ": " + name + " is not finite: " + std::to_string(value));
  }
  if (value <= 0.0) {
    throw std::invalid_argument(
      context + ": " + name + " must be positive, got: " + std::to_string(value));
  }
}

void validate_dimensions(const Eigen::Vector3d & dims, const std::string & context)
{
  validate_vector3d(dims, context);
  for (int i = 0; i < 3; ++i) {
    if (dims[i] <= 0.0) {
      throw std::invalid_argument(
        context + ": dimension[" + std::to_string(i) + "] must be positive, got: " +
        std::to_string(dims[i]));
    }
  }
}

}  // namespace

ShapeLoader::ShapeLoader()
: config_()
{
  RCLCPP_DEBUG(logger_, "ShapeLoader initialized with default config");
}

ShapeLoader::ShapeLoader(const ShapeLoaderConfig & config)
: config_(config)
{
  RCLCPP_DEBUG(logger_, "ShapeLoader initialized with custom config");
}

TopoDS_Shape ShapeLoader::load_from_step(
  const std::string & step_path,
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & rotation)
{
  RCLCPP_INFO(logger_, "Loading STEP file: %s", step_path.c_str());

  validate_vector3d(translation, "load_from_step");
  Eigen::Quaterniond normalized_rotation =
    validate_and_normalize_quaternion(rotation, "load_from_step");

  std::ifstream file(step_path);
  if (!file.good()) {
    throw std::runtime_error("STEP file not found: " + step_path);
  }
  file.close();

  STEPControl_Reader reader;
  IFSelect_ReturnStatus status = reader.ReadFile(step_path.c_str());

  if (status != IFSelect_RetDone) {
    throw std::runtime_error("Failed to read STEP file: " + step_path);
  }

  Standard_Integer num_roots = reader.TransferRoots();
  RCLCPP_DEBUG(logger_, "Transferred %d root(s) from STEP file", num_roots);

  if (num_roots == 0) {
    throw std::runtime_error("No shapes found in STEP file: " + step_path);
  }

  TopoDS_Shape shape = reader.OneShape();

  if (shape.IsNull()) {
    throw std::runtime_error("Null shape loaded from STEP file: " + step_path);
  }

  if (!translation.isZero() || !normalized_rotation.isApprox(Eigen::Quaterniond::Identity())) {
    shape = apply_transform(shape, translation, normalized_rotation);
  }

  if (config_.auto_triangulate) {
    triangulate(shape);
  }

  RCLCPP_INFO(logger_, "STEP file loaded successfully");
  return shape;
}

TopoDS_Shape ShapeLoader::load_from_stl(
  const std::string & stl_path,
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & rotation)
{
  RCLCPP_INFO(logger_, "Loading STL file: %s", stl_path.c_str());

  validate_vector3d(translation, "load_from_stl");
  Eigen::Quaterniond normalized_rotation =
    validate_and_normalize_quaternion(rotation, "load_from_stl");

  std::ifstream file(stl_path);
  if (!file.good()) {
    throw std::runtime_error("STL file not found: " + stl_path);
  }
  file.close();

  Handle(Poly_Triangulation) triangulation = RWStl::ReadFile(stl_path.c_str());

  if (triangulation.IsNull()) {
    throw std::runtime_error("Failed to read STL file: " + stl_path);
  }

  if (triangulation->NbTriangles() == 0) {
    throw std::runtime_error("STL file contains no triangles: " + stl_path);
  }

  RCLCPP_DEBUG(logger_, "STL loaded: %d triangles, %d nodes",
    triangulation->NbTriangles(), triangulation->NbNodes());

  // Build shape from triangulation using sewing for better performance.
  // Sewing attempts to connect adjacent faces to create a manifold shell/solid.
  BRepBuilderAPI_Sewing sewing(config_.linear_deflection);
  // Non-manifold mode allows edges shared by more than two faces
  sewing.SetNonManifoldMode(Standard_True);

  // Create faces from triangles and add to sewing.
  // Note: OpenCASCADE uses 1-based indexing for triangulation data.
  for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
    const Poly_Triangle & tri = triangulation->Triangle(i);
    Standard_Integer n1, n2, n3;
    tri.Get(n1, n2, n3);

    gp_Pnt p1 = triangulation->Node(n1);
    gp_Pnt p2 = triangulation->Node(n2);
    gp_Pnt p3 = triangulation->Node(n3);

    // Skip degenerate triangles (vertices too close together)
    if (p1.IsEqual(p2, Precision::Confusion()) ||
      p2.IsEqual(p3, Precision::Confusion()) ||
      p3.IsEqual(p1, Precision::Confusion()))
    {
      continue;
    }

    BRepBuilderAPI_MakePolygon poly;
    poly.Add(p1);
    poly.Add(p2);
    poly.Add(p3);
    poly.Close();

    TopoDS_Wire wire = poly.Wire();
    BRepBuilderAPI_MakeFace face_maker(wire, Standard_True);
    sewing.Add(face_maker.Face());
  }

  sewing.Perform();
  TopoDS_Shape shape = sewing.SewedShape();

  if (shape.IsNull()) {
    RCLCPP_WARN(logger_, "Sewing failed, creating compound of individual faces");
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);

    int faces_added = 0;
    for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
      const Poly_Triangle & tri = triangulation->Triangle(i);
      Standard_Integer n1, n2, n3;
      tri.Get(n1, n2, n3);

      gp_Pnt p1 = triangulation->Node(n1);
      gp_Pnt p2 = triangulation->Node(n2);
      gp_Pnt p3 = triangulation->Node(n3);

      if (p1.IsEqual(p2, Precision::Confusion()) ||
        p2.IsEqual(p3, Precision::Confusion()) ||
        p3.IsEqual(p1, Precision::Confusion()))
      {
        continue;
      }

      BRepBuilderAPI_MakePolygon poly;
      poly.Add(p1);
      poly.Add(p2);
      poly.Add(p3);
      poly.Close();

      TopoDS_Wire wire = poly.Wire();
      BRepBuilderAPI_MakeFace face_maker(wire, Standard_True);
      builder.Add(compound, face_maker.Face());
      faces_added++;
    }

    if (faces_added == 0) {
      throw std::runtime_error(
        "Failed to create any valid faces from STL file: " + stl_path);
    }

    RCLCPP_WARN(logger_, "Created compound with %d faces", faces_added);
    shape = compound;
  }

  if (!translation.isZero() || !normalized_rotation.isApprox(Eigen::Quaterniond::Identity())) {
    shape = apply_transform(shape, translation, normalized_rotation);
  }

  if (config_.auto_triangulate) {
    triangulate(shape);
  }

  RCLCPP_INFO(logger_, "STL file loaded successfully");
  return shape;
}

TopoDS_Shape ShapeLoader::load_from_urdf(const std::string & urdf_path)
{
  RCLCPP_INFO(logger_, "Loading URDF file: %s", urdf_path.c_str());

  std::ifstream file(urdf_path);
  if (!file.good()) {
    throw std::runtime_error("URDF file not found: " + urdf_path);
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  return load_from_urdf_string(buffer.str());
}

std::string ShapeLoader::resolve_package_url(const std::string & url) const
{
  const std::string package_prefix = "package://";

  if (url.find(package_prefix) != 0) {
    return url;
  }

  std::string path_without_prefix = url.substr(package_prefix.length());

  size_t slash_pos = path_without_prefix.find('/');
  if (slash_pos == std::string::npos) {
    throw std::runtime_error("Invalid package:// URL format (missing '/'): " + url);
  }

  if (slash_pos == 0) {
    throw std::runtime_error("Invalid package:// URL format (empty package name): " + url);
  }

  std::string package_name = path_without_prefix.substr(0, slash_pos);
  std::string relative_path = path_without_prefix.substr(slash_pos + 1);

  if (relative_path.empty()) {
    throw std::runtime_error("Invalid package:// URL format (empty relative path): " + url);
  }

  if (relative_path.find("..") != std::string::npos) {
    throw std::runtime_error(
      "Invalid package:// URL (contains '..' traversal): " + url);
  }

  std::string package_share_dir;
  try {
    package_share_dir = ament_index_cpp::get_package_share_directory(package_name);
  } catch (const std::exception & e) {
    throw std::runtime_error(
      "Failed to resolve package '" + package_name + "': " + e.what());
  }

  return package_share_dir + "/" + relative_path;
}

TopoDS_Shape ShapeLoader::load_from_urdf_string(const std::string & urdf_string)
{
  RCLCPP_DEBUG(logger_, "Parsing URDF string for collision geometry");

  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf_string.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to parse URDF XML");
  }

  tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if (!robot) {
    throw std::runtime_error("No <robot> element found in URDF");
  }

  std::vector<TopoDS_Shape> collision_shapes;

  for (tinyxml2::XMLElement * link = robot->FirstChildElement("link");
    link != nullptr;
    link = link->NextSiblingElement("link"))
  {
    const char * link_name = link->Attribute("name");
    RCLCPP_DEBUG(logger_, "Processing link: %s", link_name ? link_name : "unnamed");

    for (tinyxml2::XMLElement * collision = link->FirstChildElement("collision");
      collision != nullptr;
      collision = collision->NextSiblingElement("collision"))
    {
      tinyxml2::XMLElement * geometry = collision->FirstChildElement("geometry");
      if (!geometry) {
        continue;
      }

      Eigen::Vector3d origin_xyz = Eigen::Vector3d::Zero();
      Eigen::Quaterniond origin_rot = Eigen::Quaterniond::Identity();

      tinyxml2::XMLElement * origin = collision->FirstChildElement("origin");
      if (origin) {
        const char * xyz_str = origin->Attribute("xyz");
        if (xyz_str) {
          double x, y, z;
          if (sscanf(xyz_str, "%lf %lf %lf", &x, &y, &z) == 3) {
            origin_xyz = Eigen::Vector3d(x, y, z);
          }
        }

        const char * rpy_str = origin->Attribute("rpy");
        if (rpy_str) {
          double r, p, yaw;
          if (sscanf(rpy_str, "%lf %lf %lf", &r, &p, &yaw) == 3) {
            // URDF uses fixed-axis ZYX Euler angles (roll-pitch-yaw)
            origin_rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
            origin_rot.normalize();
          } else {
            RCLCPP_WARN(logger_, "Failed to parse rpy attribute: %s", rpy_str);
          }
        }
      }

      TopoDS_Shape shape;
      bool shape_created = false;

      if (tinyxml2::XMLElement * box = geometry->FirstChildElement("box")) {
        const char * size_str = box->Attribute("size");
        if (size_str) {
          double sx, sy, sz;
          if (sscanf(size_str, "%lf %lf %lf", &sx, &sy, &sz) == 3) {
            if (sx > 0 && sy > 0 && sz > 0) {
              shape = make_box(Eigen::Vector3d(sx, sy, sz), origin_xyz, origin_rot);
              shape_created = true;
              RCLCPP_DEBUG(logger_, "Created box: %.3f x %.3f x %.3f", sx, sy, sz);
            } else {
              RCLCPP_WARN(logger_, "Invalid box dimensions (must be positive): %.3f x %.3f x %.3f",
                sx, sy, sz);
            }
          } else {
            RCLCPP_WARN(logger_, "Failed to parse box size attribute: %s", size_str);
          }
        }
      } else if (tinyxml2::XMLElement * cylinder = geometry->FirstChildElement("cylinder")) {
        double radius = 0.0, length = 0.0;
        cylinder->QueryDoubleAttribute("radius", &radius);
        cylinder->QueryDoubleAttribute("length", &length);
        if (radius > 0 && length > 0) {
          shape = make_cylinder(radius, length, origin_xyz, origin_rot);
          shape_created = true;
          RCLCPP_DEBUG(logger_, "Created cylinder: r=%.3f, h=%.3f", radius, length);
        } else {
          RCLCPP_WARN(logger_, "Invalid cylinder dimensions (must be positive): r=%.3f, h=%.3f",
            radius, length);
        }
      } else if (tinyxml2::XMLElement * sphere = geometry->FirstChildElement("sphere")) {
        double radius = 0.0;
        sphere->QueryDoubleAttribute("radius", &radius);
        if (radius > 0) {
          shape = make_sphere(radius, origin_xyz);
          shape_created = true;
          RCLCPP_DEBUG(logger_, "Created sphere: r=%.3f", radius);
        } else {
          RCLCPP_WARN(logger_, "Invalid sphere radius (must be positive): r=%.3f", radius);
        }
      } else if (tinyxml2::XMLElement * mesh = geometry->FirstChildElement("mesh")) {
        const char * filename = mesh->Attribute("filename");
        if (filename) {
          // TODO(@silanus23): Implement mesh loading from URDF
          // - Support STL and STEP/STP formats
          // - Handle scale attribute: mesh->Attribute("scale")
          // - Use resolve_package_url() to resolve package:// URLs
          // - Apply origin_xyz and origin_rot transforms after loading
          RCLCPP_WARN(logger_, "Mesh geometry not yet implemented, skipping: %s", filename);
          continue;
        }
      }

      if (shape_created && !shape.IsNull()) {
        collision_shapes.push_back(shape);
      }
    }
  }

  if (collision_shapes.empty()) {
    throw std::runtime_error("No collision geometry found in URDF");
  }

  RCLCPP_INFO(logger_, "Loaded %zu collision shapes from URDF", collision_shapes.size());

  return combine_shapes(collision_shapes);
}

TopoDS_Shape ShapeLoader::make_box(
  const Eigen::Vector3d & dimensions,
  const Eigen::Vector3d & center,
  const Eigen::Quaterniond & rotation)
{
  validate_dimensions(dimensions, "make_box");
  validate_vector3d(center, "make_box");
  Eigen::Quaterniond normalized_rotation =
    validate_and_normalize_quaternion(rotation, "make_box");

  RCLCPP_DEBUG(logger_, "Creating box: %.3f x %.3f x %.3f at (%.3f, %.3f, %.3f)",
    dimensions.x(), dimensions.y(), dimensions.z(),
    center.x(), center.y(), center.z());

  gp_Pnt corner(
    -dimensions.x() / 2.0,
    -dimensions.y() / 2.0,
    -dimensions.z() / 2.0);

  TopoDS_Shape box = BRepPrimAPI_MakeBox(
    corner,
    dimensions.x(),
    dimensions.y(),
    dimensions.z()).Shape();

  box = apply_transform(box, center, normalized_rotation);

  if (config_.auto_triangulate) {
    triangulate(box);
  }

  return box;
}

TopoDS_Shape ShapeLoader::make_cylinder(
  double radius,
  double height,
  const Eigen::Vector3d & center,
  const Eigen::Quaterniond & rotation)
{
  validate_positive_dimension(radius, "radius", "make_cylinder");
  validate_positive_dimension(height, "height", "make_cylinder");
  validate_vector3d(center, "make_cylinder");
  Eigen::Quaterniond normalized_rotation =
    validate_and_normalize_quaternion(rotation, "make_cylinder");

  RCLCPP_DEBUG(logger_, "Creating cylinder: r=%.3f, h=%.3f at (%.3f, %.3f, %.3f)",
    radius, height, center.x(), center.y(), center.z());

  gp_Ax2 axis(gp_Pnt(0, 0, -height / 2.0), gp_Dir(0, 0, 1));
  TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, radius, height).Shape();

  cylinder = apply_transform(cylinder, center, normalized_rotation);

  if (config_.auto_triangulate) {
    triangulate(cylinder);
  }

  return cylinder;
}

TopoDS_Shape ShapeLoader::make_sphere(
  double radius,
  const Eigen::Vector3d & center)
{
  validate_positive_dimension(radius, "radius", "make_sphere");
  validate_vector3d(center, "make_sphere");

  RCLCPP_DEBUG(logger_, "Creating sphere: r=%.3f at (%.3f, %.3f, %.3f)",
    radius, center.x(), center.y(), center.z());

  gp_Pnt center_pnt(center.x(), center.y(), center.z());
  TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(center_pnt, radius).Shape();

  if (config_.auto_triangulate) {
    triangulate(sphere);
  }

  return sphere;
}

TopoDS_Shape ShapeLoader::make_ground_plane(
  double size_x,
  double size_y,
  double z_position,
  double thickness,
  double center_x,
  double center_y)
{
  validate_positive_dimension(size_x, "size_x", "make_ground_plane");
  validate_positive_dimension(size_y, "size_y", "make_ground_plane");
  validate_positive_dimension(thickness, "thickness", "make_ground_plane");
  if (!std::isfinite(z_position)) {
    throw std::invalid_argument(
      "make_ground_plane: z_position is not finite: " + std::to_string(z_position));
  }

  RCLCPP_DEBUG(logger_, "Creating ground plane: %.2f x %.2f at z=%.3f center=(%.3f, %.3f)",
    size_x, size_y, z_position, center_x, center_y);

  Eigen::Vector3d dimensions(size_x, size_y, thickness);
  Eigen::Vector3d center(center_x, center_y, z_position - thickness / 2.0);

  return make_box(dimensions, center, Eigen::Quaterniond::Identity());
}

void ShapeLoader::triangulate(TopoDS_Shape & shape) const
{
  try {
    BRepMesh_IncrementalMesh mesher(
      shape,
      config_.linear_deflection,
      Standard_False,
      config_.angular_deflection);
  } catch (const Standard_Failure & e) {
    throw std::runtime_error(
      "Failed to triangulate shape: " + std::string(e.GetMessageString()));
  } catch (...) {
    throw std::runtime_error("Failed to triangulate shape: unknown error");
  }
}

TopoDS_Shape ShapeLoader::apply_transform(
  const TopoDS_Shape & shape,
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & rotation) const
{
  try {
    gp_Trsf transform = create_transform(translation, rotation);
    BRepBuilderAPI_Transform transformer(shape, transform, Standard_True);
    if (!transformer.IsDone()) {
      throw std::runtime_error("Failed to apply transform: IsDone() returned false");
    }
    return transformer.Shape();
  } catch (const Standard_Failure & e) {
    throw std::runtime_error(
      "Failed to apply transform: " + std::string(e.GetMessageString()));
  } catch (...) {
    throw std::runtime_error("Failed to apply transform: unknown error");
  }
}

TopoDS_Shape ShapeLoader::combine_shapes(const std::vector<TopoDS_Shape> & shapes) const
{
  if (shapes.empty()) {
    throw std::runtime_error("Cannot combine empty list of shapes");
  }

  if (shapes.size() == 1) {
    return shapes[0];
  }

  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);

  size_t valid_shapes = 0;
  for (const auto & shape : shapes) {
    if (!shape.IsNull()) {
      builder.Add(compound, shape);
      valid_shapes++;
    }
  }

  if (valid_shapes == 0) {
    throw std::runtime_error("Cannot combine shapes: all shapes are null");
  }

  return compound;
}

gp_Trsf ShapeLoader::create_transform(
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & rotation) const
{
  validate_and_normalize_quaternion(rotation, "create_transform");
  return geometry::create_transform(translation, rotation);
}

}  // namespace io
}  // namespace hold_and_weld_gripper_sampler
