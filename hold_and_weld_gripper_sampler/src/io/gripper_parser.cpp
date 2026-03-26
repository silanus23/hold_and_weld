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

#include "hold_and_weld_gripper_sampler/io/gripper_parser.hpp"

#include <tinyxml2.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRep_Builder.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace io
{

TopoDS_Shape configure_gripper(const ParsedGripper & gripper, double grip_distance)
{
  const double finger_travel = std::max(
    0.0,
    std::min(
      (grip_distance - gripper.min_opening) / 2.0,
      (gripper.max_opening - gripper.min_opening) / 2.0));

  gp_Trsf f1_trsf;
  f1_trsf.SetTranslation(
    gp_Vec(
      gripper.finger_1_axis.x(),
      gripper.finger_1_axis.y(),
      gripper.finger_1_axis.z()) * finger_travel);

  gp_Trsf f2_trsf;
  f2_trsf.SetTranslation(
    gp_Vec(
      gripper.finger_2_axis.x(),
      gripper.finger_2_axis.y(),
      gripper.finger_2_axis.z()) * finger_travel);

  TopoDS_Shape finger_1_opened =
    BRepBuilderAPI_Transform(gripper.finger_1, f1_trsf, Standard_True).Shape();
  TopoDS_Shape finger_2_opened =
    BRepBuilderAPI_Transform(gripper.finger_2, f2_trsf, Standard_True).Shape();

  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  builder.Add(compound, finger_1_opened);
  builder.Add(compound, finger_2_opened);
  builder.Add(compound, gripper.base);

  return compound;
}

namespace
{

/**
 * @brief Convert RPY (roll, pitch, yaw) to gp_Trsf rotation
 */
gp_Trsf rpy_to_transform(double roll, double pitch, double yaw)
{
  // ZYX Euler angles (aerospace convention)
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  double qw = cr * cp * cy + sr * sp * sy;
  double qx = sr * cp * cy - cr * sp * sy;
  double qy = cr * sp * cy + sr * cp * sy;
  double qz = cr * cp * sy - sr * sp * cy;

  gp_Quaternion quat(qx, qy, qz, qw);
  gp_Trsf transform;
  transform.SetRotation(quat);
  return transform;
}

/**
 * @brief Parse origin element and return combined transform
 */
gp_Trsf parse_origin(tinyxml2::XMLElement * origin)
{
  gp_Trsf transform;

  if (!origin) {
    return transform;  // Identity
  }

  // Parse xyz translation
  const char * xyz_str = origin->Attribute("xyz");
  if (xyz_str) {
    double x, y, z;
    if (std::sscanf(xyz_str, "%lf %lf %lf", &x, &y, &z) == 3) {
      try {
        transform.SetTranslation(gp_Vec(x, y, z));
      } catch (...) {
        // Silently use identity if translation fails
      }
    }
  }

  // Parse rpy rotation
  const char * rpy_str = origin->Attribute("rpy");
  if (rpy_str) {
    double roll, pitch, yaw;
    if (std::sscanf(rpy_str, "%lf %lf %lf", &roll, &pitch, &yaw) == 3) {
      try {
        gp_Trsf rot_transform = rpy_to_transform(roll, pitch, yaw);
        transform = transform * rot_transform;
      } catch (...) {
        // Silently use identity if rotation fails
      }
    }
  }

  return transform;
}

/**
 * @brief Create OCCT shape from geometry element
 */
TopoDS_Shape create_shape_from_geometry(tinyxml2::XMLElement * geometry)
{
  if (!geometry) {
    throw std::runtime_error("Geometry element is null");
  }

  try {
    // Try each geometry type
    if (tinyxml2::XMLElement * box = geometry->FirstChildElement("box")) {
      const char * size_str = box->Attribute("size");
      if (!size_str) {
        throw std::runtime_error("Box missing 'size' attribute");
      }

      double x, y, z;
      if (std::sscanf(size_str, "%lf %lf %lf", &x, &y, &z) != 3) {
        throw std::runtime_error("Failed to parse box size");
      }

      // URDF: box centered at origin
      gp_Pnt corner(-x / 2.0, -y / 2.0, -z / 2.0);
      return BRepPrimAPI_MakeBox(corner, x, y, z).Shape();

    } else if (tinyxml2::XMLElement * cylinder = geometry->FirstChildElement("cylinder")) {
      const char * radius_str = cylinder->Attribute("radius");
      const char * length_str = cylinder->Attribute("length");

      if (!radius_str || !length_str) {
        throw std::runtime_error("Cylinder missing 'radius' or 'length' attribute");
      }

      double radius, length;
      if (std::sscanf(radius_str, "%lf", &radius) != 1 ||
        std::sscanf(length_str, "%lf", &length) != 1)
      {
        throw std::runtime_error("Failed to parse cylinder parameters");
      }

      // URDF: cylinder centered at origin, axis along Z
      gp_Ax2 axis(gp_Pnt(0, 0, -length / 2.0), gp_Dir(0, 0, 1));
      return BRepPrimAPI_MakeCylinder(axis, radius, length).Shape();

    } else if (tinyxml2::XMLElement * sphere = geometry->FirstChildElement("sphere")) {
      const char * radius_str = sphere->Attribute("radius");

      if (!radius_str) {
        throw std::runtime_error("Sphere missing 'radius' attribute");
      }

      double radius;
      if (std::sscanf(radius_str, "%lf", &radius) != 1) {
        throw std::runtime_error("Failed to parse sphere radius");
      }

      return BRepPrimAPI_MakeSphere(radius).Shape();

    } else if (geometry->FirstChildElement("mesh")) {
      throw std::runtime_error("Mesh geometry not supported");
    } else {
      throw std::runtime_error("Unknown or unsupported geometry type");
    }
  } catch (const std::exception & e) {
    throw std::runtime_error("Error creating shape from geometry: " + std::string(e.what()));
  } catch (...) {
    throw std::runtime_error("Unknown error creating shape from geometry");
  }
}

}  // namespace

Eigen::Vector3d GripperParser::parse_xyz(const std::string & xyz_str)
{
  double x, y, z;
  if (std::sscanf(xyz_str.c_str(), "%lf %lf %lf", &x, &y, &z) != 3) {
    throw std::runtime_error("Failed to parse xyz string: " + xyz_str);
  }
  return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d GripperParser::parse_rpy(const std::string & rpy_str)
{
  double r, p, y;
  if (std::sscanf(rpy_str.c_str(), "%lf %lf %lf", &r, &p, &y) != 3) {
    throw std::runtime_error("Failed to parse rpy string: " + rpy_str);
  }
  return Eigen::Vector3d(r, p, y);
}

TopoDS_Shape GripperParser::extract_link_shape(
  const std::string & urdf_string,
  const std::string & link_name)
{
  tinyxml2::XMLDocument doc;
  doc.Parse(urdf_string.c_str());

  if (doc.Error()) {
    throw std::runtime_error("Failed to parse URDF: " + std::string(doc.ErrorStr()));
  }

  tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if (!robot) {
    throw std::runtime_error("No <robot> element found");
  }

  for (tinyxml2::XMLElement * link = robot->FirstChildElement("link");
    link != nullptr;
    link = link->NextSiblingElement("link"))
  {
    const char * name = link->Attribute("name");
    if (!name || link_name != name) {
      continue;
    }

    tinyxml2::XMLElement * collision = link->FirstChildElement("collision");
    if (!collision) {
      throw std::runtime_error("Link '" + link_name + "' has no collision geometry");
    }

    tinyxml2::XMLElement * geometry = collision->FirstChildElement("geometry");
    if (!geometry) {
      throw std::runtime_error("Link '" + link_name + "' collision has no geometry element");
    }

    try {
      // Create base shape
      TopoDS_Shape shape = create_shape_from_geometry(geometry);

      // Apply origin transform if present
      tinyxml2::XMLElement * origin = collision->FirstChildElement("origin");
      if (origin) {
        gp_Trsf transform = parse_origin(origin);
        BRepBuilderAPI_Transform transformer(shape, transform, Standard_True);
        shape = transformer.Shape();
      }

      return shape;
    } catch (const std::exception & e) {
      throw std::runtime_error(
        "Error extracting shape for link '" + link_name + "': " + std::string(e.what()));
    } catch (...) {
      throw std::runtime_error(
        "Unknown error extracting shape for link '" + link_name + "'");
    }
  }

  throw std::runtime_error("Link not found: " + link_name);
}

Eigen::Vector3d GripperParser::extract_joint_axis(
  const std::string & urdf_string,
  const std::string & joint_name)
{
  tinyxml2::XMLDocument doc;
  doc.Parse(urdf_string.c_str());

  if (doc.Error()) {
    throw std::runtime_error("Failed to parse URDF: " + std::string(doc.ErrorStr()));
  }

  tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if (!robot) {
    throw std::runtime_error("No <robot> element found");
  }

  // Find the joint with matching name
  for (tinyxml2::XMLElement * joint = robot->FirstChildElement("joint");
    joint != nullptr;
    joint = joint->NextSiblingElement("joint"))
  {
    const char * name = joint->Attribute("name");
    if (!name || joint_name != name) {
      continue;
    }

    // Verify it's a prismatic joint
    const char * type = joint->Attribute("type");
    if (!type || std::string(type) != "prismatic") {
      throw std::runtime_error(
              "Joint '" + joint_name + "' is not prismatic (type: " +
              (type ? type : "unknown") + ")");
    }

    // Extract axis
    tinyxml2::XMLElement * axis_elem = joint->FirstChildElement("axis");
    if (!axis_elem) {
      // Default axis is Z
      return Eigen::Vector3d(0.0, 0.0, 1.0);
    }

    const char * xyz_str = axis_elem->Attribute("xyz");
    if (!xyz_str) {
      return Eigen::Vector3d(0.0, 0.0, 1.0);
    }

    return parse_xyz(xyz_str).normalized();
  }

  throw std::runtime_error("Joint not found: " + joint_name);
}

gp_Trsf GripperParser::extract_joint_origin(
  const std::string & urdf_string,
  const std::string & joint_name)
{
  tinyxml2::XMLDocument doc;
  doc.Parse(urdf_string.c_str());

  if (doc.Error()) {
    throw std::runtime_error("Failed to parse URDF: " + std::string(doc.ErrorStr()));
  }

  tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if (!robot) {
    throw std::runtime_error("No <robot> element found");
  }

  for (tinyxml2::XMLElement * joint = robot->FirstChildElement("joint");
    joint != nullptr;
    joint = joint->NextSiblingElement("joint"))
  {
    const char * name = joint->Attribute("name");
    if (!name || joint_name != name) {
      continue;
    }

    tinyxml2::XMLElement * origin = joint->FirstChildElement("origin");
    return parse_origin(origin);
  }

  throw std::runtime_error("Joint not found: " + joint_name);
}

std::pair<double, double> GripperParser::extract_joint_limits(
  const std::string & urdf_string,
  const std::string & joint_name)
{
  tinyxml2::XMLDocument doc;
  doc.Parse(urdf_string.c_str());

  if (doc.Error()) {
    throw std::runtime_error("Failed to parse URDF: " + std::string(doc.ErrorStr()));
  }

  tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if (!robot) {
    throw std::runtime_error("No <robot> element found");
  }

  // Find the joint with matching name
  for (tinyxml2::XMLElement * joint = robot->FirstChildElement("joint");
    joint != nullptr;
    joint = joint->NextSiblingElement("joint"))
  {
    const char * name = joint->Attribute("name");
    if (!name || joint_name != name) {
      continue;
    }

    tinyxml2::XMLElement * limit = joint->FirstChildElement("limit");
    if (!limit) {
      throw std::runtime_error("Joint '" + joint_name + "' has no limit element");
    }

    const char * lower_str = limit->Attribute("lower");
    const char * upper_str = limit->Attribute("upper");

    if (!lower_str || !upper_str) {
      throw std::runtime_error("Joint '" + joint_name + "' limit missing lower/upper");
    }

    double lower, upper;
    if (std::sscanf(lower_str, "%lf", &lower) != 1 ||
      std::sscanf(upper_str, "%lf", &upper) != 1)
    {
      throw std::runtime_error("Failed to parse joint limits for: " + joint_name);
    }

    return {lower, upper};
  }

  throw std::runtime_error("Joint not found: " + joint_name);
}

ParsedGripper GripperParser::parse_from_urdf_string(const std::string & urdf_string)
{
  ParsedGripper gripper;

  tinyxml2::XMLDocument doc;
  doc.Parse(urdf_string.c_str());

  if (doc.Error()) {
    throw std::runtime_error("Failed to parse URDF: " + std::string(doc.ErrorStr()));
  }

  tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if (!robot) {
    throw std::runtime_error("No <robot> element found");
  }

  // Find gripper_metadata element (can be inside a macro or at robot level)
  tinyxml2::XMLElement * metadata = nullptr;

  // First check directly under robot
  metadata = robot->FirstChildElement("gripper_metadata");

  // If not found, search in xacro:macro elements
  if (!metadata) {
    for (tinyxml2::XMLElement * macro = robot->FirstChildElement("xacro:macro");
      macro != nullptr && !metadata;
      macro = macro->NextSiblingElement("xacro:macro"))
    {
      metadata = macro->FirstChildElement("gripper_metadata");
    }
  }

  // Also try without xacro namespace
  if (!metadata) {
    for (tinyxml2::XMLElement * macro = robot->FirstChildElement("macro");
      macro != nullptr && !metadata;
      macro = macro->NextSiblingElement("macro"))
    {
      metadata = macro->FirstChildElement("gripper_metadata");
    }
  }

  if (!metadata) {
    throw std::runtime_error(
            "No <gripper_metadata> element found in URDF. "
            "Please add gripper convention metadata to your URDF.");
  }

  tinyxml2::XMLElement * base_link = metadata->FirstChildElement("base_link");
  if (!base_link) {
    throw std::runtime_error("gripper_metadata missing <base_link> element");
  }

  const char * base_name = base_link->Attribute("name");
  if (!base_name) {
    throw std::runtime_error("base_link missing 'name' attribute");
  }
  gripper.base_link_name = base_name;

  std::string finger_1_link, finger_1_joint;
  std::string finger_2_link, finger_2_joint;

  for (tinyxml2::XMLElement * finger = metadata->FirstChildElement("finger");
    finger != nullptr;
    finger = finger->NextSiblingElement("finger"))
  {
    const char * finger_id_str = finger->Attribute("finger_id");
    const char * link_attr = finger->Attribute("link");
    const char * joint_attr = finger->Attribute("joint");

    if (!finger_id_str || !link_attr || !joint_attr) {
      throw std::runtime_error("finger element missing required attributes");
    }

    int finger_id;
    if (std::sscanf(finger_id_str, "%d", &finger_id) != 1) {
      throw std::runtime_error("Invalid finger_id: " + std::string(finger_id_str));
    }

    if (finger_id == 1) {
      finger_1_link = link_attr;
      finger_1_joint = joint_attr;
    } else if (finger_id == 2) {
      finger_2_link = link_attr;
      finger_2_joint = joint_attr;
    } else {
      throw std::runtime_error("Invalid finger_id (expected 1 or 2): " + std::to_string(finger_id));
    }
  }

  if (finger_1_link.empty() || finger_2_link.empty()) {
    throw std::runtime_error("gripper_metadata must define exactly 2 fingers");
  }

  gripper.finger_1_link_name = finger_1_link;
  gripper.finger_2_link_name = finger_2_link;
  gripper.finger_1_joint_name = finger_1_joint;
  gripper.finger_2_joint_name = finger_2_joint;

  tinyxml2::XMLElement * gripper_type = metadata->FirstChildElement("gripper_type");
  gripper.gripper_type = gripper_type ? gripper_type->GetText() : "parallel";

  tinyxml2::XMLElement * tcp_offset = metadata->FirstChildElement("tcp_offset");
  if (tcp_offset) {
    const char * xyz_str = tcp_offset->Attribute("xyz");
    const char * rpy_str = tcp_offset->Attribute("rpy");

    gripper.tcp_offset = xyz_str ? parse_xyz(xyz_str) : Eigen::Vector3d::Zero();
    gripper.tcp_rpy = rpy_str ? parse_rpy(rpy_str) : Eigen::Vector3d::Zero();
  } else {
    gripper.tcp_offset = Eigen::Vector3d::Zero();
    gripper.tcp_rpy = Eigen::Vector3d::Zero();
  }

  gripper.base = extract_link_shape(urdf_string, gripper.base_link_name);

  // Extract finger shapes and apply joint origin transforms
  // The joint origin positions the finger link relative to the base
  TopoDS_Shape finger_1_raw = extract_link_shape(urdf_string, gripper.finger_1_link_name);
  TopoDS_Shape finger_2_raw = extract_link_shape(urdf_string, gripper.finger_2_link_name);

  gp_Trsf finger_1_joint_origin = extract_joint_origin(urdf_string, gripper.finger_1_joint_name);
  gp_Trsf finger_2_joint_origin = extract_joint_origin(urdf_string, gripper.finger_2_joint_name);

  try {
    gripper.finger_1 = BRepBuilderAPI_Transform(finger_1_raw, finger_1_joint_origin,
          Standard_True).Shape();
    gripper.finger_2 = BRepBuilderAPI_Transform(finger_2_raw, finger_2_joint_origin,
          Standard_True).Shape();
  } catch (Standard_Failure & e) {
    throw std::runtime_error(
      std::string("OCCT error transforming finger shapes: ") + e.GetMessageString());
  }

  gripper.finger_1_axis = extract_joint_axis(urdf_string, gripper.finger_1_joint_name);
  gripper.finger_2_axis = extract_joint_axis(urdf_string, gripper.finger_2_joint_name);

  auto [f1_lower, f1_upper] = extract_joint_limits(urdf_string, gripper.finger_1_joint_name);
  auto [f2_lower, f2_upper] = extract_joint_limits(urdf_string, gripper.finger_2_joint_name);

  // For parallel grippers, opening = sum of both finger movements
  // Assuming symmetric gripper: min_opening = 2 * min_single, max_opening = 2 * max_single
  // Joint limits represent single finger travel from closed position
  double min_single_travel = std::min(f1_lower, f2_lower);
  double max_single_travel = std::max(f1_upper, f2_upper);

  // Total opening is twice the single finger travel (both fingers move)
  gripper.min_opening = 2.0 * min_single_travel;
  gripper.max_opening = 2.0 * max_single_travel;

  return gripper;
}

ParsedGripper GripperParser::parse_from_urdf_file(const std::string & urdf_path)
{
  std::ifstream file(urdf_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open URDF file: " + urdf_path);
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return parse_from_urdf_string(buffer.str());
}

ParsedGripper GripperParser::parse_from_xacro_file(
  const std::string & xacro_path,
  const std::string & xacro_args)
{
  // Build xacro command
  std::string command = "xacro " + xacro_path;
  if (!xacro_args.empty()) {
    command += " " + xacro_args;
  }

  // Execute xacro and capture output
  std::array<char, 4096> buffer;
  std::string result;

  // Use lambda wrapper to avoid -Wignored-attributes warning with pclose
  auto pipe_deleter = [](FILE * fp) {if (fp) {pclose(fp);}};
  std::unique_ptr<FILE, decltype(pipe_deleter)> pipe(popen(command.c_str(), "r"), pipe_deleter);
  if (!pipe) {
    throw std::runtime_error("Failed to execute xacro command");
  }

  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }

  if (result.empty()) {
    throw std::runtime_error("xacro produced no output for: " + xacro_path);
  }

  return parse_from_urdf_string(result);
}

}  // namespace io
}  // namespace hold_and_weld_gripper_sampler
