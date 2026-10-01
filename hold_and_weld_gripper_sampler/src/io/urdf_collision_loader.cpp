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

#include "hold_and_weld_gripper_sampler/io/urdf_collision_loader.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <gp_Ax2.hxx>
#include <gp_Vec.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>

#include "hold_and_weld_gripper_sampler/geometry/occt_utils.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace io
{

namespace
{

// Below this OCCT's primitive builders fail or produce degenerate solids.
constexpr double kMinPrimitiveSize = 1e-6;

std::string name_of(const tinyxml2::XMLElement * element)
{
  const char * name = element->Attribute("name");
  return name ? name : "<unnamed>";
}

const char * required_attribute(const tinyxml2::XMLElement * element, const char * name)
{
  const char * value = element->Attribute(name);
  if (!value) {
    throw std::runtime_error(
            std::string("<") + element->Name() + "> is missing the '" + name + "' attribute");
  }
  return value;
}

// Non-finite values are rejected here so they never reach OCCT's builders.
template<size_t N>
std::array<double, N> parse_doubles(const char * text, const std::string & what)
{
  std::array<double, N> values{};
  const char * cursor = text;
  for (size_t i = 0; i < N; ++i) {
    int consumed = 0;
    if (std::sscanf(cursor, "%lf%n", &values[i], &consumed) != 1 || !std::isfinite(values[i])) {
      throw std::runtime_error(
              what + ": expected " + std::to_string(N) + " finite numbers, got '" + text + "'");
    }
    cursor += consumed;
  }
  return values;
}

double parse_size(const tinyxml2::XMLElement * element, const char * name)
{
  const double value = parse_doubles<1>(
    required_attribute(element, name), std::string("<") + element->Name() + " " + name + ">")[0];
  if (value <= kMinPrimitiveSize) {
    throw std::runtime_error(
            std::string("<") + element->Name() + "> '" + name + "' must be positive, got " +
            std::to_string(value));
  }
  return value;
}

// Returns a null shape and sets `unsupported` for geometry we don't build; malformed
// geometry throws instead.
TopoDS_Shape make_primitive(const tinyxml2::XMLElement * geometry, std::string & unsupported)
{
  if (const auto * box = geometry->FirstChildElement("box")) {
    const auto size = parse_doubles<3>(required_attribute(box, "size"), "<box size>");
    for (double s : size) {
      if (s <= kMinPrimitiveSize) {
        throw std::runtime_error("<box> size must be positive in every axis");
      }
    }
    // URDF centres the box; MakeBox takes a corner.
    return BRepPrimAPI_MakeBox(
      gp_Pnt(-size[0] / 2.0, -size[1] / 2.0, -size[2] / 2.0), size[0], size[1], size[2]).Shape();
  }
  if (const auto * cylinder = geometry->FirstChildElement("cylinder")) {
    const double radius = parse_size(cylinder, "radius");
    const double length = parse_size(cylinder, "length");
    // URDF centres the cylinder on its axis; OCCT builds it from z=0 up.
    return BRepPrimAPI_MakeCylinder(
      gp_Ax2(gp_Pnt(0, 0, -length / 2.0), gp_Dir(0, 0, 1)), radius, length).Shape();
  }
  if (const auto * sphere = geometry->FirstChildElement("sphere")) {
    return BRepPrimAPI_MakeSphere(parse_size(sphere, "radius")).Shape();
  }
  if (const auto * mesh = geometry->FirstChildElement("mesh")) {
    // TODO(silanus23): Load STL/STEP meshes: honour the scale attribute and resolve
    // package:// URLs (ShapeLoader::resolve_package_url does that).
    const char * filename = mesh->Attribute("filename");
    unsupported = std::string("Mesh geometry not supported (") +
      (filename ? filename : "no filename") + ")";
    return TopoDS_Shape();
  }
  const tinyxml2::XMLElement * child = geometry->FirstChildElement();
  unsupported = std::string("unsupported geometry type '") + (child ? child->Name() : "none") +
    "'";
  return TopoDS_Shape();
}

// Each element is moved by placement * its own <collision><origin>.
TopoDS_Shape build_link_collision(
  const tinyxml2::XMLElement * link,
  const gp_Trsf & placement,
  std::vector<std::string> & skipped)
{
  const std::string link_name = name_of(link);
  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  bool built_any = false;

  for (const auto * collision = link->FirstChildElement("collision");
    collision != nullptr;
    collision = collision->NextSiblingElement("collision"))
  {
    try {
      const auto * geometry = collision->FirstChildElement("geometry");
      if (!geometry) {
        throw std::runtime_error("<collision> has no <geometry>");
      }
      std::string unsupported;
      TopoDS_Shape shape = make_primitive(geometry, unsupported);
      if (shape.IsNull()) {
        skipped.push_back(link_name + ": " + unsupported);
        continue;
      }
      const gp_Trsf transform = placement * parse_origin(collision->FirstChildElement("origin"));
      BRepBuilderAPI_Transform transformer(shape, transform, Standard_True);
      if (!transformer.IsDone()) {
        throw std::runtime_error("placing the collision shape failed");
      }
      builder.Add(compound, transformer.Shape());
      built_any = true;
    } catch (const Standard_Failure & e) {
      throw std::runtime_error(
              "URDF link '" + link_name + "': OCCT error: " + e.GetMessageString());
    } catch (const std::exception & e) {
      throw std::runtime_error("URDF link '" + link_name + "': " + e.what());
    }
  }

  return built_any ? TopoDS_Shape(compound) : TopoDS_Shape();
}

}  // namespace

gp_Trsf parse_origin(const tinyxml2::XMLElement * origin)
{
  gp_Trsf transform;
  if (!origin) {
    return transform;
  }

  if (const char * xyz = origin->Attribute("xyz")) {
    const auto t = parse_doubles<3>(xyz, "<origin xyz>");
    transform.SetTranslation(gp_Vec(t[0], t[1], t[2]));
  }
  if (const char * rpy = origin->Attribute("rpy")) {
    const auto r = parse_doubles<3>(rpy, "<origin rpy>");
    gp_Trsf rotation;
    rotation.SetRotation(geometry::rpy_to_quaternion(r[0], r[1], r[2]));
    transform = transform * rotation;
  }
  return transform;
}

TopoDS_Shape link_collision_shape(
  const tinyxml2::XMLElement * link,
  std::vector<std::string> & skipped)
{
  return build_link_collision(link, gp_Trsf(), skipped);
}

UrdfCollisionModel load_urdf_collision(const std::string & urdf_string)
{
  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf_string.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to parse URDF XML: " + std::string(doc.ErrorStr()));
  }
  const tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if (!robot) {
    throw std::runtime_error("No <robot> element found in URDF");
  }

  // Document order is kept: GeometryMapper's surface IDs follow it.
  std::vector<const tinyxml2::XMLElement *> link_order;
  std::map<std::string, const tinyxml2::XMLElement *> links;
  for (const auto * link = robot->FirstChildElement("link"); link != nullptr;
    link = link->NextSiblingElement("link"))
  {
    link_order.push_back(link);
    if (!links.emplace(required_attribute(link, "name"), link).second) {
      throw std::runtime_error("URDF declares link '" + name_of(link) + "' twice");
    }
  }

  struct ParentJoint
  {
    std::string parent;
    gp_Trsf origin;
  };
  std::map<std::string, ParentJoint> parent_of;
  for (const auto * joint = robot->FirstChildElement("joint"); joint != nullptr;
    joint = joint->NextSiblingElement("joint"))
  {
    const std::string joint_name = name_of(joint);
    const auto * parent = joint->FirstChildElement("parent");
    const auto * child = joint->FirstChildElement("child");
    if (!parent || !child) {
      throw std::runtime_error("URDF joint '" + joint_name + "' needs <parent> and <child>");
    }
    const std::string parent_link = required_attribute(parent, "link");
    const std::string child_link = required_attribute(child, "link");
    if (!links.count(parent_link) || !links.count(child_link)) {
      throw std::runtime_error(
              "URDF joint '" + joint_name + "' names a link that is not declared");
    }
    if (!parent_of.emplace(
        child_link, ParentJoint{parent_link, parse_origin(joint->FirstChildElement("origin"))})
      .second)
    {
      throw std::runtime_error("URDF link '" + child_link + "' has more than one parent joint");
    }
  }

  UrdfCollisionModel model;
  for (const auto * link : link_order) {
    const std::string link_name = name_of(link);
    // Walk up to the root, prepending each joint: root_T_link = J_1 * ... * J_n.
    gp_Trsf link_to_root;
    std::string current = link_name;
    for (size_t depth = 0; parent_of.count(current); ++depth) {
      if (depth >= links.size()) {
        throw std::runtime_error("URDF joint tree has a cycle through link '" + link_name + "'");
      }
      const ParentJoint & joint = parent_of.at(current);
      link_to_root = joint.origin * link_to_root;
      current = joint.parent;
    }

    TopoDS_Shape shape = build_link_collision(link, link_to_root, model.skipped);
    if (!shape.IsNull()) {
      model.links.push_back({link_name, shape, link_to_root});
    }
  }
  return model;
}

}  // namespace io
}  // namespace hold_and_weld_gripper_sampler
