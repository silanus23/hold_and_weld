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


#include "hold_and_weld_application/kinematics/urdf_parser.hpp"

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hold_and_weld
{
namespace kinematics
{

URDFParser::URDFParser()
{
  RCLCPP_DEBUG(rclcpp::get_logger("urdf_parser"), "URDFParser constructed");
}

URDFParser::~URDFParser()
{
}

ParsedChain URDFParser::extract_joint_chain(
  const std::string & urdf_path,
  const std::string & base_link,
  const std::string & tip_link)
{
  auto logger = rclcpp::get_logger("urdf_parser");
  RCLCPP_INFO(logger, "Extracting joint chain from URDF");
  RCLCPP_DEBUG(logger, "URDF path: %s", urdf_path.c_str());
  RCLCPP_DEBUG(logger, "Base link: %s, Tip link: %s", base_link.c_str(), tip_link.c_str());

  if (urdf_path.empty()) {
    RCLCPP_ERROR(logger, "URDF path cannot be empty");
    throw std::invalid_argument("URDF path cannot be empty");
  }
  if (base_link.empty()) {
    RCLCPP_ERROR(logger, "Base link name cannot be empty");
    throw std::invalid_argument("Base link name cannot be empty");
  }
  if (tip_link.empty()) {
    RCLCPP_ERROR(logger, "Tip link name cannot be empty");
    throw std::invalid_argument("Tip link name cannot be empty");
  }

  auto model = load_urdf(urdf_path);

  auto link_chain = build_link_chain(model, base_link, tip_link);
  auto joints = extract_joints_from_chain(link_chain);
  auto tool_transform = extract_tool_transform(link_chain, joints);
  validate_chain(joints);

  RCLCPP_INFO(logger, "Successfully extracted chain with %zu actuated joints", joints.size());

  ParsedChain result;
  result.actuated_joints = joints;
  result.tool_transform = tool_transform;
  result.base_link = base_link;
  result.tip_link = tip_link;

  return result;
}

std::string URDFParser::resolve_package_path(const std::string & path)
{
  auto logger = rclcpp::get_logger("urdf_parser");
  if (path.find("package://") == 0) {
    std::string without_prefix = path.substr(10);
    size_t slash_pos = without_prefix.find('/');

    if (slash_pos == std::string::npos) {
      RCLCPP_ERROR(logger, "Invalid package:// URI: %s", path.c_str());
      throw std::runtime_error("Invalid package:// URI: " + path);
    }

    std::string package = without_prefix.substr(0, slash_pos);
    std::string relative = without_prefix.substr(slash_pos + 1);

    RCLCPP_DEBUG(logger, "Resolving package path: %s -> %s/%s", path.c_str(), package.c_str(),
          relative.c_str());

    try {
      std::string package_dir = ament_index_cpp::get_package_share_directory(package);
      std::string resolved = package_dir + "/" + relative;
      RCLCPP_DEBUG(logger, "Resolved to: %s", resolved.c_str());
      return resolved;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Package '%s' not found: %s", package.c_str(), e.what());
      throw std::runtime_error(
                "Package '" + package + "' not found: " + std::string(e.what()));
    }
  }

  return path;
}

urdf::ModelInterfaceSharedPtr URDFParser::load_urdf(const std::string & urdf_path)
{
  auto logger = rclcpp::get_logger("urdf_parser");
  std::string resolved_path = resolve_package_path(urdf_path);

  RCLCPP_INFO(logger, "Loading URDF from: %s", resolved_path.c_str());

  std::string urdf_string;

  if (resolved_path.substr(resolved_path.find_last_of(".") + 1) == "xacro") {
    RCLCPP_INFO(logger, "Processing xacro file: %s", resolved_path.c_str());

    // Run xacro without invoking a shell to avoid injection risks.
    // pipe_fds[0] = read end, pipe_fds[1] = write end
    int pipe_fds[2];
    if (::pipe(pipe_fds) != 0) {
      RCLCPP_ERROR(logger, "Failed to create pipe for xacro");
      throw std::runtime_error("Failed to create pipe for xacro");
    }

    pid_t pid = ::fork();
    if (pid < 0) {
      ::close(pipe_fds[0]);
      ::close(pipe_fds[1]);
      RCLCPP_ERROR(logger, "Failed to fork for xacro");
      throw std::runtime_error("Failed to fork for xacro");
    }

    if (pid == 0) {
      // Child: redirect stdout to write end of pipe, then exec xacro
      ::close(pipe_fds[0]);
      if (::dup2(pipe_fds[1], STDOUT_FILENO) < 0) {::_exit(127);}
      ::close(pipe_fds[1]);
      // Build null-terminated arg list; resolved_path must be mutable for execvp
      char arg0[] = "xacro";
      std::vector<char> path_buf(resolved_path.begin(), resolved_path.end());
      path_buf.push_back('\0');
      char * argv_exec[] = {arg0, path_buf.data(), nullptr};
      ::execvp("xacro", argv_exec);
      ::_exit(127);  // execvp failed
    }

    // Parent: close write end and read from read end
    ::close(pipe_fds[1]);
    {
      std::stringstream result;
      char buffer[256];
      ssize_t n;
      while ((n = ::read(pipe_fds[0], buffer, sizeof(buffer))) > 0) {
        result.write(buffer, n);
      }
      ::close(pipe_fds[0]);
      urdf_string = result.str();
    }

    int status = 0;
    ::waitpid(pid, &status, 0);
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      RCLCPP_ERROR(logger, "xacro failed (exit code %d) for: %s",
                   WIFEXITED(status) ? WEXITSTATUS(status) : -1, resolved_path.c_str());
      throw std::runtime_error("xacro failed for: " + resolved_path);
    }

    if (urdf_string.empty()) {
      RCLCPP_ERROR(logger, "Xacro produced empty output for: %s", resolved_path.c_str());
      throw std::runtime_error("Xacro produced empty output for: " + resolved_path);
    }
    RCLCPP_DEBUG(logger, "Xacro processed successfully (%zu bytes)", urdf_string.size());
  } else {
    std::ifstream file(resolved_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(logger, "Failed to open URDF file: %s", resolved_path.c_str());
      throw std::runtime_error("Failed to open URDF file: " + resolved_path);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    if (file.bad()) {
      RCLCPP_ERROR(logger, "I/O error while reading URDF file: %s", resolved_path.c_str());
      throw std::runtime_error("I/O error while reading URDF file: " + resolved_path);
    }
    file.close();
    urdf_string = buffer.str();
  }

  auto model = urdf::parseURDF(urdf_string);
  if (!model) {
    RCLCPP_ERROR(logger, "Failed to parse URDF from: %s", resolved_path.c_str());
    throw std::runtime_error("Failed to parse URDF from: " + resolved_path);
  }

  RCLCPP_INFO(logger, "Successfully loaded URDF model: %s", model->getName().c_str());

  return model;
}

std::vector<urdf::LinkConstSharedPtr> URDFParser::build_link_chain(
  const urdf::ModelInterfaceSharedPtr & model,
  const std::string & base_link,
  const std::string & tip_link)
{
  auto logger = rclcpp::get_logger("urdf_parser");
  RCLCPP_DEBUG(logger, "Building link chain from '%s' to '%s'", base_link.c_str(),
        tip_link.c_str());

  auto base = model->getLink(base_link);
  auto tip = model->getLink(tip_link);

  if (!base) {
    RCLCPP_ERROR(logger, "Base link '%s' not found in URDF", base_link.c_str());
    throw std::runtime_error("Base link '" + base_link + "' not found in URDF");
  }

  if (!tip) {
    RCLCPP_ERROR(logger, "Tip link '%s' not found in URDF", tip_link.c_str());
    throw std::runtime_error("Tip link '" + tip_link + "' not found in URDF");
  }

  std::vector<urdf::LinkConstSharedPtr> reverse_chain;
  auto current = tip;

  while (current && current->name != base_link) {
    reverse_chain.push_back(current);

    if (!current->parent_joint) {
      RCLCPP_ERROR(
        logger, "Cannot build chain: link '%s' has no parent joint — URDF topology may be broken",
        current->name.c_str());
      throw std::runtime_error(
                "Cannot build chain: link '" + current->name + "' has no parent joint");
    }

    std::string parent_name = current->parent_joint->parent_link_name;
    current = model->getLink(parent_name);
  }

  if (!current) {
    RCLCPP_ERROR(logger, "Cannot build chain from '%s' to '%s'", base_link.c_str(),
          tip_link.c_str());
    throw std::runtime_error(
            "Cannot build chain from '" + base_link + "' to '" + tip_link + "'");
  }
  reverse_chain.push_back(base);

  std::vector<urdf::LinkConstSharedPtr> chain(reverse_chain.rbegin(), reverse_chain.rend());

  RCLCPP_INFO(logger, "Built link chain with %zu links", chain.size());

  return chain;
}

std::vector<JointInfo> URDFParser::extract_joints_from_chain(
  const std::vector<urdf::LinkConstSharedPtr> & link_chain)
{
  auto logger = rclcpp::get_logger("urdf_parser");
  RCLCPP_DEBUG(logger, "Extracting joints from chain with %zu links", link_chain.size());

  std::vector<JointInfo> actuated_joints;

  if (link_chain.size() < 2) {
    RCLCPP_ERROR(logger, "Chain must have at least 2 links, got %zu", link_chain.size());
    throw std::runtime_error("Chain must have at least 2 links");
  }

  // Accumulate fixed joint transforms
  Eigen::Isometry3d accumulated_fixed = Eigen::Isometry3d::Identity();

  for (size_t i = 1; i < link_chain.size(); ++i) {
    auto child_link = link_chain[i];
    auto joint = child_link->parent_joint;

    if (!joint) {
      throw std::runtime_error(
                "Link '" + child_link->name + "' has no parent joint");
    }

    Eigen::Isometry3d joint_transform =
      urdf_pose_to_eigen(joint->parent_to_joint_origin_transform);

    if (joint->type == urdf::Joint::FIXED) {
      RCLCPP_DEBUG(logger, "Joint '%s' is fixed, accumulating transform", joint->name.c_str());
      accumulated_fixed = accumulated_fixed * joint_transform;
      continue;
    }

    JointInfo info;
    info.name = joint->name;

    RCLCPP_DEBUG(logger, "Processing actuated joint: %s", joint->name.c_str());

    info.origin_transform = accumulated_fixed * joint_transform;
    accumulated_fixed = Eigen::Isometry3d::Identity();

    info.axis = Eigen::Vector3d(joint->axis.x, joint->axis.y, joint->axis.z);
    double axis_norm = info.axis.norm();

    if (axis_norm < 1e-6) {
      throw std::runtime_error(
                "Joint '" + joint->name + "' has zero-length axis");
    }

    info.axis /= axis_norm;

    info.is_revolute = (joint->type == urdf::Joint::REVOLUTE ||
      joint->type == urdf::Joint::CONTINUOUS);

    if (!info.is_revolute && joint->type != urdf::Joint::PRISMATIC) {
      throw std::runtime_error(
                "Joint '" + joint->name + "' has unsupported type. "
                "Welder robots must use REVOLUTE, CONTINUOUS, or PRISMATIC joints.");
    }

    if (joint->limits) {
      info.q_min = joint->limits->lower;
      info.q_max = joint->limits->upper;
      RCLCPP_DEBUG(logger, "Joint '%s' limits: [%.3f, %.3f]",
                        joint->name.c_str(), info.q_min, info.q_max);
    } else {
      if (info.is_revolute) {
        info.q_min = -M_PI;
        info.q_max = M_PI;
        RCLCPP_WARN(logger, "Joint '%s' is continuous, using default limits [-π, π]",
                           joint->name.c_str());
      } else {
        RCLCPP_ERROR(logger, "Prismatic joint '%s' must have limits defined in URDF",
                            joint->name.c_str());
        throw std::runtime_error(
                    "Prismatic joint '" + joint->name +
                    "' must have limits defined in URDF");
      }
    }

    actuated_joints.push_back(info);
  }

  RCLCPP_INFO(logger, "Extracted %zu actuated joints", actuated_joints.size());

  return actuated_joints;
}

Eigen::Isometry3d URDFParser::extract_tool_transform(
  const std::vector<urdf::LinkConstSharedPtr> & link_chain,
  const std::vector<JointInfo> & actuated_joints)
{
  auto logger = rclcpp::get_logger("urdf_parser");
  RCLCPP_DEBUG(logger, "Extracting tool transform from chain");

  size_t last_actuated_idx = 0;

  for (size_t i = 1; i < link_chain.size(); ++i) {
    auto joint = link_chain[i]->parent_joint;
    if (!joint) {continue;}

    bool is_actuated = std::any_of(
            actuated_joints.begin(),
            actuated_joints.end(),
      [&](const JointInfo & info) {
        return info.name == joint->name;
            });

    if (is_actuated) {
      last_actuated_idx = i;
    }
  }

  Eigen::Isometry3d tool_transform = Eigen::Isometry3d::Identity();

  size_t num_tool_transforms = 0;
  for (size_t i = last_actuated_idx + 1; i < link_chain.size(); ++i) {
    auto joint = link_chain[i]->parent_joint;
    if (!joint) {continue;}

    if (joint->type == urdf::Joint::FIXED) {
      RCLCPP_DEBUG(logger, "Adding fixed transform from joint '%s' to tool transform",
                        joint->name.c_str());
      Eigen::Isometry3d T =
        urdf_pose_to_eigen(joint->parent_to_joint_origin_transform);
      tool_transform = tool_transform * T;
      num_tool_transforms++;
    } else {
      RCLCPP_ERROR(logger, "Found actuated joint '%s' after expected end of actuated chain",
                        joint->name.c_str());
      throw std::runtime_error(
                "Found actuated joint '" + joint->name +
                "' after expected end of actuated chain. "
                "All actuated joints must come before tool fixed transforms.");
    }
  }

  RCLCPP_INFO(logger, "Tool transform extracted from %zu fixed joints", num_tool_transforms);

  return tool_transform;
}

Eigen::Isometry3d URDFParser::urdf_pose_to_eigen(const urdf::Pose & pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  transform.translation() << pose.position.x, pose.position.y, pose.position.z;

  Eigen::Quaterniond q(
    pose.rotation.w,
    pose.rotation.x,
    pose.rotation.y,
    pose.rotation.z);
  transform.linear() = q.toRotationMatrix();

  return transform;
}

void URDFParser::validate_chain(const std::vector<JointInfo> & joints)
{
  auto logger = rclcpp::get_logger("urdf_parser");
  RCLCPP_DEBUG(logger, "Validating joint chain with %zu joints", joints.size());

  if (joints.empty()) {
    RCLCPP_ERROR(logger, "Joint chain is empty");
    throw std::runtime_error("Joint chain is empty");
  }

  if (joints.size() != 6) {
    RCLCPP_ERROR(logger, "Expected exactly 6 joints for welder robot, found %zu", joints.size());
    throw std::runtime_error(
            "Expected exactly 6 joints for welder robot, found " +
            std::to_string(joints.size()) + ". "
            "Welder robots must have 6 revolute joints.");
  }

  for (const auto & joint : joints) {
    if (!joint.is_revolute) {
      RCLCPP_ERROR(logger, "Joint '%s' is not revolute", joint.name.c_str());
      throw std::runtime_error(
                "Joint '" + joint.name + "' is not revolute. "
                "Welder robots must use revolute joints only.");
    }

    double axis_norm = joint.axis.norm();
    if (std::abs(axis_norm - 1.0) > 1e-3) {
      RCLCPP_ERROR(logger, "Joint '%s' axis is not normalized: norm = %.6f",
                        joint.name.c_str(), axis_norm);
      throw std::runtime_error(
                "Joint '" + joint.name + "' axis is not normalized: norm = " +
                std::to_string(axis_norm));
    }

    if (joint.q_min >= joint.q_max) {
      RCLCPP_ERROR(logger, "Joint '%s' has invalid limits: [%.3f, %.3f]",
                        joint.name.c_str(), joint.q_min, joint.q_max);
      throw std::runtime_error(
                "Joint '" + joint.name + "' has invalid limits: [" +
                std::to_string(joint.q_min) + ", " +
                std::to_string(joint.q_max) + "]");
    }

    if (std::abs(joint.q_max - joint.q_min) > 2.1 * M_PI) {
      RCLCPP_WARN(logger, "Joint '%s' has very large range: [%.3f, %.3f] (%.3f rad)",
                       joint.name.c_str(), joint.q_min, joint.q_max,
                       joint.q_max - joint.q_min);
    }
  }

  RCLCPP_INFO(logger, "Joint chain validation successful");
}

urdf::ModelInterfaceSharedPtr URDFParser::load_urdf_from_string(const std::string & urdf_string)
{
  auto logger = rclcpp::get_logger("urdf_parser");
  RCLCPP_DEBUG(logger, "Parsing URDF from string (%zu bytes)", urdf_string.size());

  auto model = urdf::parseURDF(urdf_string);
  if (!model) {
    RCLCPP_ERROR(logger, "Failed to parse URDF from provided string");
    throw std::runtime_error("Failed to parse URDF from string");
  }

  RCLCPP_INFO(logger, "Successfully loaded URDF model: %s", model->getName().c_str());
  return model;
}

ParsedChain URDFParser::extract_joint_chain_from_string(
  const std::string & urdf_string,
  const std::string & base_link,
  const std::string & tip_link)
{
  auto logger = rclcpp::get_logger("urdf_parser");
  RCLCPP_INFO(logger, "Extracting joint chain from ROS 2 URDF string");
  RCLCPP_DEBUG(logger, "Base link: %s, Tip link: %s", base_link.c_str(), tip_link.c_str());

  if (urdf_string.empty()) {
    throw std::invalid_argument("URDF string cannot be empty");
  }

  if (base_link.empty()) {
    throw std::invalid_argument("Base link name cannot be empty");
  }

  if (tip_link.empty()) {
    throw std::invalid_argument("Tip link name cannot be empty");
  }

  auto model = load_urdf_from_string(urdf_string);
  auto link_chain = build_link_chain(model, base_link, tip_link);
  auto joints = extract_joints_from_chain(link_chain);
  auto tool_transform = extract_tool_transform(link_chain, joints);
  validate_chain(joints);

  ParsedChain result;
  result.actuated_joints = joints;
  result.tool_transform = tool_transform;
  result.base_link = base_link;
  result.tip_link = tip_link;

  return result;
}

}  // namespace kinematics
}  // namespace hold_and_weld
