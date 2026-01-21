#include "hold_and_weld_application/kinematics/urdf_parser.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>

namespace hold_and_weld {
namespace kinematics {

URDFParser::URDFParser(const rclcpp::Logger& logger)
    : logger_(logger)
{
    RCLCPP_DEBUG(logger_, "URDFParser constructed");
}

URDFParser::~URDFParser()
{
}

ParsedChain URDFParser::extract_joint_chain(
    const std::string& urdf_path,
    const std::string& base_link,
    const std::string& tip_link)
{
    RCLCPP_INFO(logger_, "Extracting joint chain from URDF");
    RCLCPP_DEBUG(logger_, "URDF path: %s", urdf_path.c_str());
    RCLCPP_DEBUG(logger_, "Base link: %s, Tip link: %s", base_link.c_str(), tip_link.c_str());
    
    // Validate inputs
    if (urdf_path.empty()) {
        RCLCPP_ERROR(logger_, "URDF path cannot be empty");
        throw std::invalid_argument("URDF path cannot be empty");
    }
    if (base_link.empty()) {
        RCLCPP_ERROR(logger_, "Base link name cannot be empty");
        throw std::invalid_argument("Base link name cannot be empty");
    }
    if (tip_link.empty()) {
        RCLCPP_ERROR(logger_, "Tip link name cannot be empty");
        throw std::invalid_argument("Tip link name cannot be empty");
    }
    
    auto model = load_urdf(urdf_path);

    auto link_chain = build_link_chain(model, base_link, tip_link);
    auto joints = extract_joints_from_chain(link_chain);
    auto tool_transform = extract_tool_transform(link_chain, joints);
    validate_chain(joints);
    
    RCLCPP_INFO(logger_, "Successfully extracted chain with %zu actuated joints", joints.size());
    
    ParsedChain result;
    result.actuated_joints = joints;
    result.tool_transform = tool_transform;
    result.base_link = base_link;
    result.tip_link = tip_link;
    
    return result;
}

std::string URDFParser::resolve_package_path(const std::string& path) {
    if (path.find("package://") == 0) {
        std::string without_prefix = path.substr(10);
        size_t slash_pos = without_prefix.find('/');
        
        if (slash_pos == std::string::npos) {
            RCLCPP_ERROR(logger_, "Invalid package:// URI: %s", path.c_str());
            throw std::runtime_error("Invalid package:// URI: " + path);
        }
        
        std::string package = without_prefix.substr(0, slash_pos);
        std::string relative = without_prefix.substr(slash_pos + 1);
        
        RCLCPP_DEBUG(logger_, "Resolving package path: %s -> %s/%s", path.c_str(), package.c_str(), relative.c_str());
        
        try {
            std::string package_dir = ament_index_cpp::get_package_share_directory(package);
            std::string resolved = package_dir + "/" + relative;
            RCLCPP_DEBUG(logger_, "Resolved to: %s", resolved.c_str());
            return resolved;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Package '%s' not found: %s", package.c_str(), e.what());
            throw std::runtime_error(
                "Package '" + package + "' not found: " + std::string(e.what()));
        }
    }
    
    return path;
}

urdf::ModelInterfaceSharedPtr URDFParser::load_urdf(const std::string& urdf_path) {
    std::string resolved_path = resolve_package_path(urdf_path);
    
    RCLCPP_INFO(logger_, "Loading URDF from: %s", resolved_path.c_str());
    
    std::ifstream file(resolved_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(logger_, "Failed to open URDF file: %s", resolved_path.c_str());
        throw std::runtime_error("Failed to open URDF file: " + resolved_path);
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    
    auto model = urdf::parseURDF(buffer.str());
    if (!model) {
        RCLCPP_ERROR(logger_, "Failed to parse URDF from: %s", resolved_path.c_str());
        throw std::runtime_error("Failed to parse URDF from: " + resolved_path);
    }
    
    RCLCPP_INFO(logger_, "Successfully loaded URDF model: %s", model->getName().c_str());
    
    return model;
}

std::vector<urdf::LinkSharedPtr> URDFParser::build_link_chain(
    const urdf::ModelInterfaceSharedPtr& model,
    const std::string& base_link,
    const std::string& tip_link)
{
    RCLCPP_DEBUG(logger_, "Building link chain from '%s' to '%s'", base_link.c_str(), tip_link.c_str());
    
    auto base = model->getLink(base_link);
    auto tip = model->getLink(tip_link);
    
    if (!base) {
        RCLCPP_ERROR(logger_, "Base link '%s' not found in URDF", base_link.c_str());
        throw std::runtime_error("Base link '" + base_link + "' not found in URDF");
    }
    
    if (!tip) {
        RCLCPP_ERROR(logger_, "Tip link '%s' not found in URDF", tip_link.c_str());
        throw std::runtime_error("Tip link '" + tip_link + "' not found in URDF");
    }
    
    std::vector<urdf::LinkSharedPtr> reverse_chain;
    auto current = tip;
    
    while (current && current->name != base_link) {
        reverse_chain.push_back(current);
        
        if (!current->parent_joint) {
            throw std::runtime_error(
                "Cannot build chain: link '" + current->name + "' has no parent joint");
        }
        
        std::string parent_name = current->parent_joint->parent_link_name;
        current = model->getLink(parent_name);
    }
    
    if (!current) {
        RCLCPP_ERROR(logger_, "Cannot build chain from '%s' to '%s'", base_link.c_str(), tip_link.c_str());
        throw std::runtime_error(
            "Cannot build chain from '" + base_link + "' to '" + tip_link + "'");
    }
    reverse_chain.push_back(base);
    
    std::vector<urdf::LinkSharedPtr> chain(reverse_chain.rbegin(), reverse_chain.rend());
    
    RCLCPP_INFO(logger_, "Built link chain with %zu links", chain.size());
    
    return chain;
}

std::vector<JointInfo> URDFParser::extract_joints_from_chain(
    const std::vector<urdf::LinkSharedPtr>& link_chain)
{
    RCLCPP_DEBUG(logger_, "Extracting joints from chain with %zu links", link_chain.size());
    
    std::vector<JointInfo> actuated_joints;
    
    if (link_chain.size() < 2) {
        RCLCPP_ERROR(logger_, "Chain must have at least 2 links, got %zu", link_chain.size());
        throw std::runtime_error("Chain must have at least 2 links");
    }
    
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
            RCLCPP_DEBUG(logger_, "Joint '%s' is fixed, accumulating transform", joint->name.c_str());
            accumulated_fixed = accumulated_fixed * joint_transform;
            continue; // Don't add to actuated_joints
        }
        
        JointInfo info;
        info.name = joint->name;
        
        RCLCPP_DEBUG(logger_, "Processing actuated joint: %s", joint->name.c_str());
        
        info.origin_transform = accumulated_fixed * joint_transform;
        accumulated_fixed = Eigen::Isometry3d::Identity();
        
        info.axis = Eigen::Vector3d(joint->axis.x, joint->axis.y, joint->axis.z);
        double axis_norm = info.axis.norm();
        
        if (axis_norm < 1e-6) {
            throw std::runtime_error(
                "Joint '" + joint->name + "' has zero-length axis");
        }
        
        info.axis /= axis_norm; // Normalize
        
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
            RCLCPP_DEBUG(logger_, "Joint '%s' limits: [%.3f, %.3f]", 
                        joint->name.c_str(), info.q_min, info.q_max);
        } else {
            if (info.is_revolute) {
                // Continuous joint - use +/- pi as default range
                info.q_min = -M_PI;
                info.q_max = M_PI;
                RCLCPP_WARN(logger_, "Joint '%s' is continuous, using default limits [-π, π]", 
                           joint->name.c_str());
            } else {
                RCLCPP_ERROR(logger_, "Prismatic joint '%s' must have limits defined in URDF", 
                            joint->name.c_str());
                throw std::runtime_error(
                    "Prismatic joint '" + joint->name + 
                    "' must have limits defined in URDF");
            }
        }
        
        actuated_joints.push_back(info);
    }
    
    RCLCPP_INFO(logger_, "Extracted %zu actuated joints", actuated_joints.size());
    
    return actuated_joints;
}

Eigen::Isometry3d URDFParser::extract_tool_transform(
    const std::vector<urdf::LinkSharedPtr>& link_chain,
    const std::vector<JointInfo>& actuated_joints)
{
    RCLCPP_DEBUG(logger_, "Extracting tool transform from chain");
    
    size_t last_actuated_idx = 0;
    
    for (size_t i = 1; i < link_chain.size(); ++i) {
        auto joint = link_chain[i]->parent_joint;
        if (!joint) continue;
        
        bool is_actuated = std::any_of(
            actuated_joints.begin(), 
            actuated_joints.end(),
            [&](const JointInfo& info) { 
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
        if (!joint) continue;
        
        if (joint->type == urdf::Joint::FIXED) {
            RCLCPP_DEBUG(logger_, "Adding fixed transform from joint '%s' to tool transform", 
                        joint->name.c_str());
            Eigen::Isometry3d T = 
                urdf_pose_to_eigen(joint->parent_to_joint_origin_transform);
            tool_transform = tool_transform * T;
            num_tool_transforms++;
        } else {
            RCLCPP_ERROR(logger_, "Found actuated joint '%s' after expected end of actuated chain", 
                        joint->name.c_str());
            throw std::runtime_error(
                "Found actuated joint '" + joint->name + 
                "' after expected end of actuated chain. "
                "All actuated joints must come before tool fixed transforms.");
        }
    }
    
    RCLCPP_INFO(logger_, "Tool transform extracted from %zu fixed joints", num_tool_transforms);
    
    return tool_transform;
}

Eigen::Isometry3d URDFParser::urdf_pose_to_eigen(const urdf::Pose& pose) {
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

void URDFParser::validate_chain(const std::vector<JointInfo>& joints) {
    RCLCPP_DEBUG(logger_, "Validating joint chain with %zu joints", joints.size());
    
    if (joints.empty()) {
        RCLCPP_ERROR(logger_, "Joint chain is empty");
        throw std::runtime_error("Joint chain is empty");
    }
    
    if (joints.size() != 6) {
        RCLCPP_ERROR(logger_, "Expected exactly 6 joints for welder robot, found %zu", joints.size());
        throw std::runtime_error(
            "Expected exactly 6 joints for welder robot, found " + 
            std::to_string(joints.size()) + ". "
            "Welder robots must have 6 revolute joints.");
    }
    
    for (const auto& joint : joints) {
        if (!joint.is_revolute) {
            RCLCPP_ERROR(logger_, "Joint '%s' is not revolute", joint.name.c_str());
            throw std::runtime_error(
                "Joint '" + joint.name + "' is not revolute. "
                "Welder robots must use revolute joints only.");
        }
        
        double axis_norm = joint.axis.norm();
        if (std::abs(axis_norm - 1.0) > 1e-3) {
            RCLCPP_ERROR(logger_, "Joint '%s' axis is not normalized: norm = %.6f", 
                        joint.name.c_str(), axis_norm);
            throw std::runtime_error(
                "Joint '" + joint.name + "' axis is not normalized: norm = " + 
                std::to_string(axis_norm));
        }
        
        if (joint.q_min >= joint.q_max) {
            RCLCPP_ERROR(logger_, "Joint '%s' has invalid limits: [%.3f, %.3f]", 
                        joint.name.c_str(), joint.q_min, joint.q_max);
            throw std::runtime_error(
                "Joint '" + joint.name + "' has invalid limits: [" + 
                std::to_string(joint.q_min) + ", " + 
                std::to_string(joint.q_max) + "]");
        }
        
        if (std::abs(joint.q_max - joint.q_min) > 2.1 * M_PI) {
            RCLCPP_WARN(logger_, "Joint '%s' has very large range: [%.3f, %.3f] (%.3f rad)", 
                       joint.name.c_str(), joint.q_min, joint.q_max, 
                       joint.q_max - joint.q_min);
        }
    }
    
    RCLCPP_INFO(logger_, "Joint chain validation successful");
}

} // namespace kinematics
} // namespace hold_and_weld