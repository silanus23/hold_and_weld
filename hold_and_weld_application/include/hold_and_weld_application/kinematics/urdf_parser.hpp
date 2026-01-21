#ifndef HOLD_AND_WELD_APPLICATION__KINEMATICS__URDF_PARSER_HPP_
#define HOLD_AND_WELD_APPLICATION__KINEMATICS__URDF_PARSER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <urdf_parser/urdf_parser.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/logger.hpp>

#include "hold_and_weld_application/kinematics/kinematics_utils.hpp"

namespace hold_and_weld {
namespace kinematics {

class URDFParser {
public:
    /**
     * @brief Construct a new URDFParser object
     * 
     * @param logger ROS2 logger for logging messages
     */
    explicit URDFParser(const rclcpp::Logger& logger);
    
    /**
     * @brief Destructor
     */
    ~URDFParser();

    /**
     * @brief Extract joint chain from URDF file
     * 
     * @param urdf_path Path to URDF file (supports package://)
     * @param base_link Starting link name (e.g., "robot2_base_link")
     * @param tip_link End link name/TCP (e.g., "robot2_wire_tip")
     * @return ParsedChain containing actuated joints and tool transform
     * @throws std::runtime_error if parsing fails or chain not found
     */
    ParsedChain extract_joint_chain(
        const std::string& urdf_path,
        const std::string& base_link,
        const std::string& tip_link);

private:
    std::string resolve_package_path(const std::string& path);
    urdf::ModelInterfaceSharedPtr load_urdf(const std::string& urdf_path);
    std::vector<urdf::LinkSharedPtr> build_link_chain(
        const urdf::ModelInterfaceSharedPtr& model,
        const std::string& base_link,
        const std::string& tip_link);
    std::vector<JointInfo> extract_joints_from_chain(
        const std::vector<urdf::LinkSharedPtr>& link_chain);
    Eigen::Isometry3d extract_tool_transform(
        const std::vector<urdf::LinkSharedPtr>& link_chain,
        const std::vector<JointInfo>& actuated_joints);
    Eigen::Isometry3d urdf_pose_to_eigen(const urdf::Pose& pose);
    void validate_chain(const std::vector<JointInfo>& joints);
    
    rclcpp::Logger logger_;
};

} // namespace kinematics
} // namespace hold_and_weld

#endif