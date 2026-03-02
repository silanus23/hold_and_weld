#ifndef HOLD_AND_WELD_APPLICATION_KINEMATICS_APPROACH_VALIDATOR_HPP
#define HOLD_AND_WELD_APPLICATION_KINEMATICS_APPROACH_VALIDATOR_HPP

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <vector>
#include "action_servers/welder_action_server.hpp"

class KinematicsSolver;
class CeresIKSolver;

namespace hold_and_weld_application::kinematics
{

using JointArray = Eigen::Array<double, 6, 1>;
using JointLimits = Eigen::Array<double, 6, 2>;

class ApproachValidator {
public:
    explicit ApproachValidator(
        std::shared_ptr<KinematicsSolver> kin_solver,
        std::shared_ptr<CeresIKSolver> ik_solver,
        double manipulatibility_threshold,
        const JointLimits& joint_limits
    );

    ~ApproachValidator() = default;

    void seamSetter(const hold_and_weld::WeldSeam& seam) { seam_ = seam; }

    /**
     * @brief The Orchestrator
     * Loops through the seam and asks the private helpers to validate the approach.
     */
    bool isApproachValid(const JointArray& q_approach);

    private:
    bool isWithinLimits(const JointArray& q) const;

    std::optional<JointArray> predictNextJoints(const JointArray& current_q, const Eigen::Isometry3d& target_pose);

    double computeManipulatibility(const JointArray& q);

    std::shared_ptr<KinematicsSolver> kinematics_solver_;
    std::shared_ptr<CeresIKSolver> ceres_ik_solver_;

    double manipulatibility_threshold_;
    JointLimits joint_limits_;
    std::optional<hold_and_weld::WeldSeam> seam_;
};

} // namespace
#endif
