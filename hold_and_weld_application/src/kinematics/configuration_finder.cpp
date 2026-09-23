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

#include "hold_and_weld_application/kinematics/configuration_finder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace hold_and_weld
{
namespace kinematics
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

std::vector<double> to_std_vector(const ConfigurationFinder::Vector6d & q)
{
  return std::vector<double>(q.data(), q.data() + q.size());
}

Eigen::Isometry3d interpolate(
  const Eigen::Isometry3d & a, const Eigen::Isometry3d & b, double t)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = (1.0 - t) * a.translation() + t * b.translation();
  out.linear() = Eigen::Quaterniond(a.rotation())
    .slerp(t, Eigen::Quaterniond(b.rotation())).toRotationMatrix();
  return out;
}

double rotation_angle(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  return Eigen::AngleAxisd(a.rotation().transpose() * b.rotation()).angle();
}

// Signed angle from v1 to v2 about axis, ignoring components along axis.
double signed_angle(
  const Eigen::Vector3d & v1, const Eigen::Vector3d & v2, const Eigen::Vector3d & axis)
{
  const Eigen::Vector3d p1 = v1 - axis * axis.dot(v1);
  const Eigen::Vector3d p2 = v2 - axis * axis.dot(v2);
  return std::atan2(axis.dot(p1.cross(p2)), p1.dot(p2));
}
}  // namespace

std::string to_string(WalkFailure failure)
{
  switch (failure) {
    case WalkFailure::kNone: return "none";
    case WalkFailure::kIkFailed: return "IK did not converge";
    case WalkFailure::kJointLimit: return "joint limit";
    case WalkFailure::kLowManipulability: return "low manipulability";
    case WalkFailure::kJointStep: return "joint step (branch jump)";
  }
  return "unknown";
}

Eigen::Isometry3d standoff_pose(const Eigen::Isometry3d & ref, double approach_offset)
{
  Eigen::Isometry3d out = ref;
  out.translation() = ref.translation() + ref.rotation() * Eigen::Vector3d(0, 0, approach_offset);
  return out;
}

ConfigurationFinder::ConfigurationFinder(
  std::shared_ptr<KinematicsSolver> kin_solver,
  std::shared_ptr<CeresIKSolver> ik_solver,
  const ConfigurationFinderParams & params)
: kin_solver_(std::move(kin_solver)),
  ik_solver_(std::move(ik_solver)),
  params_(params)
{
  if (!kin_solver_ || !ik_solver_) {
    throw std::invalid_argument("ConfigurationFinder: kin_solver and ik_solver must not be null");
  }

  auto require = [](bool ok, const char * name, double value, const char * rule) {
      if (!ok || !std::isfinite(value)) {
        throw std::invalid_argument(
          std::string("ConfigurationFinderParams.") + name + " must be " + rule + ", got " +
          std::to_string(value));
      }
    };
  const auto & p = params_;
  require(p.path_step > 0.0, "path_step", p.path_step, "positive");
  require(p.path_step_rot > 0.0, "path_step_rot", p.path_step_rot, "positive");
  require(p.max_joint_step > 0.0, "max_joint_step", p.max_joint_step, "positive");
  require(p.tol_pos > 0.0, "tol_pos", p.tol_pos, "positive");
  require(p.tol_rot > 0.0, "tol_rot", p.tol_rot, "positive");
  require(
    p.manipulability_threshold >= 0.0, "manipulability_threshold", p.manipulability_threshold,
    "non-negative");
  require(p.branch_seed_weight >= 0.0, "branch_seed_weight", p.branch_seed_weight, "non-negative");
  require(p.walk_seed_weight >= 0.0, "walk_seed_weight", p.walk_seed_weight, "non-negative");
  require(p.w_limit_margin >= 0.0, "w_limit_margin", p.w_limit_margin, "non-negative");
  require(p.w_manipulability >= 0.0, "w_manipulability", p.w_manipulability, "non-negative");
  require(p.w_home >= 0.0, "w_home", p.w_home, "non-negative");
  require(p.dedupe_epsilon >= 0.0, "dedupe_epsilon", p.dedupe_epsilon, "non-negative");
}

bool ConfigurationFinder::within_limits(const Vector6d & q) const
{
  const auto & limits = kin_solver_->joint_limits();
  for (int i = 0; i < 6; ++i) {
    // Written as "not inside" so a NaN joint counts as out of limits.
    if (!(limits[i].first <= q(i) && q(i) <= limits[i].second)) {
      return false;
    }
  }
  return true;
}

double ConfigurationFinder::limit_margin(const Vector6d & q) const
{
  const auto & limits = kin_solver_->joint_limits();
  double margin = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 6; ++i) {
    margin = std::min({margin, q(i) - limits[i].first, limits[i].second - q(i)});
  }
  return margin;
}

std::vector<ConfigurationFinder::Vector6d> ConfigurationFinder::expand_solution(
  const Vector6d & q) const
{
  const auto & limits = kin_solver_->joint_limits();

  Vector6d flipped = q;
  flipped(3) += M_PI;
  flipped(4) = -q(4);
  flipped(5) += M_PI;

  // Every value of base + 2*pi*k inside [lo, hi], ascending.
  auto copies = [](double base, double lo, double hi) {
      std::vector<double> out;
      const double first = base + kTwoPi * std::ceil((lo - base) / kTwoPi);
      for (double v = first; v <= hi; v += kTwoPi) {
        out.push_back(v);
      }
      return out;
    };

  std::vector<Vector6d> expanded;
  for (const Vector6d & base : {q, flipped}) {
    for (double q4 : copies(base(3), limits[3].first, limits[3].second)) {
      for (double q6 : copies(base(5), limits[5].first, limits[5].second)) {
        Vector6d candidate = base;
        candidate(3) = q4;
        candidate(5) = q6;
        // ceil() can land a copy a rounding error away from base; keep base exact.
        if (std::abs(q4 - base(3)) < 1e-9) {candidate(3) = base(3);}
        if (std::abs(q6 - base(5)) < 1e-9) {candidate(5) = base(5);}
        if (within_limits(candidate)) {
          expanded.push_back(candidate);
        }
      }
    }
  }
  return expanded;
}

ConfigurationFinder::Vector6d ConfigurationFinder::elbow_mirror_seed(const Vector6d & q) const
{
  // Reflect the elbow (J3 origin) across the shoulder -> wrist-centre line, then turn
  // J3 so the wrist centre returns to where it was. Only a seed: Ceres does the rest.
  std::vector<Eigen::Vector3d> origins, axes;
  kin_solver_->compute_joint_axes(to_std_vector(q), origins, axes);
  const Eigen::Vector3d shoulder = origins[1];
  const Eigen::Vector3d elbow = origins[2];
  const Eigen::Vector3d wrist = origins[4];

  const Eigen::Vector3d u = (wrist - shoulder).normalized();
  const Eigen::Vector3d foot = shoulder + u * u.dot(elbow - shoulder);
  const Eigen::Vector3d mirrored_elbow = 2.0 * foot - elbow;

  Vector6d seed = q;
  seed(1) += signed_angle(elbow - shoulder, mirrored_elbow - shoulder, axes[1]);

  kin_solver_->compute_joint_axes(to_std_vector(seed), origins, axes);
  seed(2) += signed_angle(origins[4] - origins[2], wrist - origins[2], axes[2]);
  return seed;
}

std::vector<ConfigurationFinder::Vector6d> ConfigurationFinder::generate_candidates(
  const Eigen::Isometry3d & pose,
  const Vector6d & q_home) const
{
  const auto & limits = kin_solver_->joint_limits();
  auto clamp_to_limits = [&limits](Vector6d q) {
      for (int i = 0; i < 6; ++i) {
        q(i) = std::clamp(q(i), limits[i].first, limits[i].second);
      }
      return q;
    };

  Vector6d shoulder_flipped = q_home;
  shoulder_flipped(0) += (q_home(0) + M_PI <= limits[0].second) ? M_PI : -M_PI;

  const std::vector<Vector6d> seeds = {
    clamp_to_limits(q_home),
    clamp_to_limits(shoulder_flipped),
    clamp_to_limits(elbow_mirror_seed(q_home)),
    clamp_to_limits(elbow_mirror_seed(shoulder_flipped)),
  };

  std::vector<Vector6d> candidates;
  for (const auto & seed : seeds) {
    Vector6d solution;
    if (!ik_solver_->solve(pose, seed, solution, params_.tol_pos, params_.tol_rot,
        params_.branch_seed_weight))
    {
      continue;
    }
    for (const auto & q : expand_solution(solution)) {
      const bool duplicate = std::any_of(
        candidates.begin(), candidates.end(), [&](const Vector6d & existing) {
          return (existing - q).cwiseAbs().maxCoeff() <= params_.dedupe_epsilon;
        });
      if (!duplicate) {
        candidates.push_back(q);
      }
    }
  }
  return candidates;
}

std::vector<Eigen::Isometry3d> ConfigurationFinder::pilz_path(
  const std::vector<Eigen::Isometry3d> & seam,
  const std::string & segment_type,
  const Eigen::Isometry3d & approach_pose) const
{
  std::vector<Eigen::Isometry3d> path = {approach_pose};
  if (seam.empty()) {
    return path;
  }

  auto samples_for = [this](double distance, double angle) {
      return static_cast<int>(std::max(
               std::ceil(distance / params_.path_step - 1e-9),
               std::ceil(angle / params_.path_step_rot - 1e-9)));
    };

  // Straight line with slerped orientation, as Pilz LIN does.
  auto append_line = [&](const Eigen::Isometry3d & a, const Eigen::Isometry3d & b) {
      const int n = samples_for(
        (b.translation() - a.translation()).norm(), rotation_angle(a, b));
      for (int k = 1; k <= n; ++k) {
        path.push_back(interpolate(a, b, static_cast<double>(k) / n));
      }
    };

  // Plunge: LIN from the standoff onto the seam start.
  append_line(approach_pose, seam.front());

  const Eigen::Isometry3d & start = seam.front();
  const Eigen::Isometry3d & end = seam.back();

  if (segment_type == "line") {
    append_line(start, end);
  } else if (segment_type == "arc" && seam.size() >= 3) {
    // Pilz CIRC: circle through start, interim (seam[N/2], as the welder sends) and
    // goal; orientation slerped start -> goal over the path.
    const Eigen::Vector3d p0 = start.translation();
    const Eigen::Vector3d u = seam[seam.size() / 2].translation() - p0;
    const Eigen::Vector3d v = end.translation() - p0;
    const Eigen::Vector3d w = u.cross(v);
    if (w.norm() < 1e-9 * u.norm() * v.norm() || w.norm() < 1e-12) {
      append_line(start, end);  // colinear: Pilz would reject; the walk treats it as LIN
    } else {
      const Eigen::Vector3d center =
        p0 + (u.squaredNorm() * v - v.squaredNorm() * u).cross(w) / (2.0 * w.squaredNorm());
      const double radius = (p0 - center).norm();
      const Eigen::Vector3d e1 = (p0 - center) / radius;
      const Eigen::Vector3d e2 = w.normalized().cross(e1);
      const Eigen::Vector3d to_end = end.translation() - center;
      double sweep = std::atan2(to_end.dot(e2), to_end.dot(e1));
      if (sweep <= 0.0) {
        sweep += kTwoPi;
      }
      const int n = samples_for(radius * sweep, rotation_angle(start, end));
      for (int k = 1; k <= n; ++k) {
        const double t = static_cast<double>(k) / n;
        Eigen::Isometry3d p = interpolate(start, end, t);
        p.translation() = center + radius * (std::cos(t * sweep) * e1 + std::sin(t * sweep) * e2);
        path.push_back(p);
      }
      path.back().translation() = end.translation();
    }
  } else {
    // ptp / legacy / unknown types, and "arc" with fewer than 3 poses (no interim
    // point to define a circle): computeCartesianPath over the seam poses themselves.
    for (size_t i = 1; i < seam.size(); ++i) {
      append_line(seam[i - 1], seam[i]);
    }
  }
  return path;
}

WalkResult ConfigurationFinder::walk(
  const Vector6d & q_start, const std::vector<Eigen::Isometry3d> & path) const
{
  if (path.empty()) {
    throw std::invalid_argument("ConfigurationFinder::walk: path must not be empty");
  }

  WalkResult result;
  auto fail = [&result](WalkFailure failure, size_t index) {
      result.feasible = false;
      result.failure = failure;
      result.fail_index = index;
      return result;
    };

  if (!within_limits(q_start)) {
    return fail(WalkFailure::kJointLimit, 0);
  }

  // q_start must be an IK solution of path[0] (as generate_candidates() produces);
  // otherwise the first step measures a jump that Pilz would never make.
  const Eigen::Isometry3d start_pose = kin_solver_->compute_fk(to_std_vector(q_start));
  if ((start_pose.translation() - path.front().translation()).norm() > params_.tol_pos + 1e-9 ||
    rotation_angle(start_pose, path.front()) > params_.tol_rot + 1e-9)
  {
    throw std::invalid_argument(
      "ConfigurationFinder::walk: q_start is not at path[0] within tol_pos/tol_rot");
  }

  result.min_limit_margin = limit_margin(q_start);
  result.min_manipulability = kin_solver_->compute_yoshikawa_index(to_std_vector(q_start));
  if (result.min_manipulability < params_.manipulability_threshold) {
    return fail(WalkFailure::kLowManipulability, 0);
  }

  Vector6d q = q_start;
  for (size_t i = 1; i < path.size(); ++i) {
    // The seed penalty can hold IK short of a solution that needs a large joint move
    // (a wrist swing near q5 = 0). Pilz's IK would find that solution and then reject it
    // on joint velocity, so retry unpenalised and let the step check classify it.
    Vector6d q_next;
    if (!ik_solver_->solve(path[i], q, q_next, params_.tol_pos, params_.tol_rot,
        params_.walk_seed_weight) &&
      !ik_solver_->solve(path[i], q, q_next, params_.tol_pos, params_.tol_rot,
        params_.branch_seed_weight))
    {
      return fail(WalkFailure::kIkFailed, i);
    }
    if (!within_limits(q_next)) {
      return fail(WalkFailure::kJointLimit, i);
    }

    const double step = (q_next - q).cwiseAbs().maxCoeff();
    const double manipulability = kin_solver_->compute_yoshikawa_index(to_std_vector(q_next));
    result.max_joint_step = std::max(result.max_joint_step, step);
    result.min_limit_margin = std::min(result.min_limit_margin, limit_margin(q_next));
    result.min_manipulability = std::min(result.min_manipulability, manipulability);

    if (step > params_.max_joint_step) {
      return fail(WalkFailure::kJointStep, i);
    }
    if (manipulability < params_.manipulability_threshold) {
      return fail(WalkFailure::kLowManipulability, i);
    }
    q = q_next;
  }

  result.feasible = true;
  return result;
}

std::vector<Candidate> ConfigurationFinder::find(
  const std::vector<Eigen::Isometry3d> & seam,
  const std::string & segment_type,
  double approach_offset,
  const Vector6d & q_home) const
{
  if (seam.empty()) {
    return {};
  }

  const Eigen::Isometry3d approach = standoff_pose(seam.front(), approach_offset);
  const auto path = pilz_path(seam, segment_type, approach);
  const auto starts = generate_candidates(approach, q_home);

  std::vector<Candidate> candidates;
  candidates.reserve(starts.size());
  for (size_t i = 0; i < starts.size(); ++i) {
    Candidate c;
    c.index = i;
    c.q_start = starts[i];
    c.walk = walk(starts[i], path);
    c.home_distance = (starts[i] - q_home).norm();
    candidates.push_back(c);
  }

  // Min-max normalise each term over the feasible set so the weights compare like
  // with like; a term that is constant across the set contributes nothing.
  auto normalised = [&candidates](auto term) {
      double lo = std::numeric_limits<double>::infinity();
      double hi = -std::numeric_limits<double>::infinity();
      for (const auto & c : candidates) {
        if (c.walk.feasible) {
          lo = std::min(lo, term(c));
          hi = std::max(hi, term(c));
        }
      }
      return [lo, hi, term](const Candidate & c) {
               return (hi - lo) > 1e-12 ? (term(c) - lo) / (hi - lo) : 0.0;
             };
    };
  const auto margin = normalised([](const Candidate & c) {return c.walk.min_limit_margin;});
  const auto manip = normalised([](const Candidate & c) {return c.walk.min_manipulability;});
  const auto home = normalised([](const Candidate & c) {return c.home_distance;});

  for (auto & c : candidates) {
    if (c.walk.feasible) {
      c.score = params_.w_limit_margin * margin(c) +
        params_.w_manipulability * manip(c) -
        params_.w_home * home(c);
    }
  }

  std::sort(
    candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
      if (a.walk.feasible != b.walk.feasible) {
        return a.walk.feasible;
      }
      if (a.walk.feasible && a.score != b.score) {
        return a.score > b.score;
      }
      return a.index < b.index;
    });
  return candidates;
}

}  // namespace kinematics
}  // namespace hold_and_weld
