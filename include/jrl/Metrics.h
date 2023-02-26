#pragma once
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>

#include "jrl/Alignment.h"
#include "jrl/Dataset.h"
#include "jrl/Results.h"

namespace jrl {
// TODO (dan) DOCUMENT
namespace metrics {

namespace internal {

/** POSE ERRORS
 * A method for computing the error between two trajectory objects (POSES/POINTS/VECTORS)
 * returns translation error, rotation error
 */

// For any linear object, the translation is itself
template <class T>
std::pair<double, double> squaredPoseError(T est, T ref) {
  return std::make_pair((est - ref).squaredNorm(), 0);
}

// Pose Specializations
template <>
std::pair<double, double> squaredPoseError<gtsam::Pose3>(gtsam::Pose3 est, gtsam::Pose3 ref) {
  gtsam::Pose3 error_pose = ref.inverse().compose(est);
  double angle_error = error_pose.rotation().axisAngle().second;
  return std::make_pair(error_pose.translation().squaredNorm(), angle_error * angle_error);
}
template <>
std::pair<double, double> squaredPoseError<gtsam::Pose2>(gtsam::Pose2 est, gtsam::Pose2 ref) {
  gtsam::Pose2 error_pose = ref.inverse().compose(est);
  double angle_error = error_pose.theta();
  return std::make_pair(error_pose.translation().squaredNorm(), angle_error * angle_error);
}

}  // namespace internal

template <class POSE_TYPE>
boost::optional<std::pair<double, double>> computeATE(char rid, Dataset dataset, Results results,
                                                      bool with_scale = false) {
  // We have groundtruth so we can compute ATE
  if (dataset.containsGroundTruth()) {
    gtsam::Values ref = dataset.groundTruth(rid);
    gtsam::Values est = results.robot_solutions[rid].values;
    gtsam::Values aligned_est = alignment::align<POSE_TYPE>(est, ref, with_scale);

    double squared_translation_error = 0.0;
    double squared_rotation_error = 0.0;
    for (auto& key : ref.keys()) {
      std::pair<double, double> squared_pose_error =
          internal::squaredPoseError<POSE_TYPE>(aligned_est.at<POSE_TYPE>(key), ref.at<POSE_TYPE>(key));

      squared_translation_error += squared_pose_error.first;
      squared_rotation_error += squared_pose_error.second;
    }

    // Return the RMSE of the pose errors
    return std::make_pair(std::sqrt(squared_translation_error / ref.size()),
                          std::sqrt(squared_rotation_error / ref.size()));

  }
  // No ground truth, cannot compute ATE
  else {
    return boost::none;
  }
}

/* Compute the Cartesian Product of vector of vectors
 * Cite: https://stackoverflow.com/questions/5279051/how-can-i-create-cartesian-product-of-vector-of-vectors
 */
template <typename T>
std::vector<std::vector<T>> cartesianProduct(const std::vector<std::vector<T>>& input) {
  std::vector<std::vector<T>> s = {{}};
  for (const auto& u : input) {
    std::vector<std::vector<T>> r;
    for (const auto& x : s) {
      for (const auto y : u) {
        r.push_back(x);
        r.back().push_back(y);
      }
    }
    s = std::move(r);
  }
  return s;
}

double computeMeanResidual(Dataset dataset, Results results) {
  double graph_residual = 0.0;
  for (auto& rid : dataset.robots()) {
    for (auto& entry : dataset.measurements(rid)) {
      for (auto& factor : entry.measurements) {
        // Accumulate all variable owners
        gtsam::KeyVector keys = factor->keys();
        std::vector<std::vector<char>> variable_owners;
        for (auto& key : keys) {
          variable_owners.push_back(std::vector<char>());
          for (auto& owner_id : dataset.robots()) {
            if (results.robot_solutions[owner_id].values.exists(key)) {
              variable_owners.back().push_back(owner_id);
            }
          }
        }

        // Compute the Cartesian Product
        std::vector<std::vector<char>> cart_prod = cartesianProduct<char>(variable_owners);

        // For each robot assignment comput residual and accumulate
        double factor_total_residual = 0.0;
        for (std::vector<char>& assignment : cart_prod) {
          gtsam::Values assignment_values;
          for (size_t i = 0; i < assignment.size(); i++) {
            assignment_values.insert(keys[i], results.robot_solutions[assignment[i]].values.at(keys[i]));
          }
          factor_total_residual += factor->error(assignment_values);
        }
        graph_residual += factor_total_residual / cart_prod.size();
      }
    }
  }
  return graph_residual;
}

}  // namespace metrics

}  // namespace jrl