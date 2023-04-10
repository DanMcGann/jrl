#pragma once
#include "jrl/Alignment.h"
#include "jrl/Metrics.h"
namespace jrl {
namespace metrics {
/**********************************************************************************************************************/
namespace internal {
// For any linear object, the translation is itself
template <class T>
inline std::pair<double, double> squaredPoseError(T est, T ref) {
  return std::make_pair((est - ref).squaredNorm(), 0);
}

// Pose Specializations
template <>
inline std::pair<double, double> squaredPoseError<gtsam::Pose3>(gtsam::Pose3 est, gtsam::Pose3 ref) {
  gtsam::Pose3 error_pose = ref.inverse().compose(est);
  double angle_error = error_pose.rotation().axisAngle().second;
  return std::make_pair(error_pose.translation().squaredNorm(), angle_error * angle_error);
}
template <>
inline std::pair<double, double> squaredPoseError<gtsam::Pose2>(gtsam::Pose2 est, gtsam::Pose2 ref) {
  gtsam::Pose2 error_pose = ref.inverse().compose(est);
  double angle_error = error_pose.theta();
  return std::make_pair(error_pose.translation().squaredNorm(), angle_error * angle_error);
}
}  // namespace internal

/**********************************************************************************************************************/
template <class POSE_TYPE>
inline boost::optional<std::pair<double, double>> computeATE(char rid, Dataset dataset, Results results,
                                                             bool align_with_scale) {
  // We have groundtruth so we can compute ATE
  if (dataset.containsGroundTruth()) {
    gtsam::Values ref = dataset.groundTruth(rid).filter<POSE_TYPE>();
    gtsam::Values est = results.robot_solutions[rid].values.filter<POSE_TYPE>();
    gtsam::Values aligned_est = alignment::align<POSE_TYPE>(est, ref, align_with_scale);

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

/**********************************************************************************************************************/
template <class POSE_TYPE>
inline std::pair<double, double> computeSVE(Results results) {
  double squared_sve_trans = 0.0;
  double squared_sve_rot = 0.0;
  double num_shared = 0.0;

  gtsam::KeySet seen_set;
  for (auto& rid : results.robots) {
    for (auto& key : gtsam::Values(results.robot_solutions[rid].values.filter<POSE_TYPE>()).keys()) {
      // Ensure we have not already computed the error for this key
      if (seen_set.count(key) == 0) {
        seen_set.insert(key);

        // Accumulate all variable owners
        std::vector<char> variable_owners;
        for (auto& owner_id : results.robots) {
          if (results.robot_solutions[owner_id].values.exists(key)) {
            variable_owners.push_back(owner_id);
          }
        }

        // If this is a shared variable
        if (variable_owners.size() > 1) {
          num_shared += 1;
          // For each pairwise relationship
          for (size_t i = 0; i < variable_owners.size(); i++) {
            for (size_t j = i + 1; j < variable_owners.size(); j++) {  // Ignore self pair
              std::pair<double, double> squared_pose_error = internal::squaredPoseError<POSE_TYPE>(
                  results.robot_solutions[variable_owners[i]].values.at<POSE_TYPE>(key),
                  results.robot_solutions[variable_owners[j]].values.at<POSE_TYPE>(key));
              squared_sve_trans += squared_pose_error.first;
              squared_sve_rot += squared_pose_error.second;
            }
          }
        }
      }
    }
  }

  return std::make_pair(std::sqrt(squared_sve_trans / num_shared), std::sqrt(squared_sve_rot / num_shared));
}

/**********************************************************************************************************************/
template <typename T>
inline std::vector<std::vector<T>> cartesianProduct(const std::vector<std::vector<T>>& input) {
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

/**********************************************************************************************************************/
inline double computeMeanResidual(Dataset dataset, Results results) {
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

/**********************************************************************************************************************/
template <class POSE_TYPE>
inline MetricSummary computeMetricSummary(Dataset dataset, Results results, bool align_with_scale) {
  MetricSummary summary;
  summary.dataset_name = dataset.name();
  summary.robots = dataset.robots();
  summary.method_name = results.method_name;

  // Compute the Mean Residual
  summary.mean_residual = computeMeanResidual(dataset, results);

  // Compute ATE if possible
  if (dataset.containsGroundTruth()) {
    summary.robot_ate = std::map<char, std::pair<double, double>>();
    summary.total_ate = std::make_pair(0, 0);
    for (char rid : summary.robots) {
      boost::optional<std::pair<double, double>> robot_ate = computeATE<POSE_TYPE>(rid, dataset, results);
      (*summary.robot_ate)[rid] = *robot_ate;
      (*summary.total_ate) = std::make_pair((*summary.total_ate).first + (*robot_ate).first,
                                            (*summary.total_ate).second + (*robot_ate).second);
    }
  }

  // Compute the SVE is possible
  if (summary.robots.size() > 1) {
    summary.sve = computeSVE<POSE_TYPE>(results);
  }

  return summary;
}

}  // namespace metrics

}  // namespace jrl