#pragma once
#include "jrl/Alignment.h"
#include "jrl/Metrics.h"
#include "jrl/Utilities.h"

namespace jrl {

namespace metrics {
/**********************************************************************************************************************/
namespace internal {
// For any linear object, the translation is itself
template <class T>
inline PoseError squaredPoseError(T est, T ref) {
  return std::make_pair((est - ref).squaredNorm(), 0);
}

// Pose Specializations
template <>
inline PoseError squaredPoseError<gtsam::Pose3>(gtsam::Pose3 est, gtsam::Pose3 ref) {
  gtsam::Pose3 error_pose = ref.inverse().compose(est);
  double angle_error = error_pose.rotation().axisAngle().second;
  return std::make_pair(error_pose.translation().squaredNorm(), angle_error * angle_error);
}
template <>
inline PoseError squaredPoseError<gtsam::Pose2>(gtsam::Pose2 est, gtsam::Pose2 ref) {
  gtsam::Pose2 error_pose = ref.inverse().compose(est);
  double angle_error = error_pose.theta();
  return std::make_pair(error_pose.translation().squaredNorm(), angle_error * angle_error);
}
}  // namespace internal

/**********************************************************************************************************************/
template <class POSE_TYPE>
inline boost::optional<PoseError> computeATE(char rid, Dataset dataset, Results results, bool align,
                                             bool align_with_scale, bool allow_partial_results) {
  // We have groundtruth so we can compute ATE
  if (dataset.containsGroundTruth()) {
    // Grab the estimated and reference trajectories
    gtsam::Values ref = dataset.groundTruth(rid).filter<POSE_TYPE>();
    gtsam::Values est = results.robot_solutions[rid].values.filter<POSE_TYPE>();

    // If we allow partial solution get only the applicable section of the reference trajectory
    gtsam::Values filtered_ref = ref;
    if (allow_partial_results) {
      filtered_ref = ref.filter([&est](gtsam::Key key) { return est.exists(key); });
    }

    // Early exit if there are no poses
    if (filtered_ref.empty()) {
      return std::make_pair(0.0, 0.0);
    }

    // Run Umeyama alignment
    gtsam::Values aligned_est = est;
    if (align) {
      aligned_est = alignment::align<POSE_TYPE>(est, filtered_ref, align_with_scale);
    }

    double squared_translation_error = 0.0;
    double squared_rotation_error = 0.0;
    for (auto& key : filtered_ref.keys()) {
      PoseError squared_pose_error =
          internal::squaredPoseError<POSE_TYPE>(aligned_est.at<POSE_TYPE>(key), filtered_ref.at<POSE_TYPE>(key));

      squared_translation_error += squared_pose_error.first;
      squared_rotation_error += squared_pose_error.second;
    }

    // Return the RMSE of the pose errors
    return std::make_pair(std::sqrt(squared_translation_error / filtered_ref.size()),
                          std::sqrt(squared_rotation_error / filtered_ref.size()));

  }
  // No ground truth, cannot compute ATE
  else {
    return boost::none;
  }
}

/**********************************************************************************************************************/
template <class POSE_TYPE>
inline PoseError computeSVE(Results results) {
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
              PoseError squared_pose_error = internal::squaredPoseError<POSE_TYPE>(
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
inline double computeMeanResidual(Dataset dataset, Results results,
                                  std::optional<std::map<char, std::optional<size_t>>> step_idxes) {
  double graph_residual = 0.0;
  for (auto& rid : dataset.robots()) {
    auto entries = dataset.measurements(rid);
    size_t stop_idx;
    if (!step_idxes.has_value()) {
      // No step idxes provided so compute on all entries
      stop_idx = entries.size();
    } else if (step_idxes->at(rid).has_value()) {
      // Step idxes is provided and has a value for this robot
      stop_idx = *(step_idxes->at(rid));
    } else {
      // Step idxes provided but robot has not started
      stop_idx = 0;
    }
    for (size_t i = 0; i < stop_idx; i++) {
      auto entry = entries[i];
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
        std::vector<std::vector<char>> cart_prod = utils::cartesianProduct<char>(variable_owners);

        // For each robot assignment compute residual and accumulate
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
inline MetricSummary computeMetricSummary(Dataset dataset, Results results, bool align, bool align_with_scale,
                                          std::optional<std::map<char, std::optional<size_t>>> step_idxes) {
  MetricSummary summary;
  summary.dataset_name = dataset.name();
  summary.robots = dataset.robots();
  summary.method_name = results.method_name;

  // Compute the Mean Residual
  summary.mean_residual = computeMeanResidual(dataset, results, step_idxes);

  // Compute ATE if possible
  if (dataset.containsGroundTruth()) {
    summary.robot_ate = std::map<char, PoseError>();
    summary.total_ate = std::make_pair(0, 0);
    for (char rid : summary.robots) {
      boost::optional<PoseError> robot_ate =
          computeATE<POSE_TYPE>(rid, dataset, results, align, align_with_scale, step_idxes.has_value());
      (*summary.robot_ate)[rid] = *robot_ate;
      (*summary.total_ate) = std::make_pair((*summary.total_ate).first + (*robot_ate).first,
                                            (*summary.total_ate).second + (*robot_ate).second);
    }
  }

  // Compute the SVE is possible
  if (summary.robots.size() > 1) {
    summary.sve = computeSVE<POSE_TYPE>(results);
  }

  // Compute Precision+Recall if possible
  if (dataset.containsPotentialOutlierFactors() && dataset.containsOutlierFactors() && results.robot_outliers) {
    std::tie(summary.precision_recall, summary.robot_precision_recall) =
        computePrecisionRecall(dataset, results, step_idxes);
  }

  return summary;
}

}  // namespace metrics

}  // namespace jrl