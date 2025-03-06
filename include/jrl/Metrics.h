/** @brief This module implements a number of metrics that can be computed on dataset results.
 * We also provide a container for storing metrics for later processing and evaluation.
 */
#pragma once
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>

#include "jrl/Dataset.h"
#include "jrl/Results.h"

namespace jrl {

namespace metrics {
/// @brief Type helper for Pose Errors. Stored as [Translation, Rotation]
typedef std::pair<double, double> PoseError;

/// @brief Type helper for Precision Recall. Stored as [Precision, Recall]
typedef std::pair<double, double> PrecisionRecall;
}  // namespace metrics

/** @brief SImple Structure for containing a summary metrics
 *
 **/
struct MetricSummary {
  /** Members **/
  /// @brief The robots who's trajectories have been evaluated
  std::vector<char> robots;
  /// @brief The name of the dataset
  std::string dataset_name;
  /// @brief The name of the method used to generate these results
  std::string method_name;

  /// @brief The individual ATE for each robot with format: (ATE Translation, ATE Rotation)
  boost::optional<std::map<char, metrics::PoseError>> robot_ate{boost::none};
  /// @brief The total ate (sum of robot_ate's) with format: (ATE Translation, ATE Rotation)
  boost::optional<metrics::PoseError> total_ate{boost::none};
  /// @brief The joint ate after preforming a joint Uymeuama alignment with format: (ATE Translation, ATE Rotation)
  boost::optional<metrics::PoseError> joint_aligned_ate{boost::none};

  /// @brief The total Shared Variable Error (sum of robot_sve's)
  boost::optional<metrics::PoseError> sve{boost::none};

  /// @brief The Mean Residual
  boost::optional<double> mean_residual{boost::none};

  /// @brief The individual precision and recall for each robot
  boost::optional<std::map<char, metrics::PrecisionRecall>> robot_precision_recall{boost::none};
  /// @brief The total precision and recall
  boost::optional<metrics::PrecisionRecall> precision_recall{boost::none};
};

namespace metrics {

namespace internal {
/** POSE ERRORS
 * A method for computing the error between two trajectory objects (POSES/POINTS/VECTORS)
 * returns translation error, rotation error
 * Note: For any linear object, the translation is itself
 */
template <class T>
inline PoseError squaredPoseError(T est, T ref);
template <>
inline PoseError squaredPoseError<gtsam::Pose3>(gtsam::Pose3 est, gtsam::Pose3 ref);
template <>
inline PoseError squaredPoseError<gtsam::Pose2>(gtsam::Pose2 est, gtsam::Pose2 ref);
}  // namespace internal

/** @brief Computes the ATE for the specified robot.
 * This involves aligning the estimated and reference trajectories with Umeyama alignment,
 * then comparing each pose to retrieve a translation and rotation error.
 * @param rid: The robot id of the robot for which to compute ATE
 * @param dataset: The dataset containing groundtruth
 * @param results: The estimation results to be compared with the groundtruth
 * @param align_with_scale: If true aligns scale while preforming Umeyama alignment
 * @param allow_partial_results: If true we compute ATE for only the poses in results
 * @param include_shared_variables: If true we compute ATE including variables from other robots that a robot has
 * observed.
 * @returns Pair containing (ATE Translation, ATE Rotation) or boost::none if the dataset does not contain ground truth
 */
template <class POSE_TYPE>
inline boost::optional<PoseError> computeATE(char rid, Dataset dataset, Results results, bool align = true,
                                             bool align_with_scale = false, bool allow_partial_results = false,
                                             bool include_shared_variables = true);

/** @brief Computes the Joint Aligned ATE for all robots.
 * This involves aligning the joint estimated and  joint reference trajectories with Umeyama alignment,
 * then comparing each pose to retrieve a translation and rotation error.
 * @param dataset: The dataset containing groundtruth
 * @param results: The estimation results to be compared with the groundtruth
 * @param align_with_scale: If true aligns scale while preforming Umeyama alignment
 * @param allow_partial_results: If true we compute ATE for only the poses in results
 * @returns Pair containing (ATE Translation, ATE Rotation) or boost::none if the dataset does not contain ground truth
 */
template <class POSE_TYPE>
inline boost::optional<PoseError> computeJointAlignedATE(Dataset dataset, Results results,
                                                         bool align_with_scale = false,
                                                         bool allow_partial_results = false);

/** @brief Computes the SVE for the dataset
 * SVE is defined as the mean error between all combination of shared variable estimates
 * @param dataset: The dataset
 * @param results: The estimation results
 * @returns The SVE
 */
template <class POSE_TYPE>
inline PoseError computeSVE(Results results);

/** @brief Computes the mean residual of of the joint solution.
 * Because each robot can vary in their estimate of shared variables. We compute a "Mean Residual"
 * To compute the "Mean Residual", for each factor we compute the residual for each possible combination of variable
 * estimates. When all robots agree perfectly, this would match the residual without evaluating all combinations.
 * @param dataset: The dataset
 * @param results: The estimation results
 * @param step_idx: The number of entries from which we will compute the residual (used if we only have partial results)
 * if nullopt we use all entries
 * @returns The Mean Residual
 */
inline double computeMeanResidual(Dataset dataset, Results results,
                                  std::optional<std::map<char, std::optional<size_t>>> step_idx = std::nullopt);

/** @brief Computes Precision and Recall statistics for the given solution
 * Precision and recall are computed on measurement classifications (inlier vs. outlier) using the set of potential
 * outliers / true outliers marked in the dataset, along with the set of measurements rejected as outliers in the
 * solution.
 * NOTE: Inlier is considered the POSITIVE class for metrics despite results marking outliers
 * NOTE: If precision or recall cannot be computed this function reports -1
 * @param dataset: The dataset containing inlier factor information
 * @param results: The estimation results
 * @param step_idx: The number of entries from which we will compute the residual (used if we only have partial results)
 * if nullopt we use all entries
 * @returns The overall precision and recall, and the per-robot precision and recall
 */
std::pair<PrecisionRecall, std::map<char, PrecisionRecall>> computePrecisionRecall(
    Dataset dataset, Results results, std::optional<std::map<char, std::optional<size_t>>> step_idx = std::nullopt);

/** @brief Computes all metrics possible for the given datasets.
 *  Conditions to compute different metrics
 *  - ATE: dataset must contain groundtruth
 *  - SVE: dataset must be multi-robot
 *  - Mean Residual: always computable
 * Note: If step_idx is provided it is provided to computeMeanResidual, and computeATE is set to allow partial
 * results
 */
template <class POSE_TYPE>
inline MetricSummary computeMetricSummary(Dataset dataset, Results results, bool ate_align = true,
                                          bool ate_align_with_scale = false, bool ate_include_shared_variables = true,
                                          std::optional<std::map<char, std::optional<size_t>>> step_idx = std::nullopt);

}  // namespace metrics

}  // namespace jrl

#include "jrl/Metrics-inl.h"