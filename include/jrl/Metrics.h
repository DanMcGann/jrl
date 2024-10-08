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
  boost::optional<std::map<char, std::pair<double, double>>> robot_ate{boost::none};
  /// @brief The total ate (sum of robot_ate's) with format: (ATE Translation, ATE Rotation)
  boost::optional<std::pair<double, double>> total_ate{boost::none};

  /// @brief The total SVE (sum of robot_sve's)
  boost::optional<std::pair<double, double>> sve{boost::none};

  /// @brief The Mean Residual
  boost::optional<double> mean_residual{boost::none};
};

namespace metrics {

namespace internal {
/** POSE ERRORS
 * A method for computing the error between two trajectory objects (POSES/POINTS/VECTORS)
 * returns translation error, rotation error
 * Note: For any linear object, the translation is itself
 */
template <class T>
inline std::pair<double, double> squaredPoseError(T est, T ref);
template <>
inline std::pair<double, double> squaredPoseError<gtsam::Pose3>(gtsam::Pose3 est, gtsam::Pose3 ref);
template <>
inline std::pair<double, double> squaredPoseError<gtsam::Pose2>(gtsam::Pose2 est, gtsam::Pose2 ref);
}  // namespace internal

/** @brief Computes the ATE for the specified robot.
 * This involves aligning the estiamted and reference trajectories with Umeyama alignment,
 * then comparing each pose to retrieve a translation and rotation error.
 * @param rid: The robot id of the robot for which to compute ATE
 * @param dataset: The dataset containing groundtruth
 * @param results: The estimation results to be compared with the groundtruth
 * @param align_with_scale: If true aligns scale while preforming Umeyama alignment
 * @param allow_partial_results: If true we compute ATE for only the poses in results
 * @returns Pair containing (ATE Translation, ATE Rotation) or boost::none if the dataset does not contain ground truth
 */
template <class POSE_TYPE>
inline boost::optional<std::pair<double, double>> computeATE(char rid, Dataset dataset, Results results,
                                                             bool align = true, bool align_with_scale = false,
                                                             bool allow_partial_results = false);

/** @brief Computes the SVE for the dataset
 * SVE is defined as the mean error between all combination of shared variable estimates
 * @param dataset: The dataset
 * @param results: The estimation results
 * @returns The SVE
 */
template <class POSE_TYPE>
inline std::pair<double, double> computeSVE(Results results);

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

/** @brief Computes all metrics possible for the given datasets.
 *  Conditions to compute different metrics
 *  - ATE: dataset must contain groundtruth
 *  - SVE: dataset must be multi-robot
 *  - Mean Residual: always computable
 * Note: If step_idx is provided it is provided to computeMeanResidual, and computeATE is set to allow partial results
 */
template <class POSE_TYPE>
inline MetricSummary computeMetricSummary(Dataset dataset, Results results, bool align = true,
                                          bool align_with_scale = false,
                                          std::optional<std::map<char, std::optional<size_t>>> step_idx = std::nullopt);

}  // namespace metrics

}  // namespace jrl

#include "jrl/Metrics-inl.h"