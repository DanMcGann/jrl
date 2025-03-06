#include "jrl/Metrics.h"

#include "jrl/Alignment.h"
#include "jrl/Utilities.h"

namespace jrl {

namespace metrics {

/**********************************************************************************************************************/
std::pair<PrecisionRecall, std::map<char, PrecisionRecall>> computePrecisionRecall(
    Dataset dataset, Results results, std::optional<std::map<char, std::optional<size_t>>> step_idxes) {
  // Setup overall counts Positive Class = Inliers
  double tp = 0;
  double fp = 0;
  double tn = 0;
  double fn = 0;

  // setup container for robot specific stats
  std::map<char, PrecisionRecall> robot_precision_recall;

  // Iterate over all the robots and compute precision recall + update total counts
  for (auto& rid : dataset.robots()) {
    // Setup robot counts
    double robot_tp = 0;
    double robot_fp = 0;
    double robot_tn = 0;
    double robot_fn = 0;

    // Get all of robot's entries
    auto entries = dataset.measurements(rid);
    auto potential_outliers = dataset.potentialOutlierFactors(rid);
    auto true_outliers = dataset.outlierFactors(rid);

    // Get the index of the last entry to use to compute the stats
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

    // For all relevant entries include in counts
    for (size_t entry_idx = 0; entry_idx < stop_idx; entry_idx++) {
      auto entry = entries[entry_idx];
      size_t num_entry_factors = entry.measurements.nrFactors();

      // For each measurement in this entry
      for (size_t measure_idx = 0; measure_idx < num_entry_factors; measure_idx++) {
        FactorId factor_id = std::make_pair(entry_idx, measure_idx);
        bool potential_outlier = potential_outliers.find(factor_id) != potential_outliers.end();
        // If the measurement is in the set of potential outliers include it in the computation
        if (potential_outlier) {
          bool is_outlier = true_outliers.find(factor_id) != true_outliers.end();
          bool pred_outlier = (*results.robot_outliers)[rid].find(factor_id) != (*results.robot_outliers)[rid].end();

          // Increment True/False Positive/Negative
          if (!is_outlier && !pred_outlier) {
            robot_tp++;
          } else if (!is_outlier && pred_outlier) {
            robot_fn++;
          } else if (is_outlier && !pred_outlier) {
            robot_fp++;
          } else if (is_outlier && pred_outlier) {
            robot_tn++;
          }
        }

        // Initialize Precision + Recall with invalid options for case of zero denominator
        double robot_precision = -1.0;
        double robot_recall = -1.0;

        // Compute the Precision and Recall for this robot
        if (robot_tp + robot_fp > 0.0) robot_precision = robot_tp / (robot_tp + robot_fp);
        if (robot_tp + robot_fn > 0.0) robot_recall = robot_tp / (robot_tp + robot_fn);
        robot_precision_recall[rid] = std::make_pair(robot_precision, robot_recall);
      }
    }

    // Update the global counts
    tp += robot_tp;
    fp += robot_fp;
    tn += robot_tn;
    fn += robot_fn;
  }

  // Initialize Precision + Recall with invalid options for case of zero denominator
  double precision = -1.0;
  double recall = -1.0;

  // Compute the Precision and Recall for for all robots
  if (tp + fp > 0.0) precision = tp / (tp + fp);
  if (tp + fn > 0.0) recall = tp / (tp + fn);
  auto precision_recall = std::make_pair(precision, recall);

  // Return all results
  return std::make_pair(precision_recall, robot_precision_recall);
}

}  // namespace metrics
}  // namespace jrl