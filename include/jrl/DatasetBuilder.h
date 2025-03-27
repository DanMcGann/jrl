/** @brief This module implements a convience class for incrementally constructing datasets.
 * The builder interface is useful for creating datasets in simulation.
 */
#pragma once
#include <boost/optional.hpp>
#include <nlohmann/json.hpp>

#include "jrl/Dataset.h"
using json = nlohmann::json;

namespace jrl {
/// @brief Helper class for incrementally constructing a dataset, entry by entry.
class DatasetBuilder {
  /* FIELDS */
 private:
  /// @brief The name of the dataset
  std::string name_;
  /// @brief The robots involved in the dataset
  std::vector<char> robots_;

  /// @brief The groundtruth values for each robot in the dataset
  std::map<char, TypedValues> ground_truth_;
  /// @brief The initial estimate values for each robot in the dataset
  std::map<char, TypedValues> initial_estimates_;
  /// @brief The measurements for each robot in the dataset
  std::map<char, std::vector<Entry>> measurements_;
  /// @brief The set of measurements that are potentially outliers
  std::map<char, std::set<FactorId>> potential_outlier_factors_;
  /// @brief The set of measurements that are known inliers
  std::map<char, std::set<FactorId>> outlier_factors_;

  /* INTERFACE */
 public:
  /// @brief Constructor for a dataset builder
  DatasetBuilder(const std::string name, std::vector<char> robots);

  /// @brief Adds information to the dataset incrementally for a single entry
  void addEntry(char robot, uint64_t stamp, gtsam::NonlinearFactorGraph measurements,
                std::vector<std::string> measurement_types,
                const std::optional<TypedValues> initialization = std::nullopt,
                const std::optional<TypedValues> groundtruth = std::nullopt);

  /// @brief Adds ground truth information for a single robot. Can be used incrementally or in bulk.
  void addGroundTruth(char robot, TypedValues groundtruth);

  /// @brief Adds initial estimate information for a single robot. Can be used incrementally or in bulk.
  void addInitialization(char robot, TypedValues initialization);

  /// @brief Add information for the potential outliers of the dataset
  /// WARN: Overwrites any currently stored potential outlier factors
  void setPotentialOutlierFactors(char robot, std::set<FactorId> potential_outlier_factors);

  /// @brief Add information on the true inliers of the dataset
  /// WARN: Overwrites any currently stored potential outlier factors
  void setOutlierFactors(char robot, std::set<FactorId> outlier_factors);

  /// @brief Compiles a dataset from all the added entries
  Dataset build();
};
}  // namespace jrl