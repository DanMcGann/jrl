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

  /* INTERFACE */
 public:
  /// @brief Constructor for a dataset builder
  DatasetBuilder(const std::string name, std::vector<char> robots);

  /// @brief Adds information to the dataset incrementally for a single entry
  void addEntry(char robot, uint64_t stamp, gtsam::NonlinearFactorGraph measurements,
                std::vector<std::string> measurement_types,
                std::map<gtsam::FactorIndex, bool> potential_outlier_statuses = {},
                const boost::optional<TypedValues> initialization = boost::none,
                const boost::optional<TypedValues> groundtruth = boost::none);

  /// @brief 
  void addGroundTruth(char robot, TypedValues groundtruth);

  void addInitialization(char robot, TypedValues initialization);

  /// @brief Compiles a dataset from all the added entries
  Dataset build();
};
}  // namespace jrl