#pragma once
#include <boost/optional.hpp>
#include <nlohmann/json.hpp>

#include "jrl/Dataset.h"
using json = nlohmann::json;

namespace jrl {
/**
 *
 *
 *
 */
class DatasetBuilder {
  /* FIELDS */
 private:
  std::string name_;
  std::vector<char> robots_;
  size_t num_robots_;

  std::map<char, TypedValues> ground_truth_;
  std::map<char, TypedValues> initial_estimates_;
  std::map<char, std::vector<Entry>> measurements_;

  /* INTERFACE */
 public:
  DatasetBuilder(const std::string name, std::vector<char> robots);

  void addEntry(char robot, uint64_t stamp, gtsam::NonlinearFactorGraph measurements,
                std::vector<std::string> measurement_types,
                const boost::optional<TypedValues> initialization = boost::none,
                const boost::optional<TypedValues> groundtruth = boost::none);

  void addGroundTruth(char robot, TypedValues groundtruth);

  void addInitialization(char robot, TypedValues initialization);

  Dataset build();
};
}  // namespace jrl