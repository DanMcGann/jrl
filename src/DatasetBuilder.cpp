#include "jrl/DatasetBuilder.h"

#include <boost/optional.hpp>

namespace jrl {

DatasetBuilder::DatasetBuilder(const std::string name, std::vector<char> robots) : name_(name), robots_(robots) {
  for (auto& r : robots_) {
    measurements_[r] = std::vector<Entry>();
    initial_estimates_[r] = TypedValues();
    ground_truth_[r] = TypedValues();
  }
}

void DatasetBuilder::addEntry(char robot, uint64_t stamp, gtsam::NonlinearFactorGraph measurements,
                              std::vector<std::string> measurement_types,
                              std::map<gtsam::FactorIndex, bool> potential_outlier_statuses,
                              const boost::optional<TypedValues> initialization,
                              const boost::optional<TypedValues> groundtruth) {
  measurements_[robot].push_back(Entry(stamp, measurement_types, measurements, potential_outlier_statuses));
  if (initialization) {
    addInitialization(robot, *initialization);
  }

  if (groundtruth) {
    addGroundTruth(robot, *groundtruth);
  }
}

void DatasetBuilder::addGroundTruth(char robot, TypedValues groundtruth) {
  ground_truth_[robot].values.insert(groundtruth.values);
  ground_truth_[robot].types.insert(groundtruth.types.begin(), groundtruth.types.end());
}

void DatasetBuilder::addInitialization(char robot, TypedValues initialization) {
  initial_estimates_[robot].values.insert(initialization.values);
  initial_estimates_[robot].types.insert(initialization.types.begin(), initialization.types.end());
}

Dataset DatasetBuilder::build() {
  boost::optional<std::map<char, TypedValues>> gt = boost::none;
  boost::optional<std::map<char, TypedValues>> init = boost::none;
  if (ground_truth_[robots_.front()].values.size() > 0) gt = ground_truth_;
  if (initial_estimates_[robots_.front()].values.size() > 0) init = initial_estimates_;
  return Dataset(name_, robots_, measurements_, gt, init);
}

}  // namespace jrl