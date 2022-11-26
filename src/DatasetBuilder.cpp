#include "jrl/DatasetBuilder.h"

#include <boost/optional.hpp>

namespace jrl {

DatasetBuilder::DatasetBuilder(const std::string& name, std::vector<char>& robots) : name_(name), robots_(robots) {
  for (auto& r : robots_) {
    measurements_[r] = std::vector<Entry>();
    initial_estimates_[r] = std::make_pair(gtsam::Values(), ValueTypes());
    ground_truth_[r] = std::make_pair(gtsam::Values(), ValueTypes());
  }
}

void DatasetBuilder::addEntry(char& robot, uint64_t& stamp, gtsam::NonlinearFactorGraph& measurements,
                              std::vector<std::string>& measurement_types,
                              const boost::optional<std::pair<gtsam::Values, ValueTypes>>& initialization,
                              const boost::optional<std::pair<gtsam::Values, ValueTypes>>& groundtruth) {
  measurements_[robot].push_back(Entry(stamp, measurement_types, measurements));
  if (initialization) {
    initial_estimates_[robot].first.insert((*initialization).first);
    initial_estimates_[robot].second.insert((*initialization).second.begin(), (*initialization).second.end());
  }

  if (groundtruth) {
    ground_truth_[robot].first.insert((*groundtruth).first);
    ground_truth_[robot].second.insert((*groundtruth).second.begin(), (*groundtruth).second.end());
  }
}

Dataset DatasetBuilder::build() {
  boost::optional<std::map<char, std::pair<gtsam::Values, ValueTypes>>> gt = boost::none;
  boost::optional<std::map<char, std::pair<gtsam::Values, ValueTypes>>> init = boost::none;
  if (ground_truth_[robots_.front()].first.size() > 0) gt = ground_truth_;
  if (ground_truth_[robots_.front()].first.size() > 0) init = initial_estimates_;
  return Dataset(name_, robots_, measurements_, gt, init);
}

}  // namespace jrl