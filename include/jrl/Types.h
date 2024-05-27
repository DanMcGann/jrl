#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace jrl {

typedef std::map<gtsam::Key, std::string> ValueTypes;

struct TypedValues {
  gtsam::Values values;
  ValueTypes types;

  TypedValues(gtsam::Values values, ValueTypes types) : values(values), types(types) {}
  TypedValues() {}
};

struct Entry {
  uint64_t stamp;
  std::vector<std::string> measurement_types;
  gtsam::NonlinearFactorGraph measurements;
  std::map<gtsam::FactorIndex, bool> potential_outlier_statuses;

  Entry(uint64_t stamp, std::vector<std::string> measurement_types, gtsam::NonlinearFactorGraph measurements,
        std::map<gtsam::FactorIndex, bool> potential_outlier_statuses = {})
      : stamp(stamp),
        measurement_types(measurement_types),
        measurements(measurements),
        potential_outlier_statuses(potential_outlier_statuses) {}

  // Remove all factors of a certain type
  Entry remove(std::vector<std::string> remove_types);

  // Only include factors of a certain type
  Entry filter(std::vector<std::string> filter_types);
};

}  // namespace jrl
