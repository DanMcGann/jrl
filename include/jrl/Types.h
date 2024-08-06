#pragma once
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

  Entry(uint64_t stamp, std::vector<std::string> measurement_types, gtsam::NonlinearFactorGraph measurements)
      : stamp(stamp), measurement_types(measurement_types), measurements(measurements) {}
};

/** @brief A unique identifier for a factor in a dataset. [Entry Idx, Measurement Idx]
 * Each factor can be uniquely identified by its entry index, and the index that the
 * factor appears in that entry which we refer to as its Measurement Index.
 */
typedef std::pair<size_t, gtsam::FactorIndex> FactorId;

}  // namespace jrl
