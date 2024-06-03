#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace jrl {

typedef std::map<gtsam::Key, std::string> ValueTypes;
typedef std::function<bool(std::string, gtsam::NonlinearFactor::shared_ptr&)> EntryPredicate;

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

  /// @brief Helper to make predicate for filtering based on types
  /// @param types List of jrl tags to keep
  /// @return Predicate that returns true if the factor type is in the list of types
  static EntryPredicate KeepTypes(std::vector<std::string> types);

  /// @brief Helper to make predicate for removing based on types
  /// @param types List of jrl tags to remove
  /// @return Predicate that returns true if the factor type is not the list of types
  static EntryPredicate RemoveTypes(std::vector<std::string> types);

  /// @brief Returns a new Entry with only the measurements that satisfy the predicate
  /// @param predicate Function that returns true if the measurement should be included
  /// @return New Entry with only the measurements that satisfy the predicate
  Entry filtered(EntryPredicate predicate);

};

}  // namespace jrl
