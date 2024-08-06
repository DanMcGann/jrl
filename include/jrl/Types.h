/// @brief This module contains the definition for some common types used throughout the JRL library.
#pragma once
#include <gtsam/nonlinear/Values.h>

namespace jrl {

/// @brief Mapping of a variable key to a tag representing its type
typedef std::map<gtsam::Key, std::string> ValueTypes;

/// @brief Structure for holding values along with their explicit types
struct TypedValues {
  /// @brief A set of Values
  gtsam::Values values;
  /// @brief The types for all values in values
  ValueTypes types;

  /// @brief Explicit constructor for Typed Values
  TypedValues(gtsam::Values values, ValueTypes types) : values(values), types(types) {}
  /// @brief Empty constructor for Typed Values
  TypedValues() {}
};

/// @brief An entry in a dataset. Represents all measurements taken by a robot at a given time (stamp)
struct Entry {
  /// @brief The time (or generalized index) at which the measurements were taken
  uint64_t stamp;
  /// @brief The type for each measurement in measurements (associated by order)
  std::vector<std::string> measurement_types;
  /// @brief The measurements taken by the robot at this time (stamp)
  gtsam::NonlinearFactorGraph measurements;
  /// @brief Indicates all measurements that may be outliers, and their groundtruth outlier status
  /// TODO (dan) The Ground truth (statuses) should really be in datasets as a separate optional components to support
  /// datasets where the true statuses is unknown
  std::map<gtsam::FactorIndex, bool> potential_outlier_statuses;

  /// @brief Explicit constructor for an Entry
  Entry(uint64_t stamp, std::vector<std::string> measurement_types, gtsam::NonlinearFactorGraph measurements,
        std::map<gtsam::FactorIndex, bool> potential_outlier_statuses = {})
      : stamp(stamp),
        measurement_types(measurement_types),
        measurements(measurements),
        potential_outlier_statuses(potential_outlier_statuses) {}
};

}  // namespace jrl
