/// @brief This module contains the definition for some common types used throughout the JRL library.
#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
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

  /// @brief Function that takes in a type and a factor and returns true if the factor should be kept
  typedef std::function<bool(const std::string&, const gtsam::NonlinearFactor::shared_ptr&)> FilterPredicate;

  /// @brief Explicit constructor for an Entry
  Entry(uint64_t stamp, std::vector<std::string> measurement_types, gtsam::NonlinearFactorGraph measurements)
      : stamp(stamp), measurement_types(measurement_types), measurements(measurements) {}

  /// @brief Helper to make predicate for filtering based on types
  /// @param types List of jrl tags to keep
  /// @return Predicate that returns true if the factor type is in the list of types
  static FilterPredicate KeepTypes(std::vector<std::string> types);

  /// @brief Helper to make predicate for removing based on types
  /// @param types List of jrl tags to remove
  /// @return Predicate that returns true if the factor type is not the list of types
  static FilterPredicate RemoveTypes(std::vector<std::string> types);

  /// @brief Returns a new Entry with only the measurements that satisfy the predicate
  /// @param predicate Function that returns true if the measurement should be included
  /// @return New Entry with only the measurements that satisfy the predicate
  Entry filtered(FilterPredicate predicate) const;
};

/** @brief A unique identifier for a factor in a dataset. [Entry Idx, Measurement Idx]
 * Each factor can be uniquely identified by its entry index, and the index that the
 * factor appears in that entry which we refer to as its Measurement Index.
 */
typedef std::pair<size_t, gtsam::FactorIndex> FactorId;

}  // namespace jrl
