/// @brief This module implements a C++ class for representing JRL Dataset.
#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>

#include "jrl/Types.h"

namespace jrl {

/** @brief Representation of a JRL Dataset.
 * Contains robot measurements, groundtruth values, and optionally initialization.
 */
class Dataset {
  /* FIELDS */
 private:
  /// @brief The title of the dataset
  std::string name_;

  /// @brief The robots for which this dataset defines trajectories
  std::vector<char> robots_;

  /// @brief The measurements made by each robot. Ordered temporally.
  std::map<char, std::vector<Entry>> measurements_;

  /// @brief The set measurements made by each robot unordered in a factor graph
  std::map<char, gtsam::NonlinearFactorGraph> factor_graphs_;

  /// @brief The dataset ground-truth for each robot. Each robot's ground truth should contain all values that that
  /// robot observed. This means some values may appear multiple times if multiple robots observe them. Any values
  /// appearing multiple times MUST be the same.
  boost::optional<std::map<char, TypedValues>> ground_truth_;

  /// @brief The dataset Initialization for each robot. Each robot's initialization should contain all values that that
  /// robot observed. This means some values may appear multiple times if multiple robots observe them. Any values
  /// appearing multiple times MAY be different.
  boost::optional<std::map<char, TypedValues>> initial_estimates_;

  /// @brief The set of measurements that may be outliers for each robot.
  /// All measurements appearing in this set should be considered as possible outliers (i.e. loop closures).
  /// Measurements are identified by their FactorId which is their entry index and measurement index in that entry
  boost::optional<std::map<char, std::set<FactorId>>> potential_outlier_factors_;

  /// @brief The set of true measurements that are actually outliers (if known) for each robot
  /// Measurements are identified by their FactorId which is their entry index and measurement index in that entry
  boost::optional<std::map<char, std::set<FactorId>>> outlier_factors_;

  /// @brief The type of measurements made by each robot in it's graph
  std::map<char, std::set<std::string>> measurement_types_;

  /* INTERFACE */
 public:
  /** @brief Constructs a raw dataset
   */
  Dataset(const std::string name, std::vector<char> robots, std::map<char, std::vector<Entry>> measurements,
          boost::optional<std::map<char, TypedValues>> ground_truth = boost::none,
          boost::optional<std::map<char, TypedValues>> initial_estimates = boost::none,
          boost::optional<std::map<char, std::set<FactorId>>> potential_outlier_factors = boost::none,
          boost::optional<std::map<char, std::set<FactorId>>> outlier_factors = boost::none);

  /// @brief returns the name of the dataset
  std::string name() const;

  /// @brief Returns a list of the robots active in this dataset
  std::vector<char> robots() const;

  /** @brief Returns the measurements for a specific robot
   * @param robot_id: The robot identifier for the measurements to return. Not required for single robot dataset.
   * @returns The specified robot's measurements
   */
  std::vector<Entry> measurements(const boost::optional<char>& robot_id = boost::none) const;

  /** @brief Returns the factor graph for a specific robot containing all measurements
   * @param robot_id: The robot identifier for the measurements to return. Not required for single robot dataset.
   * @returns The specified robot's factor graph
   */
  gtsam::NonlinearFactorGraph factorGraph(const boost::optional<char>& robot_id = boost::none) const;

  /** @brief Returns the ground truth values for a specific robot.
   * @param robot_id: The robot identifier for the ground truth to return. Not required for single robot dataset.
   * @returns The specified robot's ground truth values
   */
  gtsam::Values groundTruth(const boost::optional<char>& robot_id = boost::none) const;
  TypedValues groundTruthWithTypes(const boost::optional<char>& robot_id = boost::none) const;
  bool containsGroundTruth() const { return ground_truth_.is_initialized(); }

  /** @brief Returns the initialization values for a specific robot.
   * @param robot_id: The robot identifier for the initialization to return. Not required for single robot dataset.
   * @returns The specified robot's initial values
   */
  gtsam::Values initialization(const boost::optional<char>& robot_id = boost::none) const;
  TypedValues initializationWithTypes(const boost::optional<char>& robot_id = boost::none) const;
  bool containsInitialization() const { return initial_estimates_.is_initialized(); }

  /** @brief Returns the set of measurements that may be outliers (i.e. loop-closures)
   * @param robot_id: The robot identifier for the measurements to return. Not required for single robot dataset.
   * @returns The specified robot's potential outlier measurements
   */
  std::set<FactorId> potentialOutlierFactors(const boost::optional<char>& robot_id = boost::none) const;
  bool containsPotentialOutlierFactors() const { return potential_outlier_factors_.is_initialized(); }

  /** @brief Returns the ground truth outlier measurements for a specific robot.
   * @param robot_id: The robot identifier for the ground truth to return. Not required for single robot dataset.
   * @returns The specified robot's ground outlier measurements
   */
  std::set<FactorId> outlierFactors(const boost::optional<char>& robot_id = boost::none) const;
  bool containsOutlierFactors() const { return outlier_factors_.is_initialized(); }


/** @brief Returns the measurement types for a specific robot
 * @param robot_id: The robot identifier for the measurement types to return. Not required for single robot dataset.
 * @returns The specified robot's measurement types
 */
  std::set<std::string> measurementTypes(const boost::optional<char>& robot_id = boost::none) const;

  /* HELPERS */
 private:
  /** @brief Generic Accessor as all the above have identical structure
   * @tparam RETURN_TYPE: Type that is being returned from this accessor
   * @param func_name: The name of the wrapper function for error messages
   * @param robot_mapping: The mapping we are accessing
   * @param robot_id: The identifier of the robot
   * @return RETURN_TYPE value for robot robot_id
   * */
  template <typename RETURN_TYPE>
  RETURN_TYPE accessor(const std::string& func_name, boost::optional<std::map<char, RETURN_TYPE>> robot_mapping,
                       const boost::optional<char>& robot_id) const;
};
}  // namespace jrl