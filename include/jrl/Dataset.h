#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace jrl {

typedef std::map<gtsam::Key, std::string> ValueTypes;

/**
 *
 *
 *
 */
struct Entry {
  uint64_t stamp;
  std::vector<std::string> measurement_types;
  gtsam::NonlinearFactorGraph measurements;
  Entry(uint64_t& stamp, std::vector<std::string>& measurement_types, gtsam::NonlinearFactorGraph& measurements)
      : stamp(stamp), measurement_types(measurement_types), measurements(measurements) {}
};

/**
 *
 *
 *
 */
class Dataset {
  /* FIELDS */
 private:
  /// @brief The title of the dataset
  std::string name_;

  /// @brief The robots for which this dataset defines trajectories
  std::vector<char> robots_;
  /// @brief The number of robots in this dataset
  size_t num_robots_;

  /// @brief The dataset ground-truth for each robot. Each robot's ground truth should contain all values that that
  /// robot observed. This means some values may appear multiple times if multiple robots observe them. Any values
  /// appearing multiple times MUST be the same.
  boost::optional<std::map<char, std::pair<gtsam::Values, ValueTypes>>> ground_truth_;

  /// @brief The dataset Initialization for each robot. Each robot's initialization should contain all values that that
  /// robot observed. This means some values may appear multiple times if multiple robots observe them. Any values
  /// appearing multiple times MAY be different.
  boost::optional<std::map<char, std::pair<gtsam::Values, ValueTypes>>> initial_estimates_;

  /// @brief The measurements made by each robot. Ordered temporally.
  std::map<char, std::vector<Entry>> measurements_;

  /* INTERFACE */
 public:
  /** @brief Constructs a raw dataset
   * @param dataset_json: The json object for the full dataset file.
   */
  Dataset(const std::string& name, std::vector<char>& robots, std::map<char, std::vector<Entry>> measurements,
          boost::optional<std::map<char, std::pair<gtsam::Values, ValueTypes>>>& ground_truth,
          boost::optional<std::map<char, std::pair<gtsam::Values, ValueTypes>>>& initial_estimates);

  /// @brief returns the name of the dataset
  std::string name();

  /// @brief Returns a list of the robots active in this dataset
  std::vector<char> robots();

  /** @brief Returns the ground truth values for a specific robot.
   * @param robot_id: The robot identifier for the ground truth to return. Not required for single robot dataset.
   * @returns The specified robot's ground truth values
   */
  gtsam::Values groundTruth(const boost::optional<char>& robot_id = boost::none);
  std::pair<gtsam::Values, ValueTypes> groundTruthWithTypes(const boost::optional<char>& robot_id = boost::none);
  bool containsGroundTruth() { return ground_truth_.is_initialized(); }

  /** @brief Returns the initialization values for a specific robot.
   * @param robot_id: The robot identifier for the initialization to return. Not required for single robot dataset.
   * @returns The specified robot's initial values
   */
  gtsam::Values initialization(const boost::optional<char>& robot_id = boost::none);
  std::pair<gtsam::Values, ValueTypes> initializationWithTypes(const boost::optional<char>& robot_id = boost::none);
  bool containsInitialization() { return initial_estimates_.is_initialized(); }

  /** @brief Returns the measurements for a specific robot
   * @param robot_id: The robot identifier for the measurements to return. Not required for single robot dataset.
   * @returns The specified robot's measurements
   */
  std::vector<Entry> measurements(const boost::optional<char>& robot_id = boost::none);

  /* HELPERS */
 private:
  /** @brief Generic Accessor as all the above have identical structure
   *  @tparam RETURN_TYPE: Type that is being returned from this accessor
   * @param func_name: The name of the wrapper function for error messages
   * @param robot_mapping: The mapping we are accessing
   * @param robot_id: The identifier of the robot
   * @return RETURN_TYPE value for robot robot_id
   * */
  template <typename RETURN_TYPE>
  RETURN_TYPE accessor(const std::string& func_name, boost::optional<std::map<char, RETURN_TYPE>> robot_mapping,
                       const boost::optional<char>& robot_id);
};
}  // namespace jrl