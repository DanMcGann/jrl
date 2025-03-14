/// @brief This module defines a container for storing the results produced by an algorithm running on a JRL dataset.
#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>

#include "jrl/Types.h"

namespace jrl {

struct Results {
  /// @brief The title of the dataset
  std::string dataset_name;

  /// @brief The name of the method used to generate these results
  std::string method_name;

  /// @brief The robots for which this dataset defines trajectories
  std::vector<char> robots;

  /// @brief Solution for each robot. Each robot's solution should contain all values that that
  /// robot observed. This means some values may appear multiple times if multiple robots observe them. Any values
  /// appearing multiple times MAY be different.
  std::map<char, TypedValues> robot_solutions;

  /// @brief The set of measurements rejected as outliers and not used to compute the solution.
  /// Factors are referred to by their index in their owner robot's factor graph.
  /// Note: Factors can only be rejected if they are declared potential outliers in the dataset.
  boost::optional<std::map<char, std::set<FactorId>>> robot_outliers;

  /** @brief Constructs a results with values
   * @param dataset_json: The json object for the full dataset file.
   */
  Results(std::string dataset_name, std::string method_name, std::vector<char> robots,
          std::map<char, TypedValues> solutions,
          boost::optional<std::map<char, std::set<FactorId>>> robot_outliers = boost::none);

  /** @brief Constructs an empty results
   * @param dataset_json: The json object for the full dataset file.
   */
  Results(std::string dataset_name, std::string method_name, std::vector<char> robots);
};
}  // namespace jrl