#include "jrl/Initialization.h"

#include "jrl/IOMeasurements.h"
#include "jrl/Utilities.h"

namespace jrl {

Initializer::Initializer() {
  measurement_forward_models_ = loadDefaultForwardModels();
  measurement_forward_model_priorities_ = loadDefaultForwardModelPriorities();
}

/**********************************************************************************************************************/
std::map<std::string, ForwardMeasurementModel::shared_ptr> Initializer::loadDefaultForwardModels() {
  // clang-format off
  std::map<std::string, ForwardMeasurementModel::shared_ptr>  forward_models = {
      {PriorFactorPose2Tag,    std::make_shared<PriorForwardModel<gtsam::Pose2>>()},
      {PriorFactorPose3Tag,    std::make_shared<PriorForwardModel<gtsam::Pose3>>()},
      {PriorFactorPoint2Tag,   std::make_shared<PriorForwardModel<gtsam::Point2>>()},
      {PriorFactorPoint3Tag,   std::make_shared<PriorForwardModel<gtsam::Point3>>()},
      {BetweenFactorPose2Tag,  std::make_shared<BetweenForwardModel<gtsam::Pose2>>()},
      {BetweenFactorPose3Tag,  std::make_shared<BetweenForwardModel<gtsam::Pose3>>()},
      {BetweenFactorPoint2Tag, std::make_shared<BetweenForwardModel<gtsam::Point2>>()},
      {BetweenFactorPoint3Tag, std::make_shared<BetweenForwardModel<gtsam::Point3>>()},
      {BearingRangeFactorPose2Tag, std::make_shared<BearingRangeForwardModel<gtsam::Pose2, gtsam::Pose2>>()},
      {BearingRangeFactorPose3Tag, std::make_shared<BearingRangeForwardModel<gtsam::Pose3, gtsam::Pose3>>()},
      {BearingRangeFactor2DTag, std::make_shared<BearingRangeForwardModel<gtsam::Pose2, gtsam::Point2>>()},
      {BearingRangeFactor3DTag, std::make_shared<BearingRangeForwardModel<gtsam::Pose3, gtsam::Point3>>()}
  };
  // clang-format on
  return forward_models;
}

/**********************************************************************************************************************/
std::map<std::string, size_t> Initializer::loadDefaultForwardModelPriorities() {
  // clang-format off
  std::map<std::string, size_t>  model_priorities = {
      {PriorFactorPose2Tag,         0}, // Priors get first priority
      {PriorFactorPose3Tag,         0},
      {PriorFactorPoint2Tag,        0},
      {PriorFactorPoint3Tag,        0},
      {BetweenFactorPose2Tag,       1}, // Between factors get second priority
      {BetweenFactorPose3Tag,       1},
      {BetweenFactorPoint2Tag,      1},
      {BetweenFactorPoint3Tag,      1},
      {BearingRangeFactorPose2Tag,  2}, // Bearing+Range gets third since it cant initialize orientation
      {BearingRangeFactorPose3Tag,  2},
      {BearingRangeFactor2DTag,     2},
      {BearingRangeFactor3DTag,     2}
  };
  // clang-format on
  return model_priorities;
}

/**********************************************************************************************************************/
gtsam::Values Initializer::initialization(const Entry& entry, const gtsam::Values& current_solution) const {
  // Get all the new variables induced by the entry measurements
  gtsam::KeySet new_variables = computeNewVariables(entry, current_solution);

  // Get the signatures for all entry measurements that have forward models
  std::map<size_t, std::pair<size_t, ForwardMeasurementModel::Signature>> forward_factor_signatures =
      computeSignaturesAndPriorities(entry);

  // For each new variable rank the factors that could be used to initialize it
  std::vector<std::vector<size_t>> variable_initialization_factors_ranked =
      rankVariableInitializationFactors(new_variables, forward_factor_signatures);

  // Compute all possible factor combinations that can be used to initialize this entry
  std::vector<std::vector<size_t>> factor_combinations =
      utils::cartesianProduct(variable_initialization_factors_ranked);

  // Until we find a solution or
  for (auto& factor_combination : factor_combinations) {
    // Compute the dependency graph for the given factor combination
    std::map<gtsam::Key, gtsam::KeySet> new_variable_dependency_graph =
        computeDependencyGraph(new_variables, forward_factor_signatures, factor_combination);

    // Compute the topological ordering for the dependency graph
    std::optional<std::vector<gtsam::Key>> ordering = computeTopologicalOrdering(new_variable_dependency_graph);

    // If there exists an ordering use it to compute the new variables
    if (ordering) {
      return computeInitialization(new_variables, factor_combination, *ordering, entry, current_solution);
    }
  }
  throw std::runtime_error(
      "Initializer::Initialization unable to find valid topological ordering to initialize new keys");
}

/**********************************************************************************************************************/
gtsam::KeySet Initializer::computeNewVariables(const Entry& entry, const gtsam::Values& current_solution) const {
  gtsam::KeySet entry_variables = entry.measurements.keys();
  gtsam::KeySet solution_keys = current_solution.keySet();
  gtsam::KeySet result;
  std::set_difference(entry_variables.cbegin(), entry_variables.cend(), solution_keys.cbegin(), solution_keys.cend(),
                      std::inserter(result, result.begin()));
  return result;
}

/**********************************************************************************************************************/
std::map<size_t, std::pair<size_t, ForwardMeasurementModel::Signature>> Initializer::computeSignaturesAndPriorities(
    const Entry& entry) const {
  std::map<size_t, std::pair<size_t, ForwardMeasurementModel::Signature>> signatures_and_priorities;
  for (size_t i = 0; i < entry.measurements.nrFactors(); i++) {
    if (measurement_forward_models_.count(entry.measurement_types[i]) != 0) {
      signatures_and_priorities[i] =
          std::make_pair(measurement_forward_model_priorities_.at(entry.measurement_types[i]),
                         measurement_forward_models_.at(entry.measurement_types[i])->signature(entry.measurements[i]));
    }
  }
  return signatures_and_priorities;
}

/**********************************************************************************************************************/
std::vector<std::vector<size_t>> Initializer::rankVariableInitializationFactors(
    const gtsam::KeySet& new_variables,
    const std::map<size_t, std::pair<size_t, ForwardMeasurementModel::Signature>>& signatures_and_priorities) const {
  std::map<gtsam::Key, std::vector<std::pair<size_t, size_t>>>
      variable_factors_and_priorities;  // (factor_idx, priority)
  std::vector<std::vector<size_t>> result;

  for (const gtsam::Key& key : new_variables) {
    variable_factors_and_priorities[key] = std::vector<std::pair<size_t, size_t>>();
    // Aggregate the factors that can output this variable
    for (const auto& kvp : signatures_and_priorities) {
      if (kvp.second.second.outputs.count(key) != 0) {
        variable_factors_and_priorities[key].push_back(std::make_pair(kvp.first, kvp.second.first));
      }
    }

    // Sort the vector by factor priority
    std::sort(variable_factors_and_priorities[key].begin(), variable_factors_and_priorities[key].end(),
              [](auto& left, auto& right) { return left.second < right.second; });

    // Aggregate only the factor indexes into the result
    result.push_back(std::vector<size_t>());
    for (const auto& kvp : variable_factors_and_priorities[key]) {
      result.back().push_back(kvp.first);
    }

    // Check that the variable has non-zero entries
    if (result.back().size() == 0) {
      std::stringstream stream;
      stream << "Initializer::Initialization user requested initialization for variable: ";
      stream << gtsam::DefaultKeyFormatter(key) << std::endl;
      stream << "However, entry does not contain any factors with forward models to initialize the variable.";
      throw std::runtime_error(stream.str());
    }
  }

  return result;
}

/**********************************************************************************************************************/
std::map<gtsam::Key, gtsam::KeySet> Initializer::computeDependencyGraph(
    const gtsam::KeySet& new_variables,
    const std::map<size_t, std::pair<size_t, ForwardMeasurementModel::Signature>>& signatures_and_priorities,
    const std::vector<size_t>& factors) const {
  std::map<gtsam::Key, gtsam::KeySet> dependency_graph;

  size_t key_idx = 0;
  for (const gtsam::Key& key : new_variables) {
    gtsam::KeySet key_factor_inputs = signatures_and_priorities.at(factors[key_idx]).second.inputs;
    std::set_intersection(key_factor_inputs.begin(), key_factor_inputs.end(), new_variables.begin(),
                          new_variables.end(), std::inserter(dependency_graph[key], dependency_graph[key].begin()));
    key_idx++;
  }
  return dependency_graph;
}

/**********************************************************************************************************************/
std::optional<std::vector<gtsam::Key>> Initializer::computeTopologicalOrdering(
    const std::map<gtsam::Key, gtsam::KeySet>& dependency_graph) const {
  std::map<gtsam::Key, gtsam::KeySet> dep_graph_acc = std::map<gtsam::Key, gtsam::KeySet>(dependency_graph);

  // Ordering will have same cardinality as the graph size
  std::vector<gtsam::Key> order;
  for (size_t i = 0; i < dependency_graph.size(); i++) {
    // Find the first variable for which there are no unsatisfied
    std::optional<gtsam::Key> next_var = std::nullopt;
    for (const auto& kvp : dep_graph_acc) {
      if (kvp.second.size() == 0) {
        next_var = kvp.first;
        break;
      }
    }

    // If there is a valid next variable add it to the order and remove it from the graph
    if (next_var) {
      order.push_back(*next_var);
      dep_graph_acc.erase(*next_var);
      for (auto& kvp : dep_graph_acc) {
        kvp.second.erase(*next_var);
      }
    }
    // If there is not a valid next variable this factor combination does not permit a solution
    else {
      return std::nullopt;
    }
  }
  return order;
}

/**********************************************************************************************************************/
gtsam::Values Initializer::computeInitialization(const gtsam::KeySet& new_variables,
                                                 const std::vector<size_t>& factor_combination,
                                                 const std::vector<gtsam::Key>& ordering, const Entry& entry,
                                                 const gtsam::Values& current_solution) const {
  // Setup accumulators
  gtsam::Values solutions(current_solution);
  gtsam::Values new_variable_initialization;

  // Construct a mapping from variables to factors
  std::map<gtsam::Key, size_t> variable_generators;
  size_t key_idx;
  for (auto& key : new_variables) {
    variable_generators[key] = factor_combination[key_idx];
    key_idx++;
  }

  // Go through the ordering and use the designated factor to generate the variable
  for (const gtsam::Key& key : ordering) {
    size_t gen_idx = variable_generators[key];
    gtsam::Values forward_model_output = measurement_forward_models_.at(entry.measurement_types[gen_idx])
                                             ->predict(entry.measurements.at(gen_idx), solutions);
    new_variable_initialization.insert(key, forward_model_output.at(key));
    solutions.insert(key, forward_model_output.at(key));
  }
  return new_variable_initialization;
}

/**********************************************************************************************************************/
template <>
gtsam::Pose2 BearingRangeForwardModel<gtsam::Pose2, gtsam::Pose2>::project(
    gtsam::Pose2 origin, gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2> br) {
  gtsam::Point2 position = origin.translation() + origin.rotation() * (br.range() * br.bearing().unit());
  return gtsam::Pose2(gtsam::Rot2(), position);
}

/**********************************************************************************************************************/
template <>
gtsam::Pose3 BearingRangeForwardModel<gtsam::Pose3, gtsam::Pose3>::project(
    gtsam::Pose3 origin, gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3> br) {
  gtsam::Point3 position = origin.translation() + origin.rotation() * (br.range() * br.bearing());
  return gtsam::Pose3(gtsam::Rot3(), position);
}

/**********************************************************************************************************************/
template <>
gtsam::Point2 BearingRangeForwardModel<gtsam::Pose2, gtsam::Point2>::project(
    gtsam::Pose2 origin, gtsam::BearingRange<gtsam::Pose2, gtsam::Point2> br) {
  return origin.translation() + origin.rotation() * (br.range() * br.bearing().unit());
}

/**********************************************************************************************************************/
template <>
gtsam::Point3 BearingRangeForwardModel<gtsam::Pose3, gtsam::Point3>::project(
    gtsam::Pose3 origin, gtsam::BearingRange<gtsam::Pose3, gtsam::Point3> br) {
  return origin.translation() + origin.rotation() * (br.range() * br.bearing());
}

}  // namespace jrl
