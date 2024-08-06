/** @brief This class provides an interface to generate initialization for dataset entries
 * This can be used while building datasets, or while solving datasets incrementally.
 * Note: Because the initialization is written generically it will not be the fastest method.
 * Projects requiring fast runtime should use custom initialization techniques.
 **/
#pragma once
#include <gtsam/geometry/BearingRange.h>

#include "jrl/Dataset.h"
#include "jrl/Types.h"

namespace jrl {

/** @brief A forward measurement model describes how some measurements can be used to predict variable values
 * rather than the typical measurement model that predicts the measurement given the state.
 */
class ForwardMeasurementModel {
  /** Types **/
 public:
  /// @brief Shared pointer for a measurement model so we can have specialization
  typedef std::shared_ptr<ForwardMeasurementModel> shared_ptr;

  /// @brief The the signature of the forward
  struct Signature {
    /// @brief Keys that the forward model requires as inputs
    gtsam::KeySet inputs;
    /// @brief Keys that the forward model produces as outputs
    gtsam::KeySet outputs;
    /// @brief Constructor from the set of inputs and outputs
    Signature(const gtsam::KeySet& inputs, const gtsam::KeySet& outputs) : inputs(inputs), outputs(outputs) {}
    /// @brief Default constructor for an empty signature (no inputs no outputs)
    Signature() {}
  };

  /** Interface **/
 public:
  /// @brief Returns the signature of the forward model for computing dependencies
  virtual Signature signature(const gtsam::NonlinearFactor::shared_ptr& measurement) const = 0;

  /** @brief Computes the forward measurement model predicting the set of outputs from the set of inputs
   * @param measurement: The measurement for this forward measurement model
   * @param inputs: values containing the inputs to this measurement model
   * @return Estimates of the outputs from the measurement model
   */
  virtual gtsam::Values predict(const gtsam::NonlinearFactor::shared_ptr& measurement,
                                const gtsam::Values& inputs) const = 0;
};

/** @brief The initializer can be used to initialize new variables from dataset entries
 */
class Initializer {
  /** Members **/
 private:
  /// @brief Mapping from measurement tag to corresponding forward model function
  std::map<std::string, ForwardMeasurementModel::shared_ptr> measurement_forward_models_;
  /// @brief Implicitly defines default measurement forward models
  std::map<std::string, ForwardMeasurementModel::shared_ptr> loadDefaultForwardModels();

  /// @brief Mapping from measurement tag to corresponding forward model priority which is used if there are multiple
  /// ways to initialize a variable
  std::map<std::string, size_t> measurement_forward_model_priorities_;
  /// @brief Implicitly defines default measurement forward model priorities
  std::map<std::string, size_t> loadDefaultForwardModelPriorities();

  /** Interface **/
 public:
  /// @brief Constructors a writer object
  Initializer();

  /** @brief Computes the initialization for any new variables in the entry
   *  @param entry: New factors affecting new variables
   *  @param current_solution: A solution to the dataset for all previous entries
   *  @return initialization for all new variables computed using forward measurement models
   **/
  gtsam::Values initialization(const Entry& entry, const gtsam::Values& current_solution) const;

  /** Helpers **/
 private:
  /** @brief Computes the set of variables in the entry that are new variables to the system
   * @param entry: The entry containing new factors
   * @param current_solution: The solution to the system made up of all previous entries
   * @returns The set of variables affected by the entry that are not already in the system
   */
  gtsam::KeySet computeNewVariables(const Entry& entry, const gtsam::Values& current_solution) const;

  /** @brief Aggregates the signatures and priorities for each measurement that has a forward model
   * @param entry: The entry of new measurements
   * @returns A mapping from factor index to its signature for each measurement with a forward model
   */
  std::map<size_t, std::pair<size_t, ForwardMeasurementModel::Signature>> computeSignaturesAndPriorities(
      const Entry& entry) const;

  /** @brief For each new variable rank the factors that could be used to initialize it
   * @param new_variables: The new variables for which we need a factor rank
   * @param forward_factor_signatures: The signatures for all new factors with forward models
   * @returns An ordered ranking of factors to generate each variable. Order matches the iteration order of
   * new_variables.
   */
  std::vector<std::vector<size_t>> rankVariableInitializationFactors(
      const gtsam::KeySet& new_variables,
      const std::map<size_t, std::pair<size_t, ForwardMeasurementModel::Signature>>& signatures_and_priorities) const;

  /** @brief Computes the dependency graph for generating the new_variables
   * @param new_variables: The new variables for which we are computing an initialization
   * @param forward_factor_signatures: The signatures of all new factors with forward models
   * @param factors: The factor combination selected for exploration
   * @returns The dependency graph each variable maintains a set of INCOMING edges
   */
  std::map<gtsam::Key, gtsam::KeySet> computeDependencyGraph(
      const gtsam::KeySet& new_variables,
      const std::map<size_t, std::pair<size_t, ForwardMeasurementModel::Signature>>& signatures_and_priorities,
      const std::vector<size_t>& factors) const;

  /** @brief Computes a topological ordering for a given dependency graph
   * @param dependency_graph: the dependency graph for which to compute an ordering
   * @returns The ordering topological ordering in which to initialize variables or nullopt if no ordering is possible
   */
  std::optional<std::vector<gtsam::Key>> computeTopologicalOrdering(
      const std::map<gtsam::Key, gtsam::KeySet>& dependency_graph) const;

  /** @brief Using a variable ordering and a forward factor assignment for each compute the initialization
   * @param new_variables: The new variables we are initializing
   * @param factor_combination: The factor assigned to generate each new variable
   * @param ordering: The order in which to generate the new variables
   * @param entry: The entry containing the factors
   * @param current_solution: The solution to the system up to but not including the entry
   */
  gtsam::Values computeInitialization(const gtsam::KeySet& new_variables, const std::vector<size_t>& factor_combination,
                                      const std::vector<gtsam::Key>& ordering, const Entry& entry,
                                      const gtsam::Values& current_solution) const;
};

/**********************************************************************************************************************/

template <typename T>
class PriorForwardModel : public ForwardMeasurementModel {
 public:
  Signature signature(const gtsam::NonlinearFactor::shared_ptr& measurement) const;
  gtsam::Values predict(const gtsam::NonlinearFactor::shared_ptr& measurement, const gtsam::Values& inputs) const;
};

template <typename T>
class BetweenForwardModel : public ForwardMeasurementModel {
 public:
  Signature signature(const gtsam::NonlinearFactor::shared_ptr& measurement) const;
  gtsam::Values predict(const gtsam::NonlinearFactor::shared_ptr& measurement, const gtsam::Values& inputs) const;
};

template <typename A1, typename A2>
class BearingRangeForwardModel : public ForwardMeasurementModel {
 public:
  Signature signature(const gtsam::NonlinearFactor::shared_ptr& measurement) const;
  gtsam::Values predict(const gtsam::NonlinearFactor::shared_ptr& measurement, const gtsam::Values& inputs) const;
  static A2 project(A1 origin, gtsam::BearingRange<A1, A2> br);
};

}  // namespace jrl

// Implementation for template classes
#include "jrl/Initialization-inl.h"
