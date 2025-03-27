#include "jrl/Dataset.h"

#include <sstream>

namespace jrl {
/**********************************************************************************************************************/
Dataset::Dataset(const std::string name, std::vector<char> robots, std::map<char, std::vector<Entry>> measurements,
                 std::optional<std::map<char, TypedValues>> ground_truth,
                 std::optional<std::map<char, TypedValues>> initial_estimates,
                 std::optional<std::map<char, std::set<FactorId>>> potential_outlier_factors,
                 std::optional<std::map<char, std::set<FactorId>>> outlier_factors)
    : name_(name),
      robots_(robots),
      measurements_(measurements),
      ground_truth_(ground_truth),
      initial_estimates_(initial_estimates),
      potential_outlier_factors_(potential_outlier_factors),
      outlier_factors_(outlier_factors) {
  // Construct the factor graphs for each robot from the temporally ordered measurements
  for (const char& rid : robots_) {
    factor_graphs_[rid] = gtsam::NonlinearFactorGraph();
    measurement_types_[rid] = std::set<std::string>();
    for (const Entry& entry : measurements_[rid]) {
      factor_graphs_[rid].push_back(entry.measurements);
      measurement_types_[rid].insert(entry.measurement_types.begin(), entry.measurement_types.end());
    }
  }
}

/**********************************************************************************************************************/
std::string Dataset::name() const { return name_; }

/**********************************************************************************************************************/
std::vector<char> Dataset::robots() const { return robots_; }

/**********************************************************************************************************************/
std::vector<Entry> Dataset::measurements(const std::optional<char>& robot_id) const {
  return accessor<std::vector<Entry>>("measurements", measurements_, robot_id);
}

/**********************************************************************************************************************/
gtsam::NonlinearFactorGraph Dataset::factorGraph(const std::optional<char>& robot_id) const {
  return accessor<gtsam::NonlinearFactorGraph>("factorGraph", factor_graphs_, robot_id);
}

/**********************************************************************************************************************/
TypedValues Dataset::groundTruthWithTypes(const std::optional<char>& robot_id) const {
  return accessor<TypedValues>("groundTruth", ground_truth_, robot_id);
}

gtsam::Values Dataset::groundTruth(const std::optional<char>& robot_id) const {
  return groundTruthWithTypes(robot_id).values;
}

/**********************************************************************************************************************/
TypedValues Dataset::initializationWithTypes(const std::optional<char>& robot_id) const {
  return accessor<TypedValues>("initialization", initial_estimates_, robot_id);
}
gtsam::Values Dataset::initialization(const std::optional<char>& robot_id) const {
  return initializationWithTypes(robot_id).values;
}

/**********************************************************************************************************************/
std::set<FactorId> Dataset::potentialOutlierFactors(const std::optional<char>& robot_id) const {
  return accessor<std::set<FactorId>>("potentialOutlierFactors", potential_outlier_factors_, robot_id);
}

/**********************************************************************************************************************/
std::set<FactorId> Dataset::outlierFactors(const std::optional<char>& robot_id) const {
  return accessor<std::set<FactorId>>("outlierFactors", outlier_factors_, robot_id);
}
/**********************************************************************************************************************/
std::set<std::string> Dataset::measurementTypes(const std::optional<char>& robot_id) const {
  return accessor<std::set<std::string>>("measurementTypes", measurement_types_, robot_id);
}

/**********************************************************************************************************************/
template <typename RETURN_TYPE>
RETURN_TYPE Dataset::accessor(const std::string& func_name, std::optional<std::map<char, RETURN_TYPE>> robot_mapping,
                              const std::optional<char>& robot_id) const {
  if (robot_mapping == std::nullopt) {
    std::stringstream stream;
    stream << "Dataset:" << func_name << "requested but dataset does not contain " << func_name << "information";
    throw std::runtime_error(stream.str());
  } else if (robot_id == std::nullopt) {
    if (robots_.size() == 1) {
      return (*robot_mapping)[robots_[0]];
    } else {
      std::stringstream stream;
      stream << "Dataset:" << func_name;
      stream << " called without a robot_id, but Dataset is a multi-robot dataset.";
      stream << "Please request data for one of the following robots: ";
      for (auto& rid : robots_) {
        stream << rid << " ";
      }
      throw std::runtime_error(stream.str());
    }
  } else {
    try {
      return (*robot_mapping)[*robot_id];
    } catch (std::out_of_range& oor) {
      std::stringstream stream;
      stream << "Dataset:" << func_name;
      stream << " called with invalid robot_id (" << *robot_id << ").";
      stream << "This dataset contains data for the following robots: ";
      for (auto& rid : robots_) {
        stream << rid << " ";
      }
      throw std::out_of_range(stream.str());
    }
  }
}

}  // namespace jrl
