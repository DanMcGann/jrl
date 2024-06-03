#include "jrl/Types.h"

namespace jrl {

/**********************************************************************************************************************/
EntryPredicate Entry::KeepTypes(std::vector<std::string> types) {
    return [types](const std::string& type, const gtsam::NonlinearFactor::shared_ptr&) {
      return std::find(types.begin(), types.end(), type) != types.end();
    };
  }


/**********************************************************************************************************************/
EntryPredicate Entry::RemoveTypes(std::vector<std::string> types) {
    return [types](const std::string& type, const gtsam::NonlinearFactor::shared_ptr&) {
      return std::find(types.begin(), types.end(), type) == types.end();
    };
  }

/**********************************************************************************************************************/
Entry Entry::filtered(EntryPredicate predicate) const {
  std::vector<std::string> out_measurement_types;
  gtsam::NonlinearFactorGraph out_measurements;
  std::map<gtsam::FactorIndex, bool> out_potential_outlier_statuses;
  gtsam::FactorIndex curr = 0;

  for (uint64_t i = 0; i < measurement_types.size(); ++i) {
    if (predicate(measurement_types[i], measurements[i])) {
      out_measurement_types.push_back(measurement_types[i]);
      out_measurements.push_back(measurements[i]);
      if (potential_outlier_statuses.find(i) != potential_outlier_statuses.end()) {
        out_potential_outlier_statuses[curr] = potential_outlier_statuses.at(i);
      }
      curr += 1;
    }
  }

  return Entry(stamp, out_measurement_types, out_measurements, out_potential_outlier_statuses);
}

}  // namespace jrl