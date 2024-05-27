#include "jrl/Types.h"

namespace jrl {

/**********************************************************************************************************************/
Entry Entry::remove(std::vector<std::string> remove_types) {
  std::vector<std::string> out_measurement_types;
  gtsam::NonlinearFactorGraph out_measurements;
  std::map<gtsam::FactorIndex, bool> out_potential_outlier_statuses;
  gtsam::FactorIndex curr = 0;

  for (uint64_t i = 0; i < measurement_types.size(); ++i) {
    // If the type isn't in remove_types, keep it
    if (std::find(remove_types.begin(), remove_types.end(), measurement_types[i]) == remove_types.end()) {
      out_measurement_types.push_back(measurement_types[i]);
      out_measurements.push_back(measurements[i]);
      if (potential_outlier_statuses.find(i) != potential_outlier_statuses.end()) {
        out_potential_outlier_statuses[curr] = potential_outlier_statuses[i];
      }
      curr += 1;
    }
  }

  return Entry(stamp, out_measurement_types, out_measurements, out_potential_outlier_statuses);
}

/**********************************************************************************************************************/
Entry Entry::filter(std::vector<std::string> filter_types) {
  std::vector<std::string> out_measurement_types;
  gtsam::NonlinearFactorGraph out_measurements;
  std::map<gtsam::FactorIndex, bool> out_potential_outlier_statuses;
  gtsam::FactorIndex curr = 0;

  for (uint64_t i = 0; i < measurement_types.size(); ++i) {
    // If the type is in filter_types, keep it
    if (std::find(filter_types.begin(), filter_types.end(), measurement_types[i]) != filter_types.end()) {
      out_measurement_types.push_back(measurement_types[i]);
      out_measurements.push_back(measurements[i]);
      if (potential_outlier_statuses.find(i) != potential_outlier_statuses.end()) {
        out_potential_outlier_statuses[curr] = potential_outlier_statuses[i];
      }
      curr += 1;
    }
  }

  return Entry(stamp, out_measurement_types, out_measurements, out_potential_outlier_statuses);
}

}  // namespace jrl