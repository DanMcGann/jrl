#include "jrl/Types.h"

namespace jrl {

/**********************************************************************************************************************/
Entry::FilterPredicate Entry::KeepTypes(std::vector<std::string> types) {
  return [types](const std::string& type, const gtsam::NonlinearFactor::shared_ptr&) {
    return std::find(types.begin(), types.end(), type) != types.end();
  };
}

/**********************************************************************************************************************/
Entry::FilterPredicate Entry::RemoveTypes(std::vector<std::string> types) {
  return [types](const std::string& type, const gtsam::NonlinearFactor::shared_ptr&) {
    return std::find(types.begin(), types.end(), type) == types.end();
  };
}

/**********************************************************************************************************************/
Entry Entry::filtered(Entry::FilterPredicate predicate) const {
  std::vector<std::string> out_measurement_types;
  gtsam::NonlinearFactorGraph out_measurements;
  gtsam::FactorIndex curr = 0;

  for (uint64_t i = 0; i < measurement_types.size(); ++i) {
    if (predicate(measurement_types[i], measurements[i])) {
      out_measurement_types.push_back(measurement_types[i]);
      out_measurements.push_back(measurements[i]);
      curr += 1;
    }
  }

  return Entry(stamp, out_measurement_types, out_measurements);
}

}  // namespace jrl