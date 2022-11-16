#include "jrl/IOMeasurements.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace jrl {
namespace io_measurements {

/**********************************************************************************************************************/
gtsam::Matrix parseCovariance(json input_json, int d) {
  auto v = input_json.get<std::vector<double>>();
  gtsam::Matrix m = Eigen::Map<gtsam::Matrix>(v.data(), d, d);
  return m;
}

json serializeCovariance(gtsam::Matrix covariance) {
  std::vector<double> vec(covariance.data(), covariance.data() + covariance.rows() * covariance.cols());
  return json(vec);
}

}  // namespace io_measurements
}  // namespace jrl