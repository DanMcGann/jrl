#include "jrl/IOMeasurements.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace jrl {
namespace io_measurements {

/**********************************************************************************************************************/
// Covariance
gtsam::Matrix parseCovariance(json input_json, int d) {
  auto v = input_json.get<std::vector<double>>();
  gtsam::Matrix m = Eigen::Map<gtsam::Matrix>(v.data(), d, d);
  return m;
}

json serializeCovariance(gtsam::Matrix covariance) {
  std::vector<double> vec(covariance.data(), covariance.data() + covariance.rows() * covariance.cols());
  return json(vec);
}

/**********************************************************************************************************************/
// Cal3_S2Stereo
gtsam::Cal3_S2Stereo::shared_ptr parseCal3_S2Stereo(json input_json) {
  double fx = input_json["fx"].get<double>();
  double fy = input_json["fy"].get<double>();
  double skew = input_json["skew"].get<double>();
  double px = input_json["px"].get<double>();
  double py = input_json["py"].get<double>();
  double baseline = input_json["baseline"].get<double>();
  return boost::make_shared<gtsam::Cal3_S2Stereo>(fx, fy, skew, px, py, baseline);
}

json serializeCal3_S2Stereo(gtsam::Cal3_S2Stereo::shared_ptr calibration) {
  json output;
  output["fx"] = calibration->fx();
  output["fy"] = calibration->fy();
  output["skew"] = calibration->skew();
  output["px"] = calibration->px();
  output["py"] = calibration->py();
  output["baseline"] = calibration->baseline();
  return output;
}

/**********************************************************************************************************************/
// StereoPoint2
gtsam::StereoPoint2 parseStereoPoint2(json input_json) {
  double uL = input_json["uL"].get<double>();
  double uR = input_json["uR"].get<double>();
  double v = input_json["v"].get<double>();
  return gtsam::StereoPoint2(uL, uR, v);
}

json serializeStereoPoint2(gtsam::StereoPoint2 point) {
  json output;
  output["uL"] = point.uL();
  output["uR"] = point.uR();
  output["v"] = point.v();
  return output;
}

}  // namespace io_measurements
}  // namespace jrl