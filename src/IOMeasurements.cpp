#include "jrl/IOMeasurements.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace jrl {
namespace io_measurements {

/**********************************************************************************************************************/
// Covariance
gtsam::Matrix parseMatrix(json input_json, int row, int col) {
  auto v = input_json.get<std::vector<double>>();
  gtsam::Matrix m = Eigen::Map<gtsam::Matrix>(v.data(), row, col);
  return m;
}

json serializeMatrix(gtsam::Matrix mat) {
  std::vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
  return json(vec);
}

gtsam::Matrix parseCovariance(json input_json, int d) { return parseMatrix(input_json, d, d); }

json serializeCovariance(gtsam::Matrix covariance) { return serializeMatrix(covariance); }

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

/**********************************************************************************************************************/
// IMUFactor
gtsam::NonlinearFactor::shared_ptr parseCombinedIMUFactor(json input_json) {
  // First construct Params
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> params = gtsam::PreintegrationCombinedParams::MakeSharedU();
  params->accelerometerCovariance = parseCovariance(input_json["accCov"], 3);
  params->gyroscopeCovariance = parseCovariance(input_json["gyroCov"], 3);
  params->biasAccCovariance = parseCovariance(input_json["biasAccCov"], 3);
  params->biasOmegaCovariance = parseCovariance(input_json["biasGyroCov"], 3);
  params->biasAccOmegaInt = parseCovariance(input_json["biasAccOmegaInt"], 6);
  params->integrationCovariance = parseCovariance(input_json["intCov"], 3);
  params->n_gravity = io_values::parse<gtsam::Vector>(input_json["g"]);

  // Then construct TangentPreintegration
  gtsam::Vector deltaXij = io_values::parse<gtsam::Vector>(input_json["deltaXij"]);
  gtsam::Matrix H_biassAcc = parseMatrix(input_json["H_biasAcc"], 9, 3);
  gtsam::Matrix H_biassOmega = parseMatrix(input_json["H_biasOmega"], 9, 3);
  double deltaTij = io_values::parse<double>(input_json["deltaTij"]);
  gtsam::Vector biasHat_vec = io_values::parse<gtsam::Vector>(input_json["biasHat"]);
  gtsam::imuBias::ConstantBias biasHat(biasHat_vec);
  gtsam::TangentPreintegration tang_pim(params, deltaXij, H_biassAcc, H_biassOmega, biasHat, deltaTij);

  // Now turn it into CombinedPreintegration
  gtsam::Matrix cov = parseCovariance(input_json["covariance"], 15);
  gtsam::PreintegratedCombinedMeasurements pim(tang_pim, cov);

  // And finally into a factor
  uint64_t xi = input_json["key0"].get<uint64_t>();
  uint64_t vi = input_json["key1"].get<uint64_t>();
  uint64_t xj = input_json["key2"].get<uint64_t>();
  uint64_t vj = input_json["key3"].get<uint64_t>();
  uint64_t bi = input_json["key4"].get<uint64_t>();
  uint64_t bj = input_json["key5"].get<uint64_t>();
  gtsam::CombinedImuFactor::shared_ptr factor =
      boost::make_shared<gtsam::CombinedImuFactor>(xi, vi, xj, vj, bi, bj, pim);
  return factor;
}

json serializeCombinedIMUFactor(std::string type_tag, gtsam::NonlinearFactor::shared_ptr& factor) {
  json output;
  typename gtsam::CombinedImuFactor::shared_ptr imu_factor =
      boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor);
  gtsam::noiseModel::Gaussian::shared_ptr noise_model =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(imu_factor->noiseModel());
  gtsam::PreintegratedCombinedMeasurements pim = imu_factor->preintegratedMeasurements();

  output["type"] = type_tag;
  for (int i = 0; i < 6; i++) {
    output["key" + std::to_string(i)] = imu_factor->keys()[i];
  }
  output["covariance"] = serializeCovariance(noise_model->covariance());
  // output["deltaXij"] = io_values::serialize(pim.preintegrated());
  output["H_biasAcc"] = serializeMatrix(pim.preintegrated_H_biasAcc());
  output["H_biasOmega"] = serializeMatrix(pim.preintegrated_H_biasOmega());
  // output["deltaTij"] = io_values::serialize(pim.deltaTij());
  // output["biasHat"] = io_values::serialize(pim.biasHatVector());

  // PreintegrationParams
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> params =
      boost::dynamic_pointer_cast<gtsam::PreintegrationCombinedParams>(pim.params());
  output["accCov"] = serializeCovariance(params->accelerometerCovariance);
  output["gyroCov"] = serializeCovariance(params->gyroscopeCovariance);
  output["biasAccCov"] = serializeCovariance(params->biasAccCovariance);
  output["biasGyroCov"] = serializeCovariance(params->biasOmegaCovariance);
  output["biasAccOmegaInt"] = serializeCovariance(params->biasAccOmegaInt);
  output["intCov"] = serializeCovariance(params->integrationCovariance);
  // output["g"] = io_values::serialize(params->n_gravity);

  // omegaCoriolis
  // body_P_sensor

  return output;
}

}  // namespace io_measurements
}  // namespace jrl