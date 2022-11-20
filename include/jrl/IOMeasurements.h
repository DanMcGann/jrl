#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;
namespace jrl {

// Define statically the tags for measurements
static const std::string BetweenFactorPose2Tag = "BetweenFactorPose2";
static const std::string BetweenFactorPose3Tag = "BetweenFactorPose3";
static const std::string PriorFactorPose2Tag = "PriorFactorPose2";
static const std::string PriorFactorPose3Tag = "PriorFactorPose3";
static const std::string RangeFactorPose2Tag = "RangeFactorPose2";
static const std::string RangeFactorPose3Tag = "RangeFactorPose3";

namespace io_measurements {

/**********************************************************************************************************************/
gtsam::Matrix parseCovariance(json input_json, int d);
json serializeCovariance(gtsam::Matrix covariance);

/**********************************************************************************************************************/
template <typename T>
gtsam::NonlinearFactor::shared_ptr parsePrior(std::function<T(json)> val_parser_fn, json input_json) {
  // Get all required fields
  json key_json = input_json["key"];
  json measurement_json = input_json["prior"];
  json covariance_json = input_json["covariance"];

  // Construct the factor
  T measured = val_parser_fn(measurement_json);
  int d = gtsam::traits<T>::GetDimension(measured);
  typename gtsam::Matrix covariance = parseCovariance(input_json["covariance"], d);
  typename gtsam::PriorFactor<T>::shared_ptr factor = boost::make_shared<gtsam::PriorFactor<T>>(
      key_json.get<uint64_t>(), measured, gtsam::noiseModel::Gaussian::Covariance(parseCovariance(covariance_json, d)));
  return factor;
}

template <typename T>
json serializePrior(std::function<json(T)> val_serializer_fn, std::string type_tag,
                    gtsam::NonlinearFactor::shared_ptr& factor) {
  json output;
  typename gtsam::PriorFactor<T>::shared_ptr prior =
      boost::dynamic_pointer_cast<typename gtsam::PriorFactor<T>>(factor);
  gtsam::noiseModel::Gaussian::shared_ptr noise_model =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(prior->noiseModel());

  output["type"] = type_tag;
  output["key"] = prior->key();
  output["prior"] = val_serializer_fn(prior->prior());

  output["covariance"] = serializeCovariance(noise_model->covariance());
  return output;
}

/**********************************************************************************************************************/
template <typename MEASURE, typename FACTOR>
gtsam::NonlinearFactor::shared_ptr parseNoiseModel2(std::function<MEASURE(json)> val_parser_fn, json input_json) {
  // Get all required fields
  json key1_json = input_json["key1"];
  json key2_json = input_json["key2"];
  json measurement_json = input_json["measurement"];
  json covariance_json = input_json["covariance"];

  // Construct the factor
  MEASURE measured = val_parser_fn(measurement_json);
  int d = gtsam::traits<MEASURE>::GetDimension(measured);
  typename FACTOR::shared_ptr factor =
      boost::make_shared<FACTOR>(key1_json.get<uint64_t>(), key2_json.get<uint64_t>(), measured,
                                 gtsam::noiseModel::Gaussian::Covariance(parseCovariance(covariance_json, d)));
  return factor;
}

template <typename MEASURE, typename FACTOR>
json serializeNoiseModel2(std::function<json(MEASURE)> val_serializer_fn, std::string type_tag,
                      gtsam::NonlinearFactor::shared_ptr& factor) {
  json output;
  typename FACTOR::shared_ptr between = boost::dynamic_pointer_cast<FACTOR>(factor);
  gtsam::noiseModel::Gaussian::shared_ptr noise_model =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(between->noiseModel());
  output["type"] = type_tag;
  output["key1"] = between->keys().front();
  output["key2"] = between->keys().back();
  output["measurement"] = val_serializer_fn(between->measured());
  output["covariance"] = serializeCovariance(noise_model->covariance());
  return output;
}

}  // namespace io_measurements

}  // namespace jrl