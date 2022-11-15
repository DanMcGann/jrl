#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

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

/**********************************************************************************************************************/
template <typename T>
gtsam::NonlinearFactor::shared_ptr parseBetween(std::function<T(json)> val_parser_fn, json input_json) {
  // Get all required fields
  json key1_json = input_json["key1"];
  json key2_json = input_json["key2"];
  json measurement_json = input_json["measurement"];
  json covariance_json = input_json["covariance"];

  // Construct the factor
  T measured = val_parser_fn(measurement_json);
  int d = gtsam::traits<T>::GetDimension(measured);
  typename gtsam::BetweenFactor<T>::shared_ptr factor = boost::make_shared<gtsam::BetweenFactor<T>>(
      key1_json.get<uint64_t>(), key2_json.get<uint64_t>(), measured,
      gtsam::noiseModel::Gaussian::Covariance(parseCovariance(covariance_json, d)));
  return factor;
}

template <typename T>
gtsam::NonlinearFactor::shared_ptr serializeBetween(std::function<json(T)> val_serializer_fn,
                                                    gtsam::NonlinearFactor::shared_ptr& factor) {
  json output;
  typename gtsam::BetweenFactor<T>::shared_ptr between = std::dynamic_pointer_cast<gtsam::BetweenFactor<T>::shared_ptr>(factor);
  output["key1"] = between->key1();
  output["key2"] = between->key2();
  output["measurement"] = val_serializer_fn(between->measured());
  output["covariance"] = serializeCovariance(between->noiseModel()->covariance());
  return factor;
}

/**********************************************************************************************************************/
template <typename T>
gtsam::NonlinearFactor::shared_ptr parsePrior(std::function<T(json)> val_parser_fn, json input_json) {
  // Get all required fields
  json key1_json = input_json["key1"];
  json measurement_json = input_json["measurement"];
  json covariance_json = input_json["covariance"];

  // Construct the factor
  T measured = val_parser_fn(measurement_json);
  int d = gtsam::traits<T>::GetDimension(measured);
  typename gtsam::Matrix covariance = parseCovariance(input_json["covariance"], d);
  typename gtsam::PriorFactor<T>::shared_ptr factor = boost::make_shared<gtsam::PriorFactor<T>>(
      key1_json.get<uint64_t>(), measured,
      gtsam::noiseModel::Gaussian::Covariance(parseCovariance(covariance_json, d)));
  return factor;
}

template <typename T>
gtsam::NonlinearFactor::shared_ptr serializePrior(std::function<json(T)> val_serializer_fn,
                                                  gtsam::NonlinearFactor::shared_ptr& factor) {
  json output;
  typename gtsam::PriorFactor<T>::shared_ptr prior = std::dynamic_pointer_cast<gtsam::PriorFactor<T>::shared_ptr>(factor);
  output["key1"] = prior->key1();
  output["measurement"] = val_serializer_fn(prior->measured());
  output["covariance"] = serializeCovariance(prior->noiseModel()->covariance());
  return factor;
}

}  // namespace io_measurements

}  // namespace jrl