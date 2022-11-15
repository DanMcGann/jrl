#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;
namespace jrl {

namespace measurement_parser_functions {

gtsam::Matrix parseCovariance(json input_json, int d) {
  auto v = input_json["covariance"].get<std::vector<double>>();
  gtsam::Matrix m = Eigen::Map<gtsam::Matrix>(v.data(), d,d);
  return m;
}

/**********************************************************************************************************************/
template <typename T>
gtsam::NonlinearFactor::shared_ptr parseBetween(std::function<T(json)> val_parser_fn, json input_json) {
  T measured = val_parser_fn(input_json["measurement"]);
  int d = gtsam::traits<T>::GetDimension(measured);
  gtsam::Matrix covariance = parseCovariance(input_json, d);
  typename gtsam::BetweenFactor<T>::shared_ptr factor = boost::make_shared<gtsam::BetweenFactor<T>>(
      input_json["key1"].get<uint64_t>(), input_json["key2"].get<uint64_t>(), measured,
      gtsam::noiseModel::Gaussian::Covariance(covariance));
  return factor;
}

template <typename T>
gtsam::NonlinearFactor::shared_ptr parsePrior(std::function<T(json)> val_parser_fn, json input_json) {
  T measured = val_parser_fn(input_json["measurement"]);
  int d = gtsam::traits<T>::GetDimension(measured);
  gtsam::Matrix covariance = parseCovariance(input_json, d);
  typename gtsam::PriorFactor<T>::shared_ptr factor = boost::make_shared<gtsam::PriorFactor<T>>(
      input_json["key1"].get<uint64_t>(), measured, gtsam::noiseModel::Gaussian::Covariance(covariance));
  return factor;
}

}  //  namespace measurement_parser_functions

}  // namespace jrl