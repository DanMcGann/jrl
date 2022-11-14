#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;
namespace jrl {

namespace measurement_parser_functions {

/**********************************************************************************************************************/
template <typename T>
void parseBetween(std::function<T(json)> val_parser_fn, input_json) {
  T measured = val_parser_fn(input_json["measurement"]);
  int d = gtsam::traits<T>::GetDimension(measured);
  gtsam::Matrix covariance = gtsam::Matrix(input_json["covariance"].get<std::vector<double>>().data(), d, 2 * d);
  BetweenFactor<T>::shared_ptr factor =
      BetweenFactor<T>::make_shared(input_json["key1"].get<uint64_t>(), input_json["key2"].get<uint64_t>(), measured,
                                    gtsam::noiseModel::Gaussian::Covariance(covariance));
  return factor;
}

template <typename T>
void parsePrior(std::function<T(json)> val_parser_fn, input_json) {
  T measured = val_parser_fn(input_json["measurement"]);
  int d = gtsam::traits<T>::GetDimension(measured);
  gtsam::Matrix covariance = gtsam::Matrix(input_json["covariance"].get<std::vector<double>>().data(), d, d);
  BetweenFactor<T>::shared_ptr factor = BetweenFactor<T>::make_shared(
      input_json["key1"].get<uint64_t>(), measured, gtsam::noiseModel::Gaussian::Covariance(covariance));
  return factor;
}

}  //  namespace measurement_parser_functions

}  // namespace jrl