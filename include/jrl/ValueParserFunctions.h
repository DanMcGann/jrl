#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;
namespace jrl {

namespace value_parser_functions {

template <typename VALUE>
void valueAccumulator(std::function<VALUE(json)> parser, json input_json, gtsam::Key, gtsam::Values& accum) {
  accum.insert(key, parser(input_json));
}

/**********************************************************************************************************************/
// POSE2
gtsam::Pose2 parsePose2(json input_json) {
  return gtsam::Pose2(input_json["x"].get<double>(), input_json["y"].get<double>(), input_json["theta"].get<double>());
}

/**********************************************************************************************************************/
// POSE3
gtsam::Pose3 parsePose3(json input_json) {
  gtsam::Vector translation(input_json["translation"].get < std::vector<double>().data());
  std::vector<double> q = input_json["rotation"].get < std::vector<double>();
  gtsam::Rot3::Quaternion rotation(q[0], q[1], q[2], q[3]);
  return gtsam::Pose3(rotation, translation);
}

}  //  namespace value_parser_functions

}  // namespace jrl