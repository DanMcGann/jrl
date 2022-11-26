#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;
namespace jrl {

static const std::string Pose2Tag = "Pose2";
static const std::string Pose3Tag = "Pose3";
static const std::string VectorTag = "Vector";
static const std::string ScalarTag = "Scalar";

namespace io_values {

template <typename VALUE>
void valueAccumulator(std::function<VALUE(json)> parser, json input_json, gtsam::Key key, gtsam::Values& accum) {
  accum.insert(key, parser(input_json));
}

/**********************************************************************************************************************/
// POSE2
gtsam::Pose2 parsePose2(json input_json);
json serializePose2(gtsam::Pose2 pose);

/**********************************************************************************************************************/
// POSE3
gtsam::Pose3 parsePose3(json input_json);
json serializePose3(gtsam::Pose3 pose);

/**********************************************************************************************************************/
// VECTOR
gtsam::Vector parseVector(json input_json);
json serializeVector(gtsam::Vector vec);

/**********************************************************************************************************************/
// SCALAR
template <typename T>
T parseScalar(json input_json) {
  return input_json["value"].get<T>();
}

template <typename T>
json serializeScalar(T value) {
  json output;
  output["type"] = ScalarTag;
  output["value"] = value;
  return output;
}

}  // namespace io_values

}  // namespace jrl