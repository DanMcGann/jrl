#include "jrl/IOValues.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace jrl {
namespace io_values {
/**********************************************************************************************************************/
// POSE2
gtsam::Pose2 parsePose2(json input_json) {
  return gtsam::Pose2(input_json["x"].get<double>(), input_json["y"].get<double>(), input_json["theta"].get<double>());
}

json serializePose2(gtsam::Pose2 pose) {
  json output;
  output["type"] = "Pose2";
  output["x"] = pose.x();
  output["y"] = pose.y();
  output["theta"] = pose.theta();
  return output;
}

/**********************************************************************************************************************/
// POSE3
gtsam::Pose3 parsePose3(json input_json) {
  gtsam::Vector3 translation(input_json["translation"].get<std::vector<double>>().data());
  std::vector<double> q = input_json["rotation"].get<std::vector<double>>();
  gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(q[0], q[1], q[2], q[3]);
  return gtsam::Pose3(rotation, translation);
}

json serializePose3(gtsam::Pose3 pose) {
  json output;
  output["type"] = "Pose3";
  auto q = pose.rotation().quaternion();
  output["translation"] = {pose.x(), pose.y(), pose.z()};
  output["rotation"] = {q.w(), q.x(), q.y(), q.z()};
  return output;
}

}  // namespace io_values
}  // namespace jrl