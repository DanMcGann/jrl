#include "jrl/IOValues.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace jrl {
namespace io_values {
/**********************************************************************************************************************/
// POSE2
gtsam::Pose2 parsePose2(json input_json) {
  std::cout << "getting x " << std::endl;
  double x = input_json["x"].get<double>();
  std::cout << "getting y " << std::endl;
  double y = input_json["y"].get<double>();
  std::cout << "getting theta " << std::endl;
  ;
  double theta = input_json["theta"].get<double>();
  std::cout << "building pose2" << std::endl;
  return gtsam::Pose2(x, y, theta);
}

json serializePose2(gtsam::Pose2 pose) {
  json output;
  output["type"] = Pose2Tag;
  output["x"] = pose.x();
  output["y"] = pose.y();
  output["theta"] = pose.theta();
  return output;
}

/**********************************************************************************************************************/
// POSE3
gtsam::Pose3 parsePose3(json input_json) {
  std::cout << "parsePose3" << std::endl;
  gtsam::Vector3 translation(input_json["translation"].get<std::vector<double>>().data());
  std::vector<double> q = input_json["rotation"].get<std::vector<double>>();
  gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(q[0], q[1], q[2], q[3]);
  return gtsam::Pose3(rotation, translation);
}

json serializePose3(gtsam::Pose3 pose) {
  json output;
  output["type"] = Pose3Tag;
  auto q = pose.rotation().quaternion();
  output["translation"] = {pose.x(), pose.y(), pose.z()};
  output["rotation"] = {q.w(), q.x(), q.y(), q.z()};
  return output;
}

/**********************************************************************************************************************/
// VECTOR
gtsam::Vector parseVector(json input_json) {
  std::vector<double> stdvec = input_json["data"].get<std::vector<double>>();
  gtsam::Vector eigvec = Eigen::Map<gtsam::Vector>(stdvec.data(), stdvec.size());
  return eigvec;
}

json serializeVector(gtsam::Vector vec) {
  json output;
  output["type"] = VectorTag;
  output["data"] = std::vector<double>(vec.data(), vec.data() + vec.size());
  return output;
}

}  // namespace io_values
}  // namespace jrl