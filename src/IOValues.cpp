#include "jrl/IOValues.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace jrl {
namespace io_values {
/**********************************************************************************************************************/
// POSE2
gtsam::Pose2 parsePose2(json input_json) {
  double x = input_json["x"].get<double>();
  double y = input_json["y"].get<double>();
  double theta = input_json["theta"].get<double>();
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
  std::vector<double> t = input_json["translation"].get<std::vector<double>>();
  gtsam::Vector3 translation(t.data());
  std::vector<double> q = input_json["rotation"].get<std::vector<double>>();
  gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(q[0], q[1], q[2], q[3]);
  return gtsam::Pose3(rotation, translation);
}

json serializePose3(gtsam::Pose3 pose) {
  json output;
  output["type"] = Pose3Tag;
  gtsam::Vector q = pose.rotation().quaternion();
  output["translation"] = {pose.x(), pose.y(), pose.z()};
  output["rotation"] = {q(0), q(1), q(2), q(3)};
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

/**********************************************************************************************************************/
// Point2
gtsam::Point2 parsePoint2(json input_json) {
  double x = input_json["x"].get<double>();
  double y = input_json["y"].get<double>();
  return gtsam::Point2(x, y);
}

json serializePoint2(gtsam::Point2 point) {
  json output;
  output["type"] = Point2Tag;
  output["x"] = point.x();
  output["y"] = point.y();
  return output;
}

/**********************************************************************************************************************/
// Point3
gtsam::Point3 parsePoint3(json input_json) {
  double x = input_json["x"].get<double>();
  double y = input_json["y"].get<double>();
  double z = input_json["z"].get<double>();
  return gtsam::Point3(x, y, z);
}

json serializePoint3(gtsam::Point3 point) {
  json output;
  output["type"] = Point3Tag;
  output["x"] = point.x();
  output["y"] = point.y();
  output["z"] = point.z();
  return output;
}

}  // namespace io_values
}  // namespace jrl