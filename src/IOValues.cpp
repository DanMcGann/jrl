#include "jrl/IOValues.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace jrl {
namespace io_values {

/**********************************************************************************************************************/
// Rot2
template <>
gtsam::Rot2 parse<gtsam::Rot2>(const json& input_json) {
  double theta = input_json["theta"].get<double>();
  return gtsam::Rot2(theta);
}

template <>
json serialize<gtsam::Rot2>(gtsam::Rot2 rot) {
  json output;
  output["type"] = Rot2Tag;
  output["theta"] = rot.theta();
  return output;
}

/**********************************************************************************************************************/
// POSE2
template <>
gtsam::Pose2 parse<gtsam::Pose2>(const json& input_json) {
  double x = input_json["x"].get<double>();
  double y = input_json["y"].get<double>();
  double theta = input_json["theta"].get<double>();
  return gtsam::Pose2(x, y, theta);
}

template <>
json serialize<gtsam::Pose2>(gtsam::Pose2 pose) {
  json output;
  output["type"] = Pose2Tag;
  output["x"] = pose.x();
  output["y"] = pose.y();
  output["theta"] = pose.theta();
  return output;
}

/**********************************************************************************************************************/
// Rot3
template <>
gtsam::Rot3 parse<gtsam::Rot3>(const json& input_json) {
  std::vector<double> q = input_json["rotation"].get<std::vector<double>>();
  return gtsam::Rot3::Quaternion(q[0], q[1], q[2], q[3]);
}

template <>
json serialize<gtsam::Rot3>(gtsam::Rot3 rot) {
  json output;
  output["type"] = Rot3Tag;
  gtsam::Vector q = rot.quaternion();
  output["rotation"] = {q(0), q(1), q(2), q(3)};
  return output;
}

/**********************************************************************************************************************/
// POSE3
template <>
gtsam::Pose3 parse<gtsam::Pose3>(const json& input_json) {
  std::vector<double> t = input_json["translation"].get<std::vector<double>>();
  gtsam::Vector3 translation(t.data());
  std::vector<double> q = input_json["rotation"].get<std::vector<double>>();
  gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(q[0], q[1], q[2], q[3]);
  return gtsam::Pose3(rotation, translation);
}

template <>
json serialize<gtsam::Pose3>(gtsam::Pose3 pose) {
  json output;
  output["type"] = Pose3Tag;
  gtsam::Vector q = pose.rotation().quaternion();
  output["translation"] = {pose.x(), pose.y(), pose.z()};
  output["rotation"] = {q(0), q(1), q(2), q(3)};
  return output;
}

/**********************************************************************************************************************/
// VECTOR
template <>
gtsam::Vector parse<gtsam::Vector>(const json& input_json) {
  std::vector<double> stdvec = input_json["data"].get<std::vector<double>>();
  gtsam::Vector eigvec = Eigen::Map<gtsam::Vector>(stdvec.data(), stdvec.size());
  return eigvec;
}

template <>
json serialize<gtsam::Vector>(gtsam::Vector vec) {
  json output;
  output["type"] = VectorTag;
  output["data"] = std::vector<double>(vec.data(), vec.data() + vec.size());
  return output;
}

/**********************************************************************************************************************/
// Point2
template <>
gtsam::Point2 parse<gtsam::Point2>(const json& input_json) {
  double x = input_json["x"].get<double>();
  double y = input_json["y"].get<double>();
  return gtsam::Point2(x, y);
}

template <>
json serialize<gtsam::Point2>(gtsam::Point2 point) {
  json output;
  output["type"] = Point2Tag;
  output["x"] = point.x();
  output["y"] = point.y();
  return output;
}

/**********************************************************************************************************************/
// Point3
template <>
gtsam::Point3 parse<gtsam::Point3>(const json& input_json) {
  double x = input_json["x"].get<double>();
  double y = input_json["y"].get<double>();
  double z = input_json["z"].get<double>();
  return gtsam::Point3(x, y, z);
}

template <>
json serialize<gtsam::Point3>(gtsam::Point3 point) {
  json output;
  output["type"] = Point3Tag;
  output["x"] = point.x();
  output["y"] = point.y();
  output["z"] = point.z();
  return output;
}


/**********************************************************************************************************************/
// Unit3
template <>
gtsam::Unit3 parse<gtsam::Unit3>(const json& input_json) {
  double i = input_json["i"].get<double>();
  double j = input_json["j"].get<double>();
  double k = input_json["k"].get<double>();
  return gtsam::Unit3(i, j, k);
}

template <>
json serialize<gtsam::Unit3>(gtsam::Unit3 unit) {
  json output;
  gtsam::Point3 p = unit.point3();
  output["type"] = Unit3Tag;
  output["i"] = p.x();
  output["j"] = p.y();
  output["k"] = p.z();
  return output;
}


}  // namespace io_values
}  // namespace jrl