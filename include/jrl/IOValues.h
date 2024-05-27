#pragma once
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;
namespace jrl {

static const std::string Rot2Tag = "Rot2";
static const std::string Pose2Tag = "Pose2";
static const std::string Rot3Tag = "Rot3";
static const std::string Pose3Tag = "Pose3";
static const std::string Point2Tag = "Point2";
static const std::string Point3Tag = "Point3";
static const std::string Unit3Tag = "Unit3";
static const std::string VectorTag = "Vector";
static const std::string ScalarTag = "Scalar";
static const std::string BearingRangeTag = "BearingRange";

namespace io_values {

/** @brief Generic function to accumulate a value (represented as json) into a gtsam::Values struct
 * @param parser: The parser function to parse input_json
 * @param input_json: The value to accumulate represented as json
 * @param key: The unique identifier of the value
 * @param accum: The values to which we add the value
*/
template <typename VALUE>
void valueAccumulator(std::function<VALUE(const json&)> parser, const json& input_json, gtsam::Key key, gtsam::Values& accum) {
  accum.insert(key, parser(input_json));
}

/**********************************************************************************************************************/
template <typename T>
T parse(const json& input_json) {  // Base Parse function for builtins. Specialization provided for all iovalues
  return input_json["value"].get<T>();
}

template <typename T>
json serialize(T obj) {  // Base Parse function for builtins. Specialization provided for all iovalues
  json output;
  output["type"] = ScalarTag;
  output["value"] = obj;
  return output;
}


/// Below we have declarations for template specializations for each value supported by iovalues
/**********************************************************************************************************************/
// Rot2
template <>
gtsam::Rot2 parse<gtsam::Rot2>(const json& input_json);
template <>
json serialize<gtsam::Rot2>(gtsam::Rot2 pose);

/**********************************************************************************************************************/
// POSE2
template <>
gtsam::Pose2 parse<gtsam::Pose2>(const json& input_json);
template <>
json serialize<gtsam::Pose2>(gtsam::Pose2 pose);

/**********************************************************************************************************************/
// Rot3
template <>
gtsam::Rot3 parse<gtsam::Rot3>(const json& input_json);
template <>
json serialize<gtsam::Rot3>(gtsam::Rot3 pose);

/**********************************************************************************************************************/
// POSE3
template <>
gtsam::Pose3 parse<gtsam::Pose3>(const json& input_json);
template <>
json serialize<gtsam::Pose3>(gtsam::Pose3 pose);

/**********************************************************************************************************************/
// VECTOR
template <>
gtsam::Vector parse<gtsam::Vector>(const json& input_json);
template <>
json serialize<gtsam::Vector>(gtsam::Vector vec);

/**********************************************************************************************************************/
// Point2
template <>
gtsam::Point2 parse<gtsam::Point2>(const json& input_json);
template <>
json serialize<gtsam::Point2>(gtsam::Point2 point);

/**********************************************************************************************************************/
// Point3
template <>
gtsam::Point3 parse<gtsam::Point3>(const json& input_json);
template <>
json serialize<gtsam::Point3>(gtsam::Point3 point);

/**********************************************************************************************************************/
// Unit3
template <>
gtsam::Unit3 parse<gtsam::Unit3>(const json& input_json);
template <>
json serialize<gtsam::Unit3>(gtsam::Unit3 point);

/**********************************************************************************************************************/
// BearingRange Need special treatment because c++ does not allow function partial specialization
template <typename A1, typename A2, typename B = typename gtsam::Bearing<A1, A2>::result_type,
          typename R = typename gtsam::Range<A1, A2>::result_type>
gtsam::BearingRange<A1, A2> parseBearingRange(const json& input_json) {
  B bearing = parse<B>(input_json["bearing"]);
  R range = parse<R>(input_json["range"]);
  return gtsam::BearingRange<A1, A2>(bearing, range);
}

template <typename A1, typename A2, typename B = typename gtsam::Bearing<A1, A2>::result_type,
          typename R = typename gtsam::Range<A1, A2>::result_type>
json serializeBearingRange(gtsam::BearingRange<A1, A2> bearingrange) {
  json output;
  output["type"] = BearingRangeTag;
  output["bearing"] = io_values::serialize<B>(bearingrange.bearing());
  output["range"] = io_values::serialize<R>(bearingrange.range());
  return output;
}

}  // namespace io_values

}  // namespace jrl