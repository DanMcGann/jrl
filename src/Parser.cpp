#include "jrl/Parser.h"

#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <fstream>

#include "jrl/IOMeasurements.h"
#include "jrl/IOValues.h"

using namespace jrl::io_measurements;
using namespace jrl::io_values;

namespace jrl {

Parser::Parser() {
  value_accumulators_ = loadDefaultValueAccumulators();
  measurement_parsers_ = loadDefaultMeasurementParsers();
}

/**********************************************************************************************************************/
std::map<std::string, ValueParser> Parser::loadDefaultValueAccumulators() {
  // clang-format off
  std::map<std::string, ValueParser> parser_functions = {
      {Pose2Tag,        [](const json& input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<gtsam::Pose2>(&parse<gtsam::Pose2>, input, key, accum); }},
      {Pose3Tag,        [](const json& input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<gtsam::Pose3>(&parse<gtsam::Pose3>, input, key, accum); }},
      {Point2Tag,       [](const json& input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<gtsam::Point2>(&parse<gtsam::Point2>, input, key, accum); }},
      {Point3Tag,       [](const json& input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<gtsam::Point3>(&parse<gtsam::Point3>, input, key, accum); }},
      {VectorTag,       [](const json& input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<gtsam::Vector>(&parse<gtsam::Vector>, input, key, accum); }},
      {ScalarTag,       [](const json& input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<double>(&parse<double>, input, key, accum); }},
  };
  // clang-format on
  return parser_functions;
}

/**********************************************************************************************************************/
std::map<std::string, MeasurementParser> Parser::loadDefaultMeasurementParsers() {
  // clang-format off
  std::map<std::string, MeasurementParser> parser_functions = {
      {PriorFactorPose2Tag,         [](const json& input) { return parsePrior<gtsam::Pose2>(&parse<gtsam::Pose2>, input); }},
      {PriorFactorPose3Tag,         [](const json& input) { return parsePrior<gtsam::Pose3>(&parse<gtsam::Pose3>, input); }},
      {BetweenFactorPose2Tag,       [](const json& input) { return parseNoiseModel2<gtsam::Pose2, gtsam::BetweenFactor<gtsam::Pose2>>(&parse<gtsam::Pose2>, input); }},
      {BetweenFactorPose3Tag,       [](const json& input) { return parseNoiseModel2<gtsam::Pose3, gtsam::BetweenFactor<gtsam::Pose3>>(&parse<gtsam::Pose3>, input); }},
      {RangeFactorPose2Tag,         [](const json& input) { return parseNoiseModel2<double, gtsam::RangeFactor<gtsam::Pose2>>(&parse<double>, input); }},
      {RangeFactorPose3Tag,         [](const json& input) { return parseNoiseModel2<double, gtsam::RangeFactor<gtsam::Pose3>>(&parse<double>, input); }},
      {RangeFactor2DTag,            [](const json& input) { return parseNoiseModel2<double, gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2>>(&parse<double>, input); }},
      {RangeFactor3DTag,            [](const json& input) { return parseNoiseModel2<double, gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>>(&parse<double>, input); }},
      {BearingRangeFactorPose2Tag,  [](const json& input) { return parseNoiseModel2<gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2>, gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2>>(&parseBearingRange<gtsam::Pose2, gtsam::Pose2>, input); }},
      {BearingRangeFactorPose3Tag,  [](const json& input) { return parseNoiseModel2<gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3>, gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3>>(&parseBearingRange<gtsam::Pose3, gtsam::Pose3>, input); }},
      {BearingRangeFactor2DTag,     [](const json& input) { return parseNoiseModel2<gtsam::BearingRange<gtsam::Pose2, gtsam::Point2>, gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>>(&parseBearingRange<gtsam::Pose2, gtsam::Point2>, input); }},
      {BearingRangeFactor3DTag,     [](const json& input) { return parseNoiseModel2<gtsam::BearingRange<gtsam::Pose3, gtsam::Point3>, gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>>(&parseBearingRange<gtsam::Pose3, gtsam::Point3>, input); }},
      {PriorFactorPoint2Tag,        [](const json& input) { return parsePrior<gtsam::Point2>(&parse<gtsam::Point2>, input); }},
      {PriorFactorPoint3Tag,        [](const json& input) { return parsePrior<gtsam::Point3>(&parse<gtsam::Point3>, input); }},
      {BetweenFactorPoint2Tag,      [](const json& input) { return parseNoiseModel2<gtsam::Point2, gtsam::BetweenFactor<gtsam::Point2>>(&parse<gtsam::Point2>, input); }},
      {BetweenFactorPoint3Tag,      [](const json& input) { return parseNoiseModel2<gtsam::Point3, gtsam::BetweenFactor<gtsam::Point3>>(&parse<gtsam::Point3>, input); }},
  };
  // clang-format on
  return parser_functions;
}

/**********************************************************************************************************************/
TypedValues Parser::parseValues(const json& values_json) const {
  gtsam::Values values;
  ValueTypes value_types;
  for (auto& value_element : values_json) {
    gtsam::Key key = value_element["key"].get<uint64_t>();
    std::string type_tag = value_element["type"].get<std::string>();
    value_types[key] = type_tag;
    value_accumulators_.at(type_tag)(value_element, key, values);
  }
  return TypedValues(values, value_types);
}

/**********************************************************************************************************************/
std::vector<Entry> Parser::parseMeasurements(const json& measurements_json) const {
  std::vector<Entry> measurements;
  for (auto& entry_element : measurements_json) {
    uint64_t stamp = entry_element["stamp"].get<uint64_t>();
    gtsam::NonlinearFactorGraph entry_measurements;
    std::vector<std::string> type_tags;
    if(entry_element.contains("measurements")) {
      for (auto& measurement : entry_element["measurements"]) {
        std::string tag = measurement["type"].get<std::string>();
        type_tags.push_back(tag);
        entry_measurements.push_back(measurement_parsers_.at(tag)(measurement));
      }
    }
    std::map<gtsam::FactorIndex, bool> potential_outlier_statuses;
    if (entry_element.contains("potential_outlier_statuses")) {
      potential_outlier_statuses =
          entry_element["potential_outlier_statuses"].get<std::map<gtsam::FactorIndex, bool>>();
    }
    measurements.emplace_back(stamp, type_tags, entry_measurements, potential_outlier_statuses);
  }
  return measurements;
}

/**********************************************************************************************************************/
json Parser::parseJson(std::string input_file_name, bool decompress_from_cbor) const {
  json parsed_json;
  if (decompress_from_cbor) {
    std::ifstream ifs(input_file_name, std::ios::binary);
    parsed_json = json::from_cbor(nlohmann::detail::input_adapter{ifs});
  } else {
    std::ifstream ifs(input_file_name);
    parsed_json = json::parse(ifs);
  }
  return parsed_json;
}

/**********************************************************************************************************************/
Dataset Parser::parseDataset(std::string dataset_file, bool decompress_from_cbor) const {
  json dataset_json = parseJson(dataset_file, decompress_from_cbor);

  // Parse Header information
  std::string name = dataset_json["name"];
  std::vector<char> robots = dataset_json["robots"].get<std::vector<char>>();

  // Parse Ground truth if it exists
  boost::optional<std::map<char, TypedValues>> groundtruth = boost::none;
  if (dataset_json.contains("groundtruth")) {
    groundtruth = std::map<char, TypedValues>();
    for (auto& el : dataset_json["groundtruth"].items()) {
      (*groundtruth)[el.key()[0]] = parseValues(el.value());
    }
  }

  // Parse Initialization if it exists
  boost::optional<std::map<char, TypedValues>> initialization = boost::none;
  if (dataset_json.contains("initialization")) {
    initialization = std::map<char, TypedValues>();
    for (auto& el : dataset_json["initialization"].items()) {
      (*initialization)[el.key()[0]] = parseValues(el.value());
    }
  }

  // Parse Measurements if it exists
  std::map<char, std::vector<Entry>> measurements;
  for (auto& el : dataset_json["measurements"].items()) {
    measurements[el.key()[0]] = parseMeasurements(el.value());
  }

  return Dataset(name, robots, measurements, groundtruth, initialization);
}

/**********************************************************************************************************************/
Results Parser::parseResults(std::string results_file, bool decompress_from_cbor) const {
  json results_json = parseJson(results_file, decompress_from_cbor);

  // Parse Header information
  std::string dataset_name = results_json["dataset_name"];
  std::string method_name = results_json["method_name"];
  std::vector<char> robots = results_json["robots"].get<std::vector<char>>();

  // Parse Ground truth if it exists
  std::map<char, TypedValues> solutions;
  for (auto& el : results_json["solutions"].items()) {
    solutions[el.key()[0]] = parseValues(el.value());
  }
  return Results(dataset_name, method_name, robots, solutions);
}

/**********************************************************************************************************************/
MetricSummary Parser::parseMetricSummary(std::string metric_summary_file, bool decompress_from_cbor) const {
  json results_json = parseJson(metric_summary_file, decompress_from_cbor);
  MetricSummary metric_summary;
  metric_summary.dataset_name = results_json["dataset_name"];
  metric_summary.method_name = results_json["method_name"];
  metric_summary.robots = results_json["robots"].get<std::vector<char>>();

  if (results_json.contains("robot_ate")) {
    metric_summary.robot_ate = results_json["robot_ate"].get<std::map<char, std::pair<double, double>>>();
  }
  if (results_json.contains("total_ate")) {
    metric_summary.total_ate = results_json["total_ate"].get<std::pair<double, double>>();
  }
  if (results_json.contains("sve")) {
    metric_summary.sve = results_json["sve"].get<std::pair<double, double>>();
  }
  if (results_json.contains("mean_residual")) {
    metric_summary.mean_residual = results_json["mean_residual"].get<double>();
  }

  return metric_summary;
}

}  // namespace jrl
