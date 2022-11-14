#pragma once

#include "jrl/Parser.h"
#include "jrl/ValueParserFunctions.h"

using measurement_parser_functions;
using value_parser_functions;

namespace jrl {
/**********************************************************************************************************************/
std::map<std::string, ValueParser> Parser::loadDefaultValueParsers() {
  std::map<std::string, ValueParser> parser_functions = {
      {"Pose2", [](json input, gtsam::Key,
                   gtsam::Values& accum) { return valueAccumulator<gtsam::Pose2>(&parsePose2, input, key, accum); }},
      {"Pose2", [](json input, gtsam::Key,
                   gtsam::Values& accum) { return valueAccumulator<gtsam::Pose3>(&parsePose3, input, key, accum); }},
  };
  return parser_functions;
}

/**********************************************************************************************************************/
std::map<std::string, MeasurementParser> Parser::loadDefaultMeasurementParsers() {
  std::map<std::string, MeasurementParser> parser_functions = {
      {"PriorFactorPose2", [](json input) { return parsePrior<gtsam::Pose2>(&parsePose2, input); }},
      {"PriorFactorPose3", [](json input) { return parsePrior<gtsam::Pose3>(&parsePose3, input); }},
      {"BetweenFactorPose2", [](json input) { return parseBetween<gtsam::Pose2>(&parsePose2, input); }},
      {"BetweenFactorPose3", [](json input) { return parseBetween<gtsam::Pose3>(&parsePose3, input); }},
  };

  return parser_functions;
}

/**********************************************************************************************************************/
gtsam::Values Parser::parseValues(json values_json) {
  gtsam::Values values;
  for (auto& value_element : values_json.items()) {
    std::string type_tag = value_element["type"].get<std::string>();
    value_accumulators_[type_tag](value_element.value(), value_element.key(), values);
  }
  return values;
}

/**********************************************************************************************************************/
std::vector<Entry> Parser::parseMeasurements(json measurements_json) {
  std::vector<Entry> measurements;
  for (auto& measure_element : measurements_json.items()) {
    uint64_t stamp = value_element["stamp"].get<uint64_t>();
    std::string type_tag = value_element["type"].get<std::string>();
    measurements.push_back(Entry(stamp, type_tag, measurement_parsers_[type_tag](measure_element)));
  }
  return measurements;
}

/**********************************************************************************************************************/
Dataset Parser::parse(std::string dataset_file) {
  std::ifstream ifs("dataset_file.json");
  json dataset_json = json::parse(ifs);

  // Parse Header information
  std::string name = dataset_json["name"];
  std::vector<char> robots = dataset_json["robots"].get<std::vector<char>>();

  // Parse Ground truth if it exists
  boost::optional<std::map<char, gtsam::Values>> groundtruth = boost::none;
  if (dataset_json.contains("groundtruth")) {
    groundtruth = std::map<char, gtsam::Values>();
    for (auto& el : dataset_json["groundtruth"].items()) {
      groundtruth[el.key().get<char>()] = parseValues(el.value());
    }
  }

  // Parse Initialization if it exists
  boost::optional<std::map<char, gtsam::Values>> initialization = boost::none;
  if (dataset_json.contains("initialization")) {
    initialization = std::map<char, gtsam::Values>();
    for (auto& el : dataset_json["initialization"].items()) {
      initialization[el.key().get<char>()] = parseValues(el.value());
    }
  }

  // Parse Measurements if it exists
  std::map<char, std::vector<Entry>> measurements;
  for (auto& el : dataset_json["measurements"].items()) {
    measurements[el.key().get<char>()] = parseMeasurements(el.value());
  }

  return Dataset(name, robots, measurements, groundtruth, initialization);
}

}  // namespace jrl