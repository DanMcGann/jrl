#include "jrl/Parser.h"

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
      {Pose2Tag, [](json input, gtsam::Key key,gtsam::Values& accum)  { return valueAccumulator<gtsam::Pose2>(&parsePose2, input, key, accum); }},
      {Pose3Tag, [](json input, gtsam::Key key, gtsam::Values& accum) { return valueAccumulator<gtsam::Pose3>(&parsePose3, input, key, accum); }}
  };
  // clang-format on
  return parser_functions;
}

/**********************************************************************************************************************/
std::map<std::string, MeasurementParser> Parser::loadDefaultMeasurementParsers() {
  // clang-format off
  std::map<std::string, MeasurementParser> parser_functions = {
      {PriorFactorPose2Tag, [](json input) { return parsePrior<gtsam::Pose2>(&parsePose2, input); }},
      {PriorFactorPose3Tag, [](json input) { return parsePrior<gtsam::Pose3>(&parsePose3, input); }},
      {BetweenFactorPose2Tag, [](json input) { return parseBetween<gtsam::Pose2>(&parsePose2, input); }},
      {BetweenFactorPose3Tag, [](json input) { return parseBetween<gtsam::Pose3>(&parsePose3, input); }}
  };
  // clang-format on
  return parser_functions;
}

/**********************************************************************************************************************/
std::pair<gtsam::Values, ValueTypes> Parser::parseValues(json values_json) {
  std::cout << "ParseValues" << std::endl;
  gtsam::Values values;
  ValueTypes value_types;
  for (auto& value_element : values_json) {
    std::cout << value_element << std::endl;
    std::cout << "Getting Key" << std::endl;
    gtsam::Key key = value_element["key"].get<uint64_t>();
    std::cout << "Getting type" << std::endl;
    std::string type_tag = value_element["type"].get<std::string>();
    value_types[key] = type_tag;
    std::cout << "parsing value info" << std::endl;
    value_accumulators_[type_tag](value_element, key, values);
    std::cout << "Done with value" << std::endl;
  }
  return std::make_pair(values, value_types);
}

/**********************************************************************************************************************/
std::vector<Entry> Parser::parseMeasurements(json measurements_json) {
  std::vector<Entry> measurements;
  for (auto& entry_element : measurements_json) {
    uint64_t stamp = entry_element["stamp"].get<uint64_t>();
    gtsam::NonlinearFactorGraph entry_measurements;
    std::vector<std::string> type_tags;
    for (auto& measurement : entry_element["measurements"]) {
      std::string tag = measurement["type"].get<std::string>();
      type_tags.push_back(tag);
      entry_measurements.push_back(measurement_parsers_[tag](measurement));
    }
    measurements.push_back(Entry(stamp, type_tags, entry_measurements));
  }
  return measurements;
}

/**********************************************************************************************************************/
Dataset Parser::parse(std::string dataset_file) {
  std::ifstream ifs(dataset_file);
  json dataset_json = json::parse(ifs);

  // Parse Header information
  std::string name = dataset_json["name"];
  std::vector<char> robots = dataset_json["robots"].get<std::vector<char>>();

  // Parse Ground truth if it exists
  boost::optional<std::map<char, std::pair<gtsam::Values, ValueTypes>>> groundtruth = boost::none;
  if (dataset_json.contains("groundtruth")) {
    groundtruth = std::map<char, std::pair<gtsam::Values, ValueTypes>>();
    for (auto& el : dataset_json["groundtruth"].items()) {
      (*groundtruth)[el.key()[0]] = parseValues(el.value());
    }
  }

  // Parse Initialization if it exists
  boost::optional<std::map<char, std::pair<gtsam::Values, ValueTypes>>> initialization = boost::none;
  if (dataset_json.contains("initialization")) {
    initialization = std::map<char, std::pair<gtsam::Values, ValueTypes>>();
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

}  // namespace jrl