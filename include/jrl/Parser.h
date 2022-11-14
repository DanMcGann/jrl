#pragma once
#include <gtsam/nonlinear/Values.h>

#include <nlohmann/json.hpp>

#include "jrl/Dataset.h"
using json = nlohmann::json;

namespace jrl {

typedef std::function<void(gtsam::Key, json, gtsam::Values)> ValueParser;
typedef std::function<Entry(json)> MeasurementParser;

class Parser {
  // Members
 private:
  std::map<std::string, ValueParser> value_accumulators_;
  std::map<std::string, ValueParser> loadDefaultValueAccumulators();

  std::map<std::string, MeasurementParser> measurement_parsers_;
  std::map<std::string, MeasurementParser> loadDefaultMeasurementParsers();

  gtsam::Values parseValues(json values_json);
  std::vector<Entry> parseMeasurements(json measurements_json);
  // Interface
 public:
  Parser();
  void registerValueParser(std::string tag, ValueParser parser_fn);
  void registerMeasurementParser(std::string tag, MeasurementParser parser_fn);
  Dataset parse(std::string dataset_file);
};

}  // namespace jrl