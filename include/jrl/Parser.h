#pragma once
#include <gtsam/nonlinear/Values.h>

#include <nlohmann/json.hpp>

#include "jrl/Dataset.h"
#include "jrl/Results.h"
using json = nlohmann::json;

namespace jrl {

typedef std::function<void(json, gtsam::Key& key, gtsam::Values&)> ValueParser;
typedef std::function<gtsam::NonlinearFactor::shared_ptr(json)> MeasurementParser;

class Parser {
  // Members
 private:
  std::map<std::string, ValueParser> value_accumulators_;
  std::map<std::string, ValueParser> loadDefaultValueAccumulators();

  std::map<std::string, MeasurementParser> measurement_parsers_;
  std::map<std::string, MeasurementParser> loadDefaultMeasurementParsers();

  TypedValues parseValues(json values_json);
  std::vector<Entry> parseMeasurements(json measurements_json);
  // Interface
 public:
  Parser();
  Dataset parseDataset(std::string dataset_file);

  Results parseResults(std::string results_file);

  // TODO
  // void registerValueParser(std::string tag, ValueParser parser_fn);
  // void registerMeasurementParser(std::string tag, MeasurementParser parser_fn);
};

}  // namespace jrl