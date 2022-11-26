#pragma once
#include <gtsam/nonlinear/Values.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "jrl/Dataset.h"
#include "jrl/Results.h"

namespace jrl {

typedef std::function<json(gtsam::Key&, gtsam::Values&)> ValueSerializer;
typedef std::function<json(gtsam::NonlinearFactor::shared_ptr&)> MeasurementSerializer;

class Writer {
  // Members
 private:
  std::map<std::string, ValueSerializer> value_serializers_;
  std::map<std::string, ValueSerializer> loadDefaultValueSerializers();

  std::map<std::string, MeasurementSerializer> measurement_serializers_;
  std::map<std::string, MeasurementSerializer> loadDefaultMeasurementSerializers();

  json serializeValues(TypedValues typed_values);
  json serializeMeasurements(std::vector<Entry> measurements);
  // Interface
 public:
  Writer();
  void writeDataset(Dataset dataset, std::string output_file_name);
  void writeResults(Results results, std::string output_file_name);

  // TODO
  // void registerValueSerializer(std::string tag, ValueSerializer serializer_fn);
  // void registerMeasurementParser(std::string tag, MeasurementSerializer serializer_fn);
};

}  // namespace jrl