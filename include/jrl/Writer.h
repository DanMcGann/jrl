#pragma once
#include <gtsam/nonlinear/Values.h>

#include <nlohmann/json.hpp>

#include "jrl/Dataset.h"
using json = nlohmann::json;

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

  json serializeValues(std::pair<gtsam::Values, ValueTypes>);
  json serializeMeasurements(std::vector<Entry>);
  // Interface
 public:
  Writer();
  void write(Dataset dataset, std::string output_file_name);

  // TODO
  // void registerValueSerializer(std::string tag, ValueSerializer serializer_fn);
  // void registerMeasurementParser(std::string tag, MeasurementSerializer serializer_fn);
};

}  // namespace jrl