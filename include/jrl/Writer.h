#pragma once
#include <gtsam/nonlinear/Values.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "jrl/Dataset.h"
#include "jrl/Metrics.h"
#include "jrl/Results.h"

namespace jrl {
/// @brief Serializes the value from values at key into json
typedef std::function<json(gtsam::Key&, gtsam::Values&)> ValueSerializer;
/// @brief Serializes the factor into json
typedef std::function<json(gtsam::NonlinearFactor::shared_ptr&)> MeasurementSerializer;

class Writer {
  /** Members **/
 private:
  /// @brief Mapping from value tag to corresponding value serialization function
  std::map<std::string, ValueSerializer> value_serializers_;
  /** @brief Implicitly defines default value serializers
   *  @return The default value serializers, which are loaded into value_serializers_ on initialization
   **/
  std::map<std::string, ValueSerializer> loadDefaultValueSerializers();

  /// @brief Mapping from measurement tag to corresponding measurement serialization function
  std::map<std::string, MeasurementSerializer> measurement_serializers_;
  /** @brief Implicitly defines default measurement serializers
   *  @return The default measurement serializers, which are loaded into measurement_serializers_ on initialization
   **/
  std::map<std::string, MeasurementSerializer> loadDefaultMeasurementSerializers();

  /** @brief Writes arbitrary JSON To file
   * @param output_json: The json to write
   * @param output_file_name: The file in which to save the json
   * @param compress_to_cbor: if true indicates that written files should be compressed with cbor
   */
  void writeJson(json output_json, std::string output_file_name, bool compress_to_cbor) const;

  /** Interface **/
 public:
  /// @brief Constructors a writer object
  Writer();

  /// @brief Serializes dataset and writes to file
  /// @param compress_with_cbor if true indicates that written files should be compressed with cbor
  void writeDataset(Dataset dataset, std::string output_file_name, bool compress_to_cbor = false) const;

  /// @brief Serializes results and writes to file
  /// @param compress_with_cbor if true indicates that written files should be compressed with cbor
  void writeResults(Results results, std::string output_file_name, bool compress_to_cbor = false) const;

  /// @brief Serializes a MetricSummary and writes to file
  /// @param compress_with_cbor if true indicates that written files should be compressed with cbor
  void writeMetricSummary(MetricSummary metric_summary, std::string output_file_name,
                          bool compress_to_cbor = false) const;

  /** @brief Serializes all values using the loaded value serializers
   *  @param typed_values Input values and types
   *  @return JSON serialized values
   **/
  json serializeValues(TypedValues typed_values) const;

  /** @brief Serializes all measurements using the loaded measurement serializers
   *  @param measurements Input entries containing temporally ordered measurements
   *  @return JSON serialized measurements
   **/
  json serializeMeasurements(std::vector<Entry> measurements) const;

  /// @brief Register a custom value serializer
  /// @param tag Tag to associate with the value
  /// @param serializer_fn Serializer function to serialize the value
  void registerValueSerializer(std::string tag, ValueSerializer serializer_fn) {
    value_serializers_[tag] = serializer_fn;
  };

  /// @brief Register a custom measurement serializer
  /// @param tag Tag to associate with the measurement
  /// @param serializer_fn Serializer function to serialize the measurement
  void registerMeasurementSerializer(std::string tag, MeasurementSerializer serializer_fn) {
    measurement_serializers_[tag] = serializer_fn;
  };
};

}  // namespace jrl