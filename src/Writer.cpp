#include "jrl/Writer.h"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/sam/RangeFactor.h>

#include <iomanip>

#include "jrl/IOMeasurements.h"
#include "jrl/IOValues.h"

using namespace jrl::io_measurements;
using namespace jrl::io_values;

namespace jrl {

Writer::Writer() {
  value_serializers_ = loadDefaultValueSerializers();
  measurement_serializers_ = loadDefaultMeasurementSerializers();
}

/**********************************************************************************************************************/
std::map<std::string, ValueSerializer> Writer::loadDefaultValueSerializers() {
  // clang-format off
  std::map<std::string, ValueSerializer> serializer_functions = {
    {Pose2Tag, [](gtsam::Key key, gtsam::Values& vals) { return serializePose2(vals.at<gtsam::Pose2>(key)); }},
    {Pose3Tag, [](gtsam::Key key, gtsam::Values& vals) { return serializePose3(vals.at<gtsam::Pose3>(key)); }},
    {VectorTag, [](gtsam::Key key, gtsam::Values& vals) { return serializeVector(vals.at<gtsam::Vector>(key)); }},
    {ScalarTag, [](gtsam::Key key, gtsam::Values& vals) { return serializeScalar<double>(vals.at<double>(key)); }},
  };
  // clang-format on
  return serializer_functions;
}

/**********************************************************************************************************************/
std::map<std::string, MeasurementSerializer> Writer::loadDefaultMeasurementSerializers() {
  // clang-format off
  std::map<std::string, MeasurementSerializer> serializer_functions = {
    {PriorFactorPose2Tag,   [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializePrior<gtsam::Pose2>(&serializePose2, PriorFactorPose2Tag, factor); }},
    {PriorFactorPose3Tag,   [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializePrior<gtsam::Pose3>(&serializePose3, PriorFactorPose3Tag, factor); }},
    {BetweenFactorPose2Tag, [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<gtsam::Pose2, gtsam::BetweenFactor<gtsam::Pose2>>(&serializePose2, BetweenFactorPose2Tag, factor); }},
    {BetweenFactorPose3Tag, [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<gtsam::Pose3, gtsam::BetweenFactor<gtsam::Pose3>>(&serializePose3, BetweenFactorPose2Tag, factor); }},
    {RangeFactorPose2Tag,   [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<double, gtsam::RangeFactor<gtsam::Pose2>>(&serializeScalar<double>, RangeFactorPose2Tag, factor); }},
    {RangeFactorPose3Tag,   [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<double, gtsam::RangeFactor<gtsam::Pose3>>(&serializeScalar<double>, RangeFactorPose3Tag, factor); }},
  };
  // clang-format onRangeFactorPose3Tag

  return serializer_functions;
}

/**********************************************************************************************************************/
json Writer::serializeValues(std::pair<gtsam::Values, ValueTypes> values_info) {
  gtsam::Values values = values_info.first;
  ValueTypes types = values_info.second;
  json output;
  for (auto& ktpair : types) {
    gtsam::Key key = ktpair.first;
    std::string type_tag = ktpair.second;
    json value_obj = value_serializers_[type_tag](key, values);
    value_obj["key"] = key;
    output.push_back(value_obj);
  }
  return output;
}

/**********************************************************************************************************************/
json Writer::serializeMeasurements(std::vector<Entry> entries) {
  json output;
  for (auto& entry : entries) {
    json entry_obj;
    entry_obj["stamp"] = entry.stamp;
    for (int i = 0; i < entry.measurements.nrFactors(); i++) {
      std::string measurement_type = entry.measurement_types[i];
      gtsam::NonlinearFactor::shared_ptr factor = entry.measurements.at(i);
      entry_obj["measurements"].push_back(measurement_serializers_[measurement_type](factor));
    }
    output.push_back(entry_obj);
  }
  return output;
}

/**********************************************************************************************************************/
void Writer::write(Dataset dataset, std::string output_file_name) {
  json output_json;

  // serialize Header information
  output_json["name"] = dataset.name();
  output_json["robots"] = dataset.robots();

  // serialize Measurements
  json measurements_json;
  for (auto& robot : dataset.robots()) {
    measurements_json[std::string(1, robot)] = serializeMeasurements(dataset.measurements(robot));
  }
  output_json["measurements"] = measurements_json;

  // Serialize Ground truth if it exists
  json groundtruth_json;
  if (dataset.containsGroundTruth()) {
    for (auto& robot : dataset.robots()) {
      groundtruth_json[std::string(1, robot)] = serializeValues(dataset.groundTruthWithTypes(robot));
    }
    output_json["groundtruth"] = groundtruth_json;
  }

  // Serialize Initialization if it exists
  json initialization_json;
  if (dataset.containsInitialization()) {
    for (auto& robot : dataset.robots()) {
      initialization_json[std::string(1, robot)] = serializeValues(dataset.initializationWithTypes(robot));
    }
    output_json["initialization"] = initialization_json;
  }

  // Write the file
  std::ofstream output_stream(output_file_name);
  output_stream << std::setw(4) << output_json;
}

}  // namespace jrl