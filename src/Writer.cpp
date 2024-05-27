#include "jrl/Writer.h"

#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <fstream>
#include <iomanip>
#include <iostream>

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
    {Pose2Tag,  [](gtsam::Key key, gtsam::Values& vals) { return io_values::serialize<gtsam::Pose2>(vals.at<gtsam::Pose2>(key)); }},
    {Pose3Tag,  [](gtsam::Key key, gtsam::Values& vals) { return io_values::serialize<gtsam::Pose3>(vals.at<gtsam::Pose3>(key)); }},
    {Point2Tag, [](gtsam::Key key, gtsam::Values& vals) { return io_values::serialize<gtsam::Point2>(vals.at<gtsam::Point2>(key)); }},
    {Point3Tag, [](gtsam::Key key, gtsam::Values& vals) { return io_values::serialize<gtsam::Point3>(vals.at<gtsam::Point3>(key)); }},
    {VectorTag, [](gtsam::Key key, gtsam::Values& vals) { return io_values::serialize<gtsam::Vector>(vals.at<gtsam::Vector>(key)); }},
    {ScalarTag, [](gtsam::Key key, gtsam::Values& vals) { return io_values::serialize<double>(vals.at<double>(key)); }},
  };
  // clang-format on
  return serializer_functions;
}

/**********************************************************************************************************************/
std::map<std::string, MeasurementSerializer> Writer::loadDefaultMeasurementSerializers() {
  // clang-format off
  std::map<std::string, MeasurementSerializer> serializer_functions = {
    {PriorFactorPose2Tag,           [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializePrior<gtsam::Pose2>(&serialize<gtsam::Pose2>, PriorFactorPose2Tag, factor); }},
    {PriorFactorPose3Tag,           [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializePrior<gtsam::Pose3>(&serialize<gtsam::Pose3>, PriorFactorPose3Tag, factor); }},
    {BetweenFactorPose2Tag,         [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<gtsam::Pose2, gtsam::BetweenFactor<gtsam::Pose2>>(&serialize<gtsam::Pose2>, BetweenFactorPose2Tag, factor); }},
    {BetweenFactorPose3Tag,         [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<gtsam::Pose3, gtsam::BetweenFactor<gtsam::Pose3>>(&serialize<gtsam::Pose3>, BetweenFactorPose3Tag, factor); }},
    {RangeFactorPose2Tag,           [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<double, gtsam::RangeFactor<gtsam::Pose2>>(&serialize<double>, RangeFactorPose2Tag, factor); }},
    {RangeFactorPose3Tag,           [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<double, gtsam::RangeFactor<gtsam::Pose3>>(&serialize<double>, RangeFactorPose3Tag, factor); }},
    {RangeFactor2DTag,              [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<double, gtsam::RangeFactor<gtsam::Pose2, gtsam::Point2>>(&serialize<double>, RangeFactor2DTag, factor); }},
    {RangeFactor3DTag,              [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<double, gtsam::RangeFactor<gtsam::Pose3, gtsam::Point3>>(&serialize<double>, RangeFactor3DTag, factor); }},
    {BearingRangeFactorPose2Tag,    [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<gtsam::BearingRange<gtsam::Pose2, gtsam::Pose2>, gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Pose2>>(&serializeBearingRange<gtsam::Pose2, gtsam::Pose2>, BearingRangeFactorPose2Tag, factor); }},
    {BearingRangeFactorPose3Tag,    [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<gtsam::BearingRange<gtsam::Pose3, gtsam::Pose3>, gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Pose3>>(&serializeBearingRange<gtsam::Pose3, gtsam::Pose3>, BearingRangeFactorPose3Tag, factor); }},
    {BearingRangeFactor2DTag,       [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<gtsam::BearingRange<gtsam::Pose2, gtsam::Point2>, gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>>(&serializeBearingRange<gtsam::Pose2, gtsam::Point2>, BearingRangeFactor2DTag, factor); }},
    {BearingRangeFactor3DTag,       [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<gtsam::BearingRange<gtsam::Pose3, gtsam::Point3>, gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>>(&serializeBearingRange<gtsam::Pose3, gtsam::Point3>, BearingRangeFactor3DTag, factor); }},
    {PriorFactorPoint2Tag,          [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializePrior<gtsam::Point2>(&serialize<gtsam::Point2>, PriorFactorPoint2Tag, factor); }},
    {PriorFactorPoint3Tag,          [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializePrior<gtsam::Point3>(&serialize<gtsam::Point3>, PriorFactorPoint3Tag, factor); }},
    {BetweenFactorPoint2Tag,        [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<gtsam::Point2, gtsam::BetweenFactor<gtsam::Point2>>(&serialize<gtsam::Point2>, BetweenFactorPoint2Tag, factor); }},
    {BetweenFactorPoint3Tag,        [](gtsam::NonlinearFactor::shared_ptr& factor) { return serializeNoiseModel2<gtsam::Point3, gtsam::BetweenFactor<gtsam::Point3>>(&serialize<gtsam::Point3>, BetweenFactorPoint2Tag, factor); }},
  };
  // clang-format on

  return serializer_functions;
}

/**********************************************************************************************************************/
json Writer::serializeValues(TypedValues typed_values) const {
  gtsam::Values values = typed_values.values;
  ValueTypes types = typed_values.types;
  json output;
  for (auto& ktpair : types) {
    gtsam::Key key = ktpair.first;
    std::string type_tag = ktpair.second;
    json value_obj = value_serializers_.at(type_tag)(key, values);
    value_obj["key"] = key;
    output.push_back(value_obj);
  }
  return output;
}

/**********************************************************************************************************************/
json Writer::serializeMeasurements(std::vector<Entry> entries) const {
  json output;
  for (auto& entry : entries) {
    json entry_obj;
    entry_obj["stamp"] = entry.stamp;
    size_t numFactors = entry.measurements.nrFactors();
    for (int i = 0; i < numFactors; i++) {
      std::string measurement_type = entry.measurement_types[i];
      gtsam::NonlinearFactor::shared_ptr factor = entry.measurements.at(i);
      entry_obj["measurements"].push_back(measurement_serializers_.at(measurement_type)(factor));
    }
    if (!entry.potential_outlier_statuses.empty()) {
      entry_obj["potential_outlier_statuses"] = entry.potential_outlier_statuses;
    }
    output.push_back(entry_obj);
  }
  return output;
}

/**********************************************************************************************************************/
void Writer::writeJson(json output_json, std::string output_file_name, bool compress_to_cbor) const {
  if (compress_to_cbor) {
    std::ofstream output_stream(output_file_name + ".cbor", std::ios::out | std::ios::binary);
    json::to_cbor(output_json, nlohmann::detail::output_adapter<char>{output_stream});
    output_stream.close();
  } else {
    std::ofstream output_stream(output_file_name);
    output_stream << output_json;
    output_stream.close();
  }
}

/**********************************************************************************************************************/
void Writer::writeDataset(Dataset dataset, std::string output_file_name, bool compress_to_cbor) const {
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
  writeJson(output_json, output_file_name, compress_to_cbor);
}

/**********************************************************************************************************************/
void Writer::writeResults(Results results, std::string output_file_name, bool compress_to_cbor) const {
  json output_json;

  // serialize Header information
  output_json["dataset_name"] = results.dataset_name;
  output_json["method_name"] = results.method_name;
  output_json["robots"] = results.robots;

  // Serialize solutions
  json solution_json;
  for (auto& robot : results.robots) {
    solution_json[std::string(1, robot)] = serializeValues(results.robot_solutions[robot]);
  }
  output_json["solutions"] = solution_json;
  // Write the file
  writeJson(output_json, output_file_name, compress_to_cbor);
}

/**********************************************************************************************************************/
void Writer::writeMetricSummary(MetricSummary metric_summary, std::string output_file_name,
                                bool compress_to_cbor) const {
  json output_json;

  // serialize Header information
  output_json["dataset_name"] = metric_summary.dataset_name;
  output_json["method_name"] = metric_summary.method_name;
  output_json["robots"] = metric_summary.robots;
  if (metric_summary.robot_ate) output_json["robot_ate"] = *metric_summary.robot_ate;
  if (metric_summary.total_ate) output_json["total_ate"] = *metric_summary.total_ate;
  if (metric_summary.sve) output_json["sve"] = *metric_summary.sve;
  if (metric_summary.mean_residual) output_json["mean_residual"] = *metric_summary.mean_residual;

  // Write the file
  writeJson(output_json, output_file_name, compress_to_cbor);
}

}  // namespace jrl