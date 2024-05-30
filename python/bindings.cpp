#include <gtsam/nonlinear/NonlinearFactor.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "jrl/Dataset.h"
#include "jrl/DatasetBuilder.h"
#include "jrl/IOMeasurements.h"
#include "jrl/IOValues.h"
#include "jrl/Initialization.h"
#include "jrl/Metrics.h"
#include "jrl/Parser.h"
#include "jrl/Results.h"
#include "jrl/Writer.h"
#include "typecasters.h"

namespace py = pybind11;
using namespace jrl;

PYBIND11_MODULE(jrl_python, m) {
  // Import gtsam to ensure that python has access to return types
  py::module gtsam = py::module::import("gtsam");

  m.attr("Rot2Tag") = py::str(Rot2Tag);
  m.attr("Pose2Tag") = py::str(Pose2Tag);
  m.attr("Rot3Tag") = py::str(Rot3Tag);
  m.attr("Pose3Tag") = py::str(Pose3Tag);
  m.attr("Point2Tag") = py::str(Point2Tag);
  m.attr("Point3Tag") = py::str(Point3Tag);
  m.attr("Unit3Tag") = py::str(Unit3Tag);
  m.attr("VectorTag") = py::str(VectorTag);
  m.attr("ScalarTag") = py::str(ScalarTag);
  m.attr("BearingRangeTag") = py::str(BearingRangeTag);

  m.attr("PriorFactorPose2Tag") = py::str(PriorFactorPose2Tag);
  m.attr("PriorFactorPose3Tag") = py::str(PriorFactorPose3Tag);
  m.attr("BetweenFactorPose2Tag") = py::str(BetweenFactorPose2Tag);
  m.attr("BetweenFactorPose3Tag") = py::str(BetweenFactorPose3Tag);
  m.attr("RangeFactorPose2Tag") = py::str(RangeFactorPose2Tag);
  m.attr("RangeFactorPose3Tag") = py::str(RangeFactorPose3Tag);
  m.attr("RangeFactor2DTag") = py::str(RangeFactor2DTag);
  m.attr("RangeFactor3DTag") = py::str(RangeFactor3DTag);
  m.attr("BearingRangeFactorPose2Tag") = py::str(BearingRangeFactorPose2Tag);
  m.attr("BearingRangeFactorPose3Tag") = py::str(BearingRangeFactorPose3Tag);
  m.attr("BearingRangeFactor2DTag") = py::str(BearingRangeFactor2DTag);
  m.attr("BearingRangeFactor3DTag") = py::str(BearingRangeFactor3DTag);
  m.attr("PriorFactorPoint2Tag") = py::str(PriorFactorPoint2Tag);
  m.attr("PriorFactorPoint3Tag") = py::str(PriorFactorPoint3Tag);
  m.attr("BetweenFactorPoint2Tag") = py::str(BetweenFactorPoint2Tag);
  m.attr("BetweenFactorPoint3Tag") = py::str(BetweenFactorPoint3Tag);

  /**
   * ########     ###    ########    ###     ######  ######## ########
   * ##     ##   ## ##      ##      ## ##   ##    ## ##          ##
   * ##     ##  ##   ##     ##     ##   ##  ##       ##          ##
   * ##     ## ##     ##    ##    ##     ##  ######  ######      ##
   * ##     ## #########    ##    #########       ## ##          ##
   * ##     ## ##     ##    ##    ##     ## ##    ## ##          ##
   * ########  ##     ##    ##    ##     ##  ######  ########    ##
   */
  /**********************************************************************************************************************/
  py::class_<Entry>(m, "Entry")
      .def(py::init<uint64_t &, std::vector<std::string> &, gtsam::NonlinearFactorGraph &,
                    std::map<gtsam::FactorIndex, bool> &>(),
           py::arg("stamp"), py::arg("measurement_types"), py::arg("measurements"),
           py::arg("potential_outlier_statuses") = std::map<gtsam::FactorIndex, bool>())
      .def_readwrite("stamp", &Entry::stamp)
      .def_readwrite("measurement_types", &Entry::measurement_types)
      .def_readwrite("measurements", &Entry::measurements)
      .def_readwrite("potential_outlier_statuses", &Entry::potential_outlier_statuses)
      .def("remove", &Entry::remove)
      .def("filter", &Entry::filter)
      .def(py::pickle(
          [](const Entry &entry) {  // __getstate__
            return py::make_tuple(entry.stamp, entry.measurement_types, entry.measurements,
                                  entry.potential_outlier_statuses);
          },
          [](py::tuple tup) {  // __setstate__
            Entry entry(tup[0].cast<uint64_t>(), tup[1].cast<std::vector<std::string>>(),
                        tup[2].cast<gtsam::NonlinearFactorGraph>(), tup[3].cast<std::map<gtsam::FactorIndex, bool>>());
            return entry;
          }));

  /**********************************************************************************************************************/
  py::class_<TypedValues>(m, "TypedValues")
      .def(py::init<gtsam::Values &, ValueTypes &>())
      .def_readwrite("values", &TypedValues::values)
      .def_readwrite("types", &TypedValues::types)
      .def(py::pickle(
          [](const TypedValues &typed_values) {  // __getstate__
            return py::make_tuple(typed_values.values, typed_values.types);
          },
          [](py::tuple tup) {  // __setstate__
            TypedValues tv(tup[0].cast<gtsam::Values>(), tup[1].cast<ValueTypes>());
            return tv;
          }));

  /**********************************************************************************************************************/
  py::class_<Dataset>(m, "Dataset")
      .def(py::init<const std::string &, std::vector<char> &, std::map<char, std::vector<Entry>>,
                    boost::optional<std::map<char, TypedValues>> &, boost::optional<std::map<char, TypedValues>> &>())
      .def("name", &Dataset::name)
      .def("robots", &Dataset::robots)
      .def("groundTruth", &Dataset::groundTruth)
      .def("groundTruthWithTypes", &Dataset::groundTruthWithTypes)
      .def("containsGroundTruth", &Dataset::containsGroundTruth)
      .def("initialization", &Dataset::initialization)
      .def("initializationWithTypes", &Dataset::initializationWithTypes)
      .def("containsInitialization", &Dataset::containsInitialization)
      .def("measurements", &Dataset::measurements)
      .def("factorGraph", &Dataset::factorGraph)
      .def(py::pickle(
          [](const Dataset &dataset) {  // __getstate__
            std::map<char, std::vector<Entry>> measurements;
            boost::optional<std::map<char, TypedValues>> ground_truth = boost::none;
            boost::optional<std::map<char, TypedValues>> initialization = boost::none;

            if (dataset.containsGroundTruth()) {
              ground_truth = std::map<char, TypedValues>();
            }

            if (dataset.containsInitialization()) {
              initialization = std::map<char, TypedValues>();
            }

            for (auto &rid : dataset.robots()) {
              measurements[rid] = dataset.measurements(rid);
              if (ground_truth) (*ground_truth)[rid] = dataset.groundTruthWithTypes(rid);
              if (initialization) (*initialization)[rid] = dataset.initializationWithTypes(rid);
            }

            return py::make_tuple(dataset.name(), dataset.robots(), measurements, ground_truth, initialization);
          },
          [](py::tuple tup) {  // __setstate__
            Dataset dataset(tup[0].cast<std::string>(), tup[1].cast<std::vector<char>>(),
                            tup[2].cast<std::map<char, std::vector<Entry>>>(),
                            tup[3].cast<boost::optional<std::map<char, TypedValues>>>(),
                            tup[4].cast<boost::optional<std::map<char, TypedValues>>>());
            return dataset;
          }));

  /**********************************************************************************************************************/
  py::class_<DatasetBuilder>(m, "DatasetBuilder")
      .def(py::init<const std::string &, std::vector<char> &>())
      .def("addEntry", &DatasetBuilder::addEntry, py::arg("robot"), py::arg("stamp"), py::arg("measurements"),
           py::arg("measurement_types"), py::arg("potential_outlier_statuses") = std::map<gtsam::FactorIndex, bool>(),
           py::arg("initialization") = py::none(), py::arg("groundtruth") = py::none())
      .def("addGroundTruth", &DatasetBuilder::addGroundTruth, py::arg("robot"), py::arg("groundtruth"))
      .def("addInitialization", &DatasetBuilder::addInitialization, py::arg("robot"), py::arg("initialization"))
      .def("build", &DatasetBuilder::build);

  /**
   * ########  ########  ######  ##     ## ##       ########  ######
   * ##     ## ##       ##    ## ##     ## ##          ##    ##    ##
   * ##     ## ##       ##       ##     ## ##          ##    ##
   * ########  ######    ######  ##     ## ##          ##     ######
   * ##   ##   ##             ## ##     ## ##          ##          ##
   * ##    ##  ##       ##    ## ##     ## ##          ##    ##    ##
   * ##     ## ########  ######   #######  ########    ##     ######
   */

  /**********************************************************************************************************************/
  py::class_<Results>(m, "Results")
      .def(py::init<const std::string &, const std::string &, std::vector<char> &, std::map<char, TypedValues> &>())
      .def_readwrite("dataset_name", &Results::dataset_name)
      .def_readwrite("method_name", &Results::method_name)
      .def_readwrite("robots", &Results::robots)
      .def_readwrite("robot_solutions", &Results::robot_solutions)
      .def(py::pickle(
          [](const Results &results) {  // __getstate__
            return py::make_tuple(results.dataset_name, results.method_name, results.robots, results.robot_solutions);
          },
          [](py::tuple tup) {  // __setstate__
            Results result(tup[0].cast<std::string>(), tup[1].cast<std::string>(), tup[2].cast<std::vector<char>>(),
                           tup[3].cast<std::map<char, TypedValues>>());
            return result;
          }));

  /**
   * ####  #######
   *  ##  ##     ##
   *  ##  ##     ##
   *  ##  ##     ##
   *  ##  ##     ##
   *  ##  ##     ##
   * ####  #######
   */

  /**********************************************************************************************************************/
  py::class_<Parser>(m, "Parser")
      .def(py::init<>())
      .def("parseDataset", &Parser::parseDataset)
      .def("parseResults", &Parser::parseResults)
      .def("parseMetricSummary", &Parser::parseMetricSummary);

  /**********************************************************************************************************************/
  py::class_<Writer>(m, "Writer")
      .def(py::init<>())
      .def("writeDataset", &Writer::writeDataset)
      .def("writeResults", &Writer::writeResults)
      .def("writeMetricSummary", &Writer::writeMetricSummary);

  /**
   * ##     ## ######## ######## ########  ####  ######   ######
   * ###   ### ##          ##    ##     ##  ##  ##    ## ##    ##
   * #### #### ##          ##    ##     ##  ##  ##       ##
   * ## ### ## ######      ##    ########   ##  ##        ######
   * ##     ## ##          ##    ##   ##    ##  ##             ##
   * ##     ## ##          ##    ##    ##   ##  ##    ## ##    ##
   * ##     ## ########    ##    ##     ## ####  ######   ######
   */

  /**********************************************************************************************************************/
  py::class_<MetricSummary>(m, "MetricSummary")
      .def(py::init<>())
      .def_readwrite("robots", &MetricSummary::robots)
      .def_readwrite("dataset_name", &MetricSummary::dataset_name)
      .def_readwrite("method_name", &MetricSummary::method_name)
      .def_readwrite("robot_ate", &MetricSummary::robot_ate)
      .def_readwrite("total_ate", &MetricSummary::total_ate)
      .def_readwrite("sve", &MetricSummary::sve)
      .def_readwrite("mean_residual", &MetricSummary::mean_residual);

  /**********************************************************************************************************************/
  m.def("computeMetricSummaryPoint2", &metrics::computeMetricSummary<gtsam::Point2>, py::return_value_policy::copy,
        py::arg("dataset"), py::arg("results"), py::arg("align") = true, py::arg("align_with_scale") = false,
        py::arg("step_idx") = std::nullopt);
  m.def("computeMetricSummaryPoint3", &metrics::computeMetricSummary<gtsam::Point3>, py::return_value_policy::copy,
        py::arg("dataset"), py::arg("results"), py::arg("align") = true, py::arg("align_with_scale") = false,
        py::arg("step_idx") = std::nullopt);
  m.def("computeMetricSummaryPose2", &metrics::computeMetricSummary<gtsam::Pose2>, py::return_value_policy::copy,
        py::arg("dataset"), py::arg("results"), py::arg("align") = true, py::arg("align_with_scale") = false,
        py::arg("step_idx") = std::nullopt);
  m.def("computeMetricSummaryPose3", &metrics::computeMetricSummary<gtsam::Pose3>, py::return_value_policy::copy,
        py::arg("dataset"), py::arg("results"), py::arg("align") = true, py::arg("align_with_scale") = false,
        py::arg("step_idx") = std::nullopt);

  /**********************************************************************************************************************/
  m.def("computeMeanResidual", &metrics::computeMeanResidual, py::return_value_policy::copy, py::arg("dataset"),
        py::arg("results"), py::arg("step_idx") = std::nullopt);

  /**********************************************************************************************************************/
  m.def("computeSVEPoint2", &metrics::computeSVE<gtsam::Point2>, py::return_value_policy::copy, py::arg("results"));
  m.def("computeSVEPoint3", &metrics::computeSVE<gtsam::Point3>, py::return_value_policy::copy, py::arg("results"));
  m.def("computeSVEPose2", &metrics::computeSVE<gtsam::Pose2>, py::return_value_policy::copy, py::arg("results"));
  m.def("computeSVEPose3", &metrics::computeSVE<gtsam::Pose3>, py::return_value_policy::copy, py::arg("results"));

  /**********************************************************************************************************************/
  m.def("computeATEPoint2", &metrics::computeATE<gtsam::Point2>, py::return_value_policy::copy, py::arg("rid"),
        py::arg("dataset"), py::arg("results"), py::arg("align") = true, py::arg("align_with_scale") = false,
        py::arg("allow_partial_results") = false);
  m.def("computeATEPoint3", &metrics::computeATE<gtsam::Point3>, py::return_value_policy::copy, py::arg("rid"),
        py::arg("dataset"), py::arg("results"), py::arg("align") = true, py::arg("align_with_scale") = false,
        py::arg("allow_partial_results") = false);
  m.def("computeATEPose2", &metrics::computeATE<gtsam::Pose2>, py::return_value_policy::copy, py::arg("rid"),
        py::arg("dataset"), py::arg("results"), py::arg("align") = true, py::arg("align_with_scale") = false,
        py::arg("allow_partial_results") = false);
  m.def("computeATEPose3", &metrics::computeATE<gtsam::Pose3>, py::return_value_policy::copy, py::arg("rid"),
        py::arg("dataset"), py::arg("results"), py::arg("align") = true, py::arg("align_with_scale") = false,
        py::arg("allow_partial_results") = false);

  /**
   * #### ##    ## #### ######## ####    ###    ##       #### ########    ###    ######## ####  #######  ##    ##
   *  ##  ###   ##  ##     ##     ##    ## ##   ##        ##       ##    ## ##      ##     ##  ##     ## ###   ##
   *  ##  ####  ##  ##     ##     ##   ##   ##  ##        ##      ##    ##   ##     ##     ##  ##     ## ####  ##
   *  ##  ## ## ##  ##     ##     ##  ##     ## ##        ##     ##    ##     ##    ##     ##  ##     ## ## ## ##
   *  ##  ##  ####  ##     ##     ##  ######### ##        ##    ##     #########    ##     ##  ##     ## ##  ####
   *  ##  ##   ###  ##     ##     ##  ##     ## ##        ##   ##      ##     ##    ##     ##  ##     ## ##   ###
   * #### ##    ## ####    ##    #### ##     ## ######## #### ######## ##     ##    ##    ####  #######  ##    ##
   */
  py::class_<Initializer>(m, "Initializer").def(py::init<>()).def("initialization", &Initializer::initialization);
}
