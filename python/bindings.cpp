#include <gtsam/nonlinear/NonlinearFactor.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "jrl/Dataset.h"
#include "jrl/DatasetBuilder.h"
#include "jrl/IOMeasurements.h"
#include "jrl/IOValues.h"
#include "jrl/Parser.h"
#include "jrl/Results.h"
#include "jrl/Writer.h"
#include "typecasters.h"

namespace py = pybind11;
using namespace jrl;

PYBIND11_MODULE(jrl_python, m) {
  // Import gtsam to ensure that python has access to return types
  py::module gtsam = py::module::import("gtsam");

  m.attr("Pose2Tag") = py::str(Pose2Tag);
  m.attr("Pose3Tag") = py::str(Pose3Tag);
  m.attr("Point2Tag") = py::str(Point2Tag);
  m.attr("Point3Tag") = py::str(Point3Tag);
  m.attr("VectorTag") = py::str(VectorTag);
  m.attr("ScalarTag") = py::str(ScalarTag);

  m.attr("PriorFactorPose2Tag") = py::str(PriorFactorPose2Tag);
  m.attr("PriorFactorPose3Tag") = py::str(PriorFactorPose3Tag);
  m.attr("BetweenFactorPose2Tag") = py::str(BetweenFactorPose2Tag);
  m.attr("BetweenFactorPose3Tag") = py::str(BetweenFactorPose3Tag);
  m.attr("RangeFactorPose2Tag") = py::str(RangeFactorPose2Tag);
  m.attr("RangeFactorPose3Tag") = py::str(RangeFactorPose3Tag);
  m.attr("PriorFactorPoint2Tag") = py::str(PriorFactorPoint2Tag);
  m.attr("PriorFactorPointTag") = py::str(PriorFactorPoint3Tag);
  m.attr("BetweenFactorPoint2Tag") = py::str(BetweenFactorPoint2Tag);
  m.attr("BetweenFactorPoint3Tag") = py::str(BetweenFactorPoint3Tag);

  /**********************************************************************************************************************/
  py::class_<Entry>(m, "Entry")
      .def(py::init<uint64_t &, std::vector<std::string> &, gtsam::NonlinearFactorGraph &>())
      .def_readwrite("stamp", &Entry::stamp)
      .def_readwrite("measurement_types", &Entry::measurement_types)
      .def_readwrite("measurements", &Entry::measurements);

  /**********************************************************************************************************************/
  py::class_<TypedValues>(m, "TypedValues")
      .def(py::init<gtsam::Values &, ValueTypes &>())
      .def_readwrite("values", &TypedValues::values)
      .def_readwrite("types", &TypedValues::types);

  /**********************************************************************************************************************/
  py::class_<Dataset>(m, "Dataset")
      .def(py::init<const std::string &, std::vector<char> &, std::map<char, std::vector<Entry>>,
                    boost::optional<std::map<char, TypedValues>> &, boost::optional<std::map<char, TypedValues>> &>())
      .def("name", &Dataset::name)
      .def("robots", &Dataset::robots)
      .def("groundTruth", &Dataset::groundTruth)
      .def("containsGroundTruth", &Dataset::containsGroundTruth)
      .def("initialization", &Dataset::initialization)
      .def("containsInitialization", &Dataset::containsInitialization)
      .def("measurements", &Dataset::measurements);

  /**********************************************************************************************************************/
  py::class_<Results>(m, "Results")
      .def(py::init<const std::string &, const std::string &, std::vector<char> &, std::map<char, TypedValues> &>())
      .def_readwrite("dataset_name", &Results::dataset_name)
      .def_readwrite("method_name", &Results::method_name)
      .def_readwrite("robots", &Results::robots)
      .def_readwrite("robot_solutions", &Results::robot_solutions);

  /**********************************************************************************************************************/
  py::class_<DatasetBuilder>(m, "DatasetBuilder")
      .def(py::init<const std::string &, std::vector<char> &>())
      .def("addEntry", &DatasetBuilder::addEntry, py::arg("robot"), py::arg("stamp"), py::arg("measurements"),
           py::arg("measurement_types"), py::arg("initialization") = py::none(), py::arg("groundtruth") = py::none())
      .def("build", &DatasetBuilder::build);

  /**********************************************************************************************************************/
  py::class_<Parser>(m, "Parser")
      .def(py::init<>())
      .def("parseDataset", &Parser::parseDataset)
      .def("parseResults", &Parser::parseResults);

  /**********************************************************************************************************************/
  py::class_<Writer>(m, "Writer")
      .def(py::init<>())
      .def("writeDataset", &Writer::writeDataset)
      .def("writeResults", &Writer::writeResults);
}
