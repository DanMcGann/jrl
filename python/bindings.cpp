#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "jrl/Dataset.h"
#include "typecasters.h"

namespace py = pybind11;
using namespace jrl;



PYBIND11_MODULE(jrl_python, m) {
  py::class_<Entry>(m, "Entry").def(py::init<uint64_t &, std::vector<std::string>&, gtsam::NonlinearFactorGraph &>())
    .def_readwrite("stamp", &Entry::stamp)
    .def_readwrite("measurement_types", &Entry::measurement_types)
    .def_readwrite("measurements", &Entry::measurements);

  py::class_<Dataset>(m, "Dataset")
      .def(py::init<const std::string &, std::vector<char> &, std::map<char, std::vector<Entry>>,
                    boost::optional<std::map<char, std::pair<gtsam::Values, ValueTypes>>> &,
                    boost::optional<std::map<char, std::pair<gtsam::Values, ValueTypes>>> &>())
      .def("robots", &Dataset::robots)
      .def("groundTruth", &Dataset::groundTruth)
      .def("initialization", &Dataset::initialization)
      .def("measurements", &Dataset::measurements);
}
