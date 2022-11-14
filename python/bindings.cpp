#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#include "jrl/Dataset.h"

namespace py = pybind11;

PYBIND11_MODULE(jrl, m) {
  py::class_<Entry>(m, "Entry").def(py::init<uint16_t &, std::string &, gtsam::NonlinearFactor::shared_ptr &>());

  py::class_<Dataset>(m, "Dataset")
      .def(py::init<const std::string &, std::vector<char> &, std::map<char, std::vector<Entry>>,
                    boost::optional<std::map<char, gtsam::Values>> &,
                    boost::optional<std::map<char, gtsam::Values>> &>())
      .def("robots", &Dataset::robots)
      .def("groundTruth", &Dataset::groundTruth)
      .def("initialization", &Dataset::initialization)
      .def("measurements", &Dataset::measurements);
}
