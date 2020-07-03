// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_PYTHON_BINDINGS_RUNTIME_HPP_
#define PYTHON_PYTHON_BINDINGS_RUNTIME_HPP_
#include "bark/geometry/geometry.hpp"
#include "bark/python_wrapper/common.hpp"
#include "bark/runtime/runtime.hpp"

namespace py = pybind11;

using bark::runtime::EvalRuntime;
using bark::runtime::Runtime;
using bark::runtime::RuntimePtr;

class PyRuntime : public Runtime {
 public:
  using Runtime::Runtime;
  void Step() override { PYBIND11_OVERLOAD_PURE(void, Runtime, step); }
};

void python_runtime(py::module m) {
  py::class_<Runtime, PyRuntime, RuntimePtr>(m, "PyRuntime")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("step", py::overload_cast<>(&Runtime::Step))
      .def("step", py::overload_cast<int>(&Runtime::Step))
      .def("step", py::overload_cast<float>(&Runtime::Step))
      .def("step", py::overload_cast<double>(&Runtime::Step))
      .def("step", py::overload_cast<
                       Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>(
                       &Runtime::Step));

  m.def("eval_runtime", py::overload_cast<Runtime, int>(&EvalRuntime));
  m.def("eval_runtime",
        py::overload_cast<Runtime,
                          Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>(
            &EvalRuntime));
}

#endif  // PYTHON_PYTHON_BINDINGS_RUNTIME_HPP_
