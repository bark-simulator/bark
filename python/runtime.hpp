// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_PYTHON_BINDINGS_RUNTIME_HPP_
#define PYTHON_PYTHON_BINDINGS_RUNTIME_HPP_
#include "python/common.hpp"
#include "modules/runtime/runtime.hpp"

namespace py = pybind11;

using modules::runtime::Runtime;
using modules::runtime::EvalRuntime;
using modules::runtime::RuntimePtr;

class PyRuntime : public Runtime {
 public:
  using Runtime::Runtime;
  void Step() override {
    PYBIND11_OVERLOAD_PURE(
        void,
        Runtime,
        step);
  }
};

void python_runtime(py::module m) {
  py::class_<Runtime,
             PyRuntime,
             RuntimePtr>(m, "PyRuntime")
      .def(py::init<modules::commons::Params*>())
      .def("step", py::overload_cast<>(&Runtime::Step))
      .def("step", py::overload_cast<int>(&Runtime::Step))
      .def("step", py::overload_cast<float>(&Runtime::Step))
      .def("step", py::overload_cast<double>(&Runtime::Step));
  
  m.def("eval_runtime", &EvalRuntime, "Check wether binding is correct.");
}

#endif  // PYTHON_PYTHON_BINDINGS_RUNTIME_HPP_
