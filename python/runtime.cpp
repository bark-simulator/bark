// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "python/runtime.hpp"
namespace py = pybind11;


using std::shared_ptr;
void python_runtime(py::module m) {
  py::class_<Runtime,
             PyRuntime,
             RuntimePtr>(m, "Runtime")
      .def(py::init<modules::commons::Params *>());
}