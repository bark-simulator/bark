// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Observer.hpp"
#include <string>

namespace py = pybind11;
using namespace bark::commons;
using std::shared_ptr;

void python_observer(py::module m) {
  py::class_<ObserverModel, PyObserverModel, ObserverModelPtr>(
      m, "ObserverModel")
      .def(py::init<const ParamsPtr&>())
      .def("Observe", &ObserverModel::Observe);
}
