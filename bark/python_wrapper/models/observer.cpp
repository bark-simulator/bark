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

  py::class_<ObserverModelNone, ObserverModel,
             shared_ptr<ObserverModelNone>>(m, "ObserverNone")
    .def(py::init<const ParamsPtr&>())
    .def("__repr__", [](const ObserverModelNone& m) {
          return "bark.models.observer.ObserverNone";
    })
    .def(py::pickle([](const ObserverModelNone& b) {
        return py::make_tuple(ParamsToPython(b.GetParams()));
      },
      [](py::tuple t) {
        if (t.size() != 1)
          throw std::runtime_error("Invalid observer model state!");
        /* Create a new C++ instance */
        return new ObserverModelNone(
            PythonToParams(t[0].cast<py::tuple>()));
    }));

}
