// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "observer.hpp"
#include "bark/models/observer/observer_model_none.hpp"
#include "bark/models/observer/observer_model_parametric.hpp"
#include <string>

namespace py = pybind11;
using namespace bark::commons;
using std::shared_ptr;

void python_observer(py::module m) {
  py::class_<ObserverModel, PyObserverModel, ObserverModelPtr>(
    m, "ObserverModel")
    .def(py::init<const bark::commons::ParamsPtr&>())
    .def("Observe", &ObserverModel::Observe)
    .def_property("observe_only_for_agents", &ObserverModel::GetObserveOnlyForAgents,
                               &ObserverModel::SetObserveOnlyForAgents);

  py::class_<ObserverModelNone, ObserverModel,
             shared_ptr<ObserverModelNone>>(m, "ObserverModelNone")
    .def(py::init<const bark::commons::ParamsPtr&>())
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

  py::class_<ObserverModelParametric, ObserverModel,
             shared_ptr<ObserverModelParametric>>(m, "ObserverModelParametric")
    .def(py::init<const bark::commons::ParamsPtr&>())
    .def("__repr__", [](const ObserverModelParametric& m) {
          return "bark.models.observer.ObserverModelParametric";
    })
    .def(py::pickle([](const ObserverModelParametric& b) {
        return py::make_tuple(ParamsToPython(b.GetParams()));
      },
      [](py::tuple t) {
        if (t.size() != 1)
          throw std::runtime_error("Invalid observer model state!");
        /* Create a new C++ instance */
        return new ObserverModelParametric(
            PythonToParams(t[0].cast<py::tuple>()));
    }));

}
