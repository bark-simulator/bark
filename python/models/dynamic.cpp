// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "dynamic.hpp"
#include "modules/models/dynamic/single_track.hpp"

namespace py = pybind11;
using namespace modules::models::dynamic;

void python_dynamic(py::module m) {
  py::class_<DynamicModel, PyDynamicModel, DynamicModelPtr>(m, "DynamicModel")
      .def(py::init<>())
      .def("stateSpaceModel", &DynamicModel::StateSpaceModel);

  py::class_<SingleTrackModel,
             DynamicModel,
             std::shared_ptr<SingleTrackModel>>(m, "SingleTrackModel")
      .def(py::init<>())
      .def("__repr__", [](const SingleTrackModel &m) {
        return "bark.dynamic.SingleTrackModel";
      })
      .def(py::pickle(
        [](const SingleTrackModel &m) -> std::string { 
            return "SingleTrackModel"; // 0
        },
        [](std::string s) { // __setstate__
            if (s != "SingleTrackModel")
                throw std::runtime_error("Invalid dynamic modelstate!");

            /* Create a new C++ instance */
            return new SingleTrackModel();
      }));

  py::enum_<StateDefinition>(m, "StateDefinition", py::arithmetic())
      .value("TIME_POSITION", TIME_POSITION)
      .value("X_POSITION", X_POSITION)
      .value("Y_POSITION", Y_POSITION)
      .value("THETA_POSITION", THETA_POSITION)
      .value("MIN_STATE_SIZE", MIN_STATE_SIZE)
      .value("Z_POSITION", Z_POSITION)
      .export_values();
}
