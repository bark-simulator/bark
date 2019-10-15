// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <string>
#include "dynamic.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/dynamic/triple_integrator.hpp"
#include "modules/commons/params/default_params.hpp"

namespace py = pybind11;
using namespace modules::models::dynamic;
using namespace modules::commons;

void python_dynamic(py::module m) {
  py::class_<DynamicModel, PyDynamicModel, DynamicModelPtr>(m, "DynamicModel")
      .def(py::init<Params*>())
      .def("stateSpaceModel", &DynamicModel::StateSpaceModel);

  py::class_<SingleTrackModel,
             DynamicModel,
             std::shared_ptr<SingleTrackModel>>(m, "SingleTrackModel")
      .def(py::init<Params*>())
      .def("__repr__", [](const SingleTrackModel &m) {
        return "bark.dynamic.SingleTrackModel";
      })
      .def(py::pickle(
        [](const SingleTrackModel &m) -> std::string {
            return "SingleTrackModel";  // 0
        },
        [](std::string s) {  // __setstate__
            if (s != "SingleTrackModel")
                throw std::runtime_error("Invalid dynamic modelstate!");
            // param pointer must be set via python
            return new SingleTrackModel(new DefaultParams());
      }));

  py::class_<TripleIntegratorModel,
             DynamicModel,
             std::shared_ptr<TripleIntegratorModel>>(m, "TripleIntegratorModel")
      .def(py::init<Params*>())
      .def("__repr__", [](const TripleIntegratorModel &m) {
        return "bark.dynamic.TripleIntegratorModel";
      })
      .def(py::pickle(
        [](const TripleIntegratorModel &m) -> std::string {
            return "TripleIntegratorModel";  // 0
        },
        [](std::string s) {  // __setstate__
            if (s != "TripleIntegratorModel")
                throw std::runtime_error("Invalid dynamic modelstate!");
            // param pointer must be set via python
            return new TripleIntegratorModel(new DefaultParams());
      }));

  py::enum_<StateDefinition>(m, "StateDefinition", py::arithmetic())
      .value("TIME_POSITION", TIME_POSITION)
      .value("X_POSITION", X_POSITION)
      .value("Y_POSITION", Y_POSITION)
      .value("THETA_POSITION", THETA_POSITION)
      .value("VEL_POSITION", VEL_POSITION)
      .value("MIN_STATE_SIZE", MIN_STATE_SIZE)
      .value("Z_POSITION", Z_POSITION)
      .export_values();
}
