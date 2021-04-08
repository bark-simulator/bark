// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "dynamic.hpp"
#include <string>
#include "bark/commons/params/setter_params.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/dynamic/triple_integrator.hpp"

#include "bark/python_wrapper/polymorphic_conversion.hpp"

namespace py = pybind11;
using namespace bark::models::dynamic;
using namespace bark::commons;

void python_dynamic(py::module m) {
  py::class_<DynamicModel, PyDynamicModel, DynamicModelPtr>(m, "DynamicModel")
      .def(py::init<ParamsPtr>())
      .def("stateSpaceModel", &DynamicModel::StateSpaceModel);

  py::class_<SingleTrackModel, DynamicModel, std::shared_ptr<SingleTrackModel>>(
      m, "SingleTrackModel")
      .def("GetWheelBase", &SingleTrackModel::GetWheelBase)
      .def(py::init<ParamsPtr>())
      .def("__repr__",
           [](const SingleTrackModel& m) {
             return "bark.dynamic.SingleTrackModel";
           })
      .def(py::pickle(
          [](const SingleTrackModel& m) {
            return py::make_tuple(ParamsToPython(m.GetParams()));
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid single track model state!");
            // param pointer must be set via python
            /* Create a new C++ instance */
            return new SingleTrackModel(
                PythonToParams(t[0].cast<py::tuple>()));
          }));

  py::class_<TripleIntegratorModel, DynamicModel,
             std::shared_ptr<TripleIntegratorModel>>(m, "TripleIntegratorModel")
      .def(py::init<ParamsPtr>())
      .def("__repr__",
           [](const TripleIntegratorModel& m) {
             return "bark.dynamic.TripleIntegratorModel";
           })
      .def(py::pickle(
          [](const TripleIntegratorModel& m) -> std::string {
            return "TripleIntegratorModel";  // 0
          },
          [](std::string s) {  // __setstate__
            if (s != "TripleIntegratorModel")
              throw std::runtime_error("Invalid dynamic modelstate!");
            return new TripleIntegratorModel(std::make_shared<SetterParams>());
          }));


  py::class_<AccelerationLimits, std::shared_ptr<AccelerationLimits>>(m, "AccelerationLimits")
      .def(py::init<>())
      .def_readwrite("lat_acc_max", &AccelerationLimits::lat_acc_max)
      .def_readwrite("lat_acc_min", &AccelerationLimits::lat_acc_min)
      .def_readwrite("lon_acc_max", &AccelerationLimits::lon_acc_max)
      .def_readwrite("lon_acc_min", &AccelerationLimits::lon_acc_min)
      .def("__repr__",
           [](const AccelerationLimits& b) { return "bark.dynamic.AccelerationLimits"; })
      .def(py::pickle(
          [](const AccelerationLimits& a) {
            return py::make_tuple(a.lat_acc_max, a.lat_acc_min, a.lon_acc_max, a.lon_acc_min);
          },
          [](py::tuple t) {
            if (t.size() != 4)
              throw std::runtime_error("Invalid AccelerationLimits!");
            /* Create a new C++ instance */
            return new AccelerationLimits{t[0].cast<double>(),
                                    t[1].cast<double>(),
                                    t[2].cast<double>(),
                                    t[3].cast<double>()};
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

