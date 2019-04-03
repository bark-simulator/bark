// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "behavior.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"

namespace py = pybind11;
using modules::models::behavior::BehaviorModel;
using modules::models::behavior::BehaviorModelPtr;
using modules::models::behavior::BehaviorConstantVelocity;
using std::shared_ptr;
void python_behavior(py::module m) {
  py::class_<BehaviorModel,
             PyBehaviorModel,
             BehaviorModelPtr>(m, "BehaviorModel")
      .def(py::init<modules::commons::Params *>())
      .def("plan", &BehaviorModel::Plan)
      .def("clone", &BehaviorModel::Clone)
      .def_property("last_trajectory",
                    &BehaviorModel::get_last_trajectory,
                    &BehaviorModel::set_last_trajectory);

  py::class_<BehaviorConstantVelocity,
             BehaviorModel,
             shared_ptr<BehaviorConstantVelocity>>(m,
                                                   "BehaviorConstantVelocity")
      .def(py::init<modules::commons::Params *>())
      .def("__repr__", [](const BehaviorConstantVelocity &m) {
        return "bark.behavior.BehaviorConstantVelocity";
      });
}