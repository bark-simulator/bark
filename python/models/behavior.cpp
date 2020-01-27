// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "behavior.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/models/behavior/dynamic_model/dynamic_model.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "python/models/plan/plan.hpp"

namespace py = pybind11;
using modules::models::behavior::BehaviorModel;
using modules::models::behavior::BehaviorModelPtr;
using modules::models::behavior::BehaviorConstantVelocity;
using modules::models::behavior::BehaviorMotionPrimitives;
using modules::models::behavior::DynamicBehaviorModel;
using modules::models::behavior::BehaviorIDMClassic;
using  modules::models::dynamic::DynamicModelPtr;

using std::shared_ptr;
void python_behavior(py::module m) {
  py::class_<BehaviorModel,
             PyBehaviorModel,
             BehaviorModelPtr>(m, "BehaviorModel")
    .def(py::init<modules::commons::Params *>())
    .def("Plan", &BehaviorModel::Plan)
    .def("Clone", &BehaviorModel::Clone)
    .def("SetLastTrajectory", &BehaviorModel::SetLastTrajectory)
    .def("SetLastAction", &BehaviorModel::SetLastAction)
    .def("GetLastAction", &BehaviorModel::GetLastAction)
    .def_property("last_trajectory",
                  &BehaviorModel::GetLastTrajectory,
                  &BehaviorModel::SetLastTrajectory);

  py::class_<BehaviorConstantVelocity,
             BehaviorModel,
             shared_ptr<BehaviorConstantVelocity>>(m,
                                                   "BehaviorConstantVelocity")
    .def(py::init<modules::commons::Params*>())
    .def("__repr__", [](const BehaviorConstantVelocity &m) {
      return "bark.behavior.BehaviorConstantVelocity";
    })
    .def(py::pickle(
      [](const BehaviorConstantVelocity& b) {
        return py::make_tuple(b.GetLastTrajectory());
      },
      [](py::tuple t) {
        if (t.size() != 1)
          throw std::runtime_error("Invalid behavior model state!");
        /* Create a new C++ instance */
        return new BehaviorConstantVelocity(nullptr);
      }));

  py::class_<BehaviorIDMClassic,
             BehaviorModel,
             shared_ptr<BehaviorIDMClassic>>(m, "BehaviorIDMClassic")
    .def(py::init<modules::commons::Params*>())
    .def("__repr__", [](const BehaviorIDMClassic &m) {
      return "bark.behavior.BehaviorIDMClassic";
    })
    .def(py::pickle(
      [](const BehaviorIDMClassic& b) {
        return py::make_tuple(b.GetLastTrajectory());
      },
      [](py::tuple t) {
        if (t.size() != 1)
          throw std::runtime_error("Invalid behavior model state!");
        return new BehaviorIDMClassic(nullptr);
      }));

  py::class_<BehaviorMotionPrimitives,
             BehaviorModel,
             shared_ptr<BehaviorMotionPrimitives>>(m,
    "BehaviorMotionPrimitives")
    .def(py::init<const DynamicModelPtr&, modules::commons::Params *>())
    .def("__repr__", [](const BehaviorMotionPrimitives &b) {
      return "bark.behavior.BehaviorMotionPrimitives";
    })
    .def("AddMotionPrimitive", &BehaviorMotionPrimitives::AddMotionPrimitive)
    .def("ActionToBehavior", &BehaviorMotionPrimitives::ActionToBehavior);

  py::class_<DynamicBehaviorModel,
             BehaviorModel,
             shared_ptr<DynamicBehaviorModel>>(m, "DynamicBehaviorModel")
      .def(py::init<const DynamicModelPtr&,
           modules::commons::Params *>())
      .def("__repr__", [](const DynamicBehaviorModel &b) {
        return "bark.behavior.DynamicBehaviorModel";
      });

  python_behavior_plan(m);
}