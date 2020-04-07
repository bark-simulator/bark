// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "behavior.hpp"
#include "python/polymorphic_conversion.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/models/behavior/motion_primitives/continuous_actions.hpp"
#include "modules/models/behavior/motion_primitives/macro_actions.hpp"
#include "modules/models/behavior/dynamic_model/dynamic_model.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/behavior/idm/idm_lane_tracking.hpp"
#include "modules/models/behavior/mobil/mobil.hpp"
#include "modules/models/behavior/static_trajectory/behavior_static_trajectory.hpp"
#include "python/models/plan/plan.hpp"


namespace py = pybind11;
using namespace modules::models::behavior;
using namespace modules::models::behavior::primitives;
using modules::models::dynamic::DynamicModelPtr;

using std::shared_ptr;
void python_behavior(py::module m) {
  py::class_<BehaviorModel,
             PyBehaviorModel,
             BehaviorModelPtr>(m, "BehaviorModel")
    .def(py::init<const modules::commons::ParamsPtr&>())
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
    .def(py::init<const modules::commons::ParamsPtr&>())
    .def("__repr__", [](const BehaviorConstantVelocity &m) {
      return "bark.behavior.BehaviorConstantVelocity";
    })
    .def(py::pickle(
      [](const BehaviorConstantVelocity& b) {
        return py::make_tuple(ParamsToPython(b.GetParams()));
      },
      [](py::tuple t) {
        if (t.size() != 1)
          throw std::runtime_error("Invalid behavior model state!");
        /* Create a new C++ instance */
        return new BehaviorConstantVelocity(PythonToParams(t[0].cast<py::tuple>()));
      }));

  py::class_<BehaviorIDMClassic,
             BehaviorModel,
             shared_ptr<BehaviorIDMClassic>>(m, "BehaviorIDMClassic")
    .def(py::init<const modules::commons::ParamsPtr&>())
    .def("__repr__", [](const BehaviorIDMClassic &m) {
      return "bark.behavior.BehaviorIDMClassic";
    })
    .def(py::pickle(
      [](const BehaviorIDMClassic& b) {
        // We throw away other information such as last trajectories
        return py::make_tuple(ParamsToPython(b.GetParams()));
      },
      [](py::tuple t) {
        if (t.size() != 1)
          throw std::runtime_error("Invalid behavior model state!");
        return new BehaviorIDMClassic(PythonToParams(t[0].cast<py::tuple>()));
      }));
  
  py::class_<BehaviorIDMLaneTracking,
             BehaviorModel,
             shared_ptr<BehaviorIDMLaneTracking>>(m, "BehaviorIDMLaneTracking")
    .def(py::init<const modules::commons::ParamsPtr&>())
    .def("__repr__", [](const BehaviorIDMLaneTracking &m) {
      return "bark.behavior.BehaviorIDMLaneTracking";
    })
    .def(py::pickle(
      [](const BehaviorIDMLaneTracking& b) {
        // We throw away other information such as last trajectories
        return py::make_tuple(ParamsToPython(b.GetParams()));
      },
      [](py::tuple t) {
        if (t.size() != 1)
          throw std::runtime_error("Invalid behavior model state!");
        return new BehaviorIDMLaneTracking(PythonToParams(t[0].cast<py::tuple>()));
      }));

  py::class_<BehaviorMobil,
             BehaviorModel,
             shared_ptr<BehaviorMobil>>(m, "BehaviorMobil")
      .def(py::init<const modules::commons::ParamsPtr&>())
      .def("__repr__", [](const BehaviorMobil &m) {
        return "bark.behavior.BehaviorMobil";
      })
      .def(py::pickle(
        [](const BehaviorMobil &b) { 
            return py::make_tuple(ParamsToPython(b.GetParams()));
        },
        [](py::tuple t) { // __setstate__
            if (t.size() != 1)
                throw std::runtime_error("Invalid behavior model state!");

            /* Create a new C++ instance */
            return new BehaviorMobil(PythonToParams(t[0].cast<py::tuple>())); // param pointer must be set afterwards
        }));

  py::class_<BehaviorMotionPrimitives,
             BehaviorModel,
             shared_ptr<BehaviorMotionPrimitives>>(m,
    "BehaviorMotionPrimitives")
    .def("ActionToBehavior", &BehaviorMotionPrimitives::ActionToBehavior);

  py::class_<BehaviorMPContinuousActions,
             BehaviorMotionPrimitives,
             shared_ptr<BehaviorMPContinuousActions>>(m, "BehaviorMPContinuousActions")
    .def(py::init<const DynamicModelPtr&, const modules::commons::ParamsPtr&>())
    .def("__repr__", [](const BehaviorMPContinuousActions &b) {
      return "bark.behavior.BehaviorMPContinuousActions";
    })
    .def("AddMotionPrimitive", &BehaviorMPContinuousActions::AddMotionPrimitive);

  py::class_<Primitive,
            PyPrimitive,
            PrimitivePtr>(m, "Primitive")
  .def(py::init<const modules::commons::ParamsPtr&,
      const modules::models::dynamic::DynamicModelPtr&>())
  .def("Plan", &Primitive::Plan)
  .def("IsPreConditionSatisfied", &Primitive::IsPreConditionSatisfied);

  py::class_<BehaviorMPMacroActions,
            BehaviorModel,
            shared_ptr<BehaviorMPMacroActions>>(m, "BehaviorMPMacroActions")
  .def(py::init<const modules::models::dynamic::DynamicModelPtr&,
                const modules::commons::ParamsPtr&>())
  .def("GetNumMotionPrimitives", &BehaviorMPMacroActions::GetNumMotionPrimitives)
  .def("AddMotionPrimitive", &BehaviorMPMacroActions::AddMotionPrimitive);

  py::class_<PrimitiveConstAccStayLane,
             Primitive,
             std::shared_ptr<PrimitiveConstAccStayLane>>(m, "PrimitiveConstAccStayLane")
    .def(py::init<const modules::commons::ParamsPtr&,
        const modules::models::dynamic::DynamicModelPtr&,
        float, float>());

  py::class_<PrimitiveConstAccChangeToLeft,
             Primitive,
             std::shared_ptr<PrimitiveConstAccChangeToLeft>>(m, "PrimitiveConstAccChangeToLeft")
    .def(py::init<const modules::commons::ParamsPtr&,
        const modules::models::dynamic::DynamicModelPtr&,
        float>());
  
  py::class_<PrimitiveConstAccChangeToRight,
             Primitive,
             std::shared_ptr<PrimitiveConstAccChangeToRight>>(m, "PrimitiveConstAccChangeToRight")
    .def(py::init<const modules::commons::ParamsPtr&,
        const modules::models::dynamic::DynamicModelPtr&,
        float>());

  py::class_<DynamicBehaviorModel,
             BehaviorModel,
             shared_ptr<DynamicBehaviorModel>>(m, "DynamicBehaviorModel")
    .def(py::init<const DynamicModelPtr&,
          const modules::commons::ParamsPtr&>())
    .def("SetLastAction", &BehaviorModel::SetLastAction)
    .def("GetLastAction", &BehaviorModel::GetLastAction)
    .def("__repr__", [](const DynamicBehaviorModel &b) {
      return "bark.behavior.DynamicBehaviorModel";
    })
    .def(py::pickle(
      [](const DynamicBehaviorModel& b) {
        return py::make_tuple(
          b.GetDynamicModel(),
          ParamsToPython(b.GetParams()));
      },
      [](py::tuple t) {
        if (t.size() != 2)
          throw std::runtime_error("Invalid behavior model state!");
        return new DynamicBehaviorModel(
          t[0].cast<DynamicModelPtr>(),
          PythonToParams(t[1].cast<py::tuple>()));
      }));

  py::class_<BehaviorStaticTrajectory,
             BehaviorModel,
             shared_ptr<BehaviorStaticTrajectory>>(m, "BehaviorStaticTrajectory")
      .def(py::init<const modules::commons::ParamsPtr&>())
      .def(py::init<const modules::commons::ParamsPtr&, const modules::models::dynamic::Trajectory&>())
      .def_property_readonly("static_trajectory", &BehaviorStaticTrajectory::get_static_trajectory)
      .def("__repr__", [](const BehaviorStaticTrajectory &b) {
        return "bark.behavior.BehaviorStaticTrajectory";
      })
      .def(py::pickle(
      [](const BehaviorStaticTrajectory& b) {
        return py::make_tuple(ParamsToPython(b.GetParams()), b.get_static_trajectory());
      },
      [](py::tuple t) {
        if (t.size() != 2)
          throw std::runtime_error("Invalid behavior model state!");
        /* Create a new C++ instance */
        return new BehaviorStaticTrajectory(PythonToParams(t[0].cast<py::tuple>()), t[1].cast<modules::models::dynamic::Trajectory>());
      }));


  python_behavior_plan(m);
}