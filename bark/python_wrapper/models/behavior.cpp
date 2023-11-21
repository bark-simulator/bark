// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "behavior.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/behavior/dynamic_model/dynamic_model.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/behavior/idm/idm_lane_tracking.hpp"
#include "bark/models/behavior/idm/stochastic/idm_stochastic.hpp"
#include "bark/models/behavior/motion_primitives/continuous_actions.hpp"
#include "bark/models/behavior/motion_primitives/macro_actions.hpp"
#include "bark/models/behavior/motion_primitives/random_macro_actions.hpp"
#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"
#include "bark/models/behavior/motion_primitives/param_config/behav_macro_actions_from_param_server.hpp"
#include "bark/models/behavior/rule_based/intersection_behavior.hpp"
#include "bark/models/behavior/rule_based/lane_change_behavior.hpp"
#include "bark/models/behavior/rule_based/mobil_behavior.hpp"
#include "bark/models/behavior/static_trajectory/behavior_static_trajectory.hpp"
#include "bark/models/behavior/not_started/behavior_not_started.hpp"
#include "bark/models/behavior/behavior_safety/behavior_safety.hpp"
#include "bark/models/behavior/behavior_rss/behavior_rss.hpp"
#include "bark/python_wrapper/models/plan/plan.hpp"
#include "bark/python_wrapper/polymorphic_conversion.hpp"

namespace py = pybind11;
using namespace bark::models::behavior;
using namespace bark::models::behavior::primitives;
using bark::models::dynamic::DynamicModelPtr;
using bark::models::dynamic::AccelerationLimits;

using std::shared_ptr;
void python_behavior(py::module m) {
  py::class_<BehaviorModel, PyBehaviorModel, bark::commons::BaseType,
             BehaviorModelPtr>(m, "BehaviorModel")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("Plan", &BehaviorModel::Plan)
      .def("Clone", &BehaviorModel::Clone)
      .def("SetLastTrajectory", &BehaviorModel::SetLastTrajectory)
      .def("SetLastAction", &BehaviorModel::SetLastAction)
      .def("GetLastAction", &BehaviorModel::GetLastAction)
      .def("SetLastSolutionTime", &BehaviorModel::SetLastSolutionTime)
      .def("GetLastSolutionTime", &BehaviorModel::GetLastSolutionTime)
      .def("ActionToBehavior", &BehaviorModel::ActionToBehavior)
      .def_property("last_trajectory", &BehaviorModel::GetLastTrajectory,
                    &BehaviorModel::SetLastTrajectory)
      .def_property("behavior_status", &BehaviorModel::GetBehaviorStatus,
                    &BehaviorModel::SetBehaviorStatus);

  py::class_<BehaviorConstantAcceleration, BehaviorModel,
             shared_ptr<BehaviorConstantAcceleration>>(m,
                                                   "BehaviorConstantAcceleration")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("__repr__",
           [](const BehaviorConstantAcceleration& m) {
             return "bark.behavior.BehaviorConstantAcceleration";
           })
      .def(py::pickle(
          [](const BehaviorConstantAcceleration& b) {
            return py::make_tuple(ParamsToPython(b.GetParams()));
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid behavior model state!");
            /* Create a new C++ instance */
            return new BehaviorConstantAcceleration(
                PythonToParams(t[0].cast<py::tuple>()));
          }));

  py::class_<BehaviorIDMClassic, BehaviorModel, shared_ptr<BehaviorIDMClassic>>(
      m, "BehaviorIDMClassic", py::multiple_inheritance())
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("__repr__",
           [](const BehaviorIDMClassic& m) {
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
            return new BehaviorIDMClassic(
                PythonToParams(t[0].cast<py::tuple>()));
          }));

  py::class_<BehaviorIDMLaneTracking, BehaviorModel,
             shared_ptr<BehaviorIDMLaneTracking>>(m, "BehaviorIDMLaneTracking")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("SetLaneCorridor", &BehaviorIDMClassic::SetLaneCorridor)
      .def("__repr__",
           [](const BehaviorIDMLaneTracking& m) {
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
            return new BehaviorIDMLaneTracking(
                PythonToParams(t[0].cast<py::tuple>()));
          }));

  py::class_<BehaviorLaneChangeRuleBased, BehaviorModel,
             shared_ptr<BehaviorLaneChangeRuleBased>>(
      m, "BehaviorLaneChangeRuleBased")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("__repr__",
           [](const BehaviorLaneChangeRuleBased& m) {
             return "bark.behavior.BehaviorLaneChangeRuleBased";
           })
      .def(py::pickle(
          [](const BehaviorLaneChangeRuleBased& b) {
            return py::make_tuple(ParamsToPython(b.GetParams()));
          },
          [](py::tuple t) {  // __setstate__
            if (t.size() != 1)
              throw std::runtime_error("Invalid behavior model state!");
            /* Create a new C++ instance */
            return new BehaviorLaneChangeRuleBased(
                PythonToParams(t[0].cast<py::tuple>()));  // param pointer must
                                                          // be set afterwards
          }));

  py::class_<BehaviorMobilRuleBased, BehaviorModel,
             shared_ptr<BehaviorMobilRuleBased>>(m, "BehaviorMobilRuleBased")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("__repr__",
           [](const BehaviorMobilRuleBased& m) {
             return "bark.behavior.BehaviorMobilRuleBased";
           })
      .def(py::pickle(
          [](const BehaviorMobilRuleBased& b) {
            return py::make_tuple(ParamsToPython(b.GetParams()));
          },
          [](py::tuple t) {  // __setstate__
            if (t.size() != 1)
              throw std::runtime_error("Invalid behavior model state!");
            /* Create a new C++ instance */
            return new BehaviorMobilRuleBased(
                PythonToParams(t[0].cast<py::tuple>()));  // param pointer must
                                                          // be set afterwards
          }));

  py::class_<BehaviorIntersectionRuleBased, BehaviorModel,
             shared_ptr<BehaviorIntersectionRuleBased>>(
      m, "BehaviorIntersectionRuleBased")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("__repr__",
           [](const BehaviorIntersectionRuleBased& m) {
             return "bark.behavior.BehaviorIntersectionRuleBased";
           })
      .def(py::pickle(
          [](const BehaviorIntersectionRuleBased& b) {
            return py::make_tuple(ParamsToPython(b.GetParams()));
          },
          [](py::tuple t) {  // __setstate__
            if (t.size() != 1)
              throw std::runtime_error("Invalid behavior model state!");
            /* Create a new C++ instance */
            return new BehaviorIntersectionRuleBased(
                PythonToParams(t[0].cast<py::tuple>()));  // param pointer must
                                                          // be set afterwards
          }));

  py::class_<BehaviorIDMStochastic, BehaviorModel,
             shared_ptr<BehaviorIDMStochastic>>(m, "BehaviorIDMStochastic",
                                                py::multiple_inheritance())
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("__repr__",
           [](const BehaviorIDMStochastic& m) {
             return "bark.behavior.BehaviorIDMStochastic";
           })
      .def(py::pickle(
          [](const BehaviorIDMStochastic& b) {
            // We throw away other information such as last trajectories
            return py::make_tuple(ParamsToPython(b.GetParams()));
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid behavior model state!");
            return new BehaviorIDMStochastic(
                PythonToParams(t[0].cast<py::tuple>()));
          }));

  py::class_<BehaviorMotionPrimitives, BehaviorModel,
             shared_ptr<BehaviorMotionPrimitives>>(m,
                                                   "BehaviorMotionPrimitives")
      .def("ActionToBehavior", &BehaviorMotionPrimitives::ActionToBehavior)
      .def_property_readonly("num_motion_primitives",
                             &BehaviorMotionPrimitives::GetNumMotionPrimitives);

  py::class_<BehaviorMPContinuousActions, BehaviorMotionPrimitives,
             shared_ptr<BehaviorMPContinuousActions>>(
      m, "BehaviorMPContinuousActions")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("GetNumMotionPrimitives",
           &BehaviorMPContinuousActions::GetNumMotionPrimitives)
      .def("__repr__",
           [](const BehaviorMPContinuousActions& b) {
             return "bark.behavior.BehaviorMPContinuousActions";
           })
      .def("AddMotionPrimitive",
           &BehaviorMPContinuousActions::AddMotionPrimitive);

  py::class_<Primitive, PyPrimitive, PrimitivePtr>(m, "Primitive")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("Plan", &Primitive::Plan)
      .def("IsPreConditionSatisfied", &Primitive::IsPreConditionSatisfied)
      .def_property_readonly("name", &Primitive::GetName);

  py::class_<BehaviorMPMacroActions, BehaviorModel,
             shared_ptr<BehaviorMPMacroActions>>(m, "BehaviorMPMacroActions")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def(py::init<const bark::commons::ParamsPtr&,
                    const std::vector<primitives::PrimitivePtr>&>())
      .def("GetNumMotionPrimitives",
           &BehaviorMPMacroActions::GetNumMotionPrimitives)
      .def("GetMotionPrimitives", &BehaviorMPMacroActions::GetMotionPrimitives)
      .def("AddMotionPrimitive", &BehaviorMPMacroActions::AddMotionPrimitive)
      .def("__eq__", &BehaviorMPMacroActions::operator==)
      .def("IsEqualTo", &BehaviorMPMacroActions::IsEqualTo)
      .def("ClearMotionPrimitives",
           &BehaviorMPMacroActions::ClearMotionPrimitives)
      .def(py::pickle(
          [](const BehaviorMPMacroActions& b) {
            std::vector<py::tuple> prims;
            for (const auto& p : b.GetMotionPrimitives()) {
              prims.emplace_back(PrimitiveToPython(p));
            }
            return py::make_tuple(ParamsToPython(b.GetParams()), prims);
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid behavior model state!");
            auto tuples = t[1].cast<std::vector<py::tuple>>();
            std::vector<PrimitivePtr> prims;
            for (const auto& tup : tuples) {
              prims.emplace_back(PythonToPrimitive(tup));
            }
            return new BehaviorMPMacroActions(
                PythonToParams(t[0].cast<py::tuple>()), prims);
          }));
  
  py::class_<BehaviorRandomMacroActions, BehaviorMPMacroActions,
             shared_ptr<BehaviorRandomMacroActions>>(m, "BehaviorRandomMacroActions")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def(py::init<const bark::commons::ParamsPtr&,
                    const std::vector<primitives::PrimitivePtr>&>())
      .def(py::pickle(
          [](const BehaviorRandomMacroActions& b) {
            std::vector<py::tuple> prims;
            for (const auto& p : b.GetMotionPrimitives()) {
              prims.emplace_back(PrimitiveToPython(p));
            }
            return py::make_tuple(ParamsToPython(b.GetParams()), prims);
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid behavior model state!");
            auto tuples = t[1].cast<std::vector<py::tuple>>();
            std::vector<PrimitivePtr> prims;
            for (const auto& tup : tuples) {
              prims.emplace_back(PythonToPrimitive(tup));
            }
            return new BehaviorRandomMacroActions(
                PythonToParams(t[0].cast<py::tuple>()), prims);
          }));

  py::class_<PrimitiveGapKeeping, Primitive,
             std::shared_ptr<PrimitiveGapKeeping>>(m, "PrimitiveGapKeeping")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def(py::pickle(
          [](const PrimitiveGapKeeping& b) {
            return py::make_tuple(ParamsToPython(b.Primitive::GetParams()));
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid behavior model state!");
            return new PrimitiveGapKeeping(
                PythonToParams(t[0].cast<py::tuple>()));
          }));

  py::class_<PrimitiveConstAccStayLane, Primitive,
             std::shared_ptr<PrimitiveConstAccStayLane>>(
      m, "PrimitiveConstAccStayLane")
      .def(py::init<const bark::commons::ParamsPtr&, double>())
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def(py::pickle(
          [](const PrimitiveConstAccStayLane& b) {
            return py::make_tuple(ParamsToPython(b.Primitive::GetParams()), 
                                    b.GetAcceleration());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid behavior model state!");
            return new PrimitiveConstAccStayLane(
                PythonToParams(t[0].cast<py::tuple>()), t[1].cast<double>());
          }));

  py::class_<PrimitiveConstAccChangeToLeft, Primitive,
             std::shared_ptr<PrimitiveConstAccChangeToLeft>>(
      m, "PrimitiveConstAccChangeToLeft")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def(py::pickle(
          [](const PrimitiveConstAccChangeToLeft& b) {
            return py::make_tuple(ParamsToPython(b.Primitive::GetParams()));
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid behavior model state!");
            return new PrimitiveConstAccChangeToLeft(
                PythonToParams(t[0].cast<py::tuple>()));
          }));

  py::class_<PrimitiveConstAccChangeToRight, Primitive,
             std::shared_ptr<PrimitiveConstAccChangeToRight>>(
      m, "PrimitiveConstAccChangeToRight")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def(py::pickle(
          [](const PrimitiveConstAccChangeToRight& b) {
            return py::make_tuple(ParamsToPython(b.Primitive::GetParams()));
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid behavior model state!");
            return new PrimitiveConstAccChangeToRight(
                PythonToParams(t[0].cast<py::tuple>()));
          }));

  m.def("BehaviorMacroActionsFromParamServer",
        &BehaviorMacroActionsFromParamServer);

  py::class_<BehaviorDynamicModel, BehaviorModel,
             shared_ptr<BehaviorDynamicModel>>(m, "BehaviorDynamicModel")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("SetLastAction", &BehaviorModel::SetLastAction)
      .def("GetAction", &BehaviorDynamicModel::GetAction)
      .def("ActionToBehavior", &BehaviorDynamicModel::ActionToBehavior)
      .def("__repr__",
           [](const BehaviorDynamicModel& b) {
             return "bark.behavior.BehaviorDynamicModel";
           })
      .def(py::pickle(
          [](const BehaviorDynamicModel& b) {
            return py::make_tuple(ParamsToPython(b.GetParams()));
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid behavior model state!");
            return new BehaviorDynamicModel(
                PythonToParams(t[0].cast<py::tuple>()));
          }));

  py::class_<BehaviorStaticTrajectory, BehaviorModel,
             shared_ptr<BehaviorStaticTrajectory>>(m,
                                                   "BehaviorStaticTrajectory")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def(py::init<const bark::commons::ParamsPtr&,
                    const bark::models::dynamic::Trajectory&>())
      .def_property_readonly("static_trajectory",
                             &BehaviorStaticTrajectory::GetStaticTrajectory)
      .def("__repr__",
           [](const BehaviorStaticTrajectory& b) {
             return "bark.behavior.BehaviorStaticTrajectory";
           })
      .def(py::pickle(
          [](const BehaviorStaticTrajectory& b) {
            return py::make_tuple(ParamsToPython(b.GetParams()),
                                  b.GetStaticTrajectory());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid behavior model state!");
            /* Create a new C++ instance */
            return new BehaviorStaticTrajectory(
                PythonToParams(t[0].cast<py::tuple>()),
                t[1].cast<bark::models::dynamic::Trajectory>());
          }));

  py::class_<BehaviorNotStarted, BehaviorModel,
             shared_ptr<BehaviorNotStarted>>(m, "BehaviorNotStarted")
      .def(py::init<const bark::commons::ParamsPtr&>())
      .def("__repr__",
           [](const BehaviorNotStarted& b) {
             return "bark.behavior.BehaviorNotStarted";
           })
      .def(py::pickle(
          [](const BehaviorNotStarted& b) {
            return py::make_tuple(ParamsToPython(b.GetParams()));
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid behavior model state!");
            /* Create a new C++ instance */
            return new BehaviorNotStarted(
                PythonToParams(t[0].cast<py::tuple>()));
          }));
  
  py::class_<BehaviorSafety, BehaviorModel,
             shared_ptr<BehaviorSafety>>(m, "BehaviorSafety")
    .def(py::init<const bark::commons::ParamsPtr&>())
    .def("SetBehaviorModel", &BehaviorSafety::SetBehaviorModel)
    .def("__repr__",
      [](const BehaviorSafety& b) {
        return "bark.behavior.BehaviorSafety";
      })
    .def(py::pickle(
      [](const BehaviorSafety& b) {
        return py::make_tuple(
          ParamsToPython(b.GetParams()),
          ParamsToPython(b.GetBehaviorModel()->GetParams()));
      },
      [](py::tuple t) {
        if (t.size() != 2)
          throw std::runtime_error("Invalid behavior model state!");
        /* Create a new C++ instance */
        auto bs = new BehaviorSafety(
          PythonToParams(t[0].cast<py::tuple>()));
        auto bm = std::make_shared<BehaviorIDMLaneTracking>(
          PythonToParams(t[1].cast<py::tuple>()));
        bs->SetBehaviorModel(bm);
        return bs;
      }));

  py::class_<BehaviorRSSConformant, BehaviorModel,
             shared_ptr<BehaviorRSSConformant>>(m, "BehaviorRSSConformant")
    .def(py::init<const bark::commons::ParamsPtr&>())
    #ifdef RSS
    .def("SetNominalBehaviorModel", &BehaviorRSSConformant::SetNominalBehaviorModel)
    .def("SetSafetyBehaviorModel", &BehaviorRSSConformant::SetSafetyBehaviorModel)
    .def("GetLongitudinalResponse", &BehaviorRSSConformant::GetLongitudinalResponse)
    .def("GetLateralLeftResponse", &BehaviorRSSConformant::GetLateralLeftResponse)
    .def("GetLateralRightResponse", &BehaviorRSSConformant::GetLateralRightResponse)
    .def("SetLongitudinalResponse", &BehaviorRSSConformant::SetLongitudinalResponse)
    .def("SetLateralLeftResponse", &BehaviorRSSConformant::SetLateralLeftResponse)
    .def("SetLateralRightResponse", &BehaviorRSSConformant::SetLateralRightResponse)
    .def("GetSafetyPolygons", &BehaviorRSSConformant::GetSafetyPolygons)
    .def("ComputeSafetyPolygons", &BehaviorRSSConformant::ComputeSafetyPolygons)
    #endif
    .def("GetAccelerationLimitsVehicleCs", &BehaviorRSSConformant::GetAccelerationLimitsVehicleCs)
    .def("GetAccelerationLimitsStreetCs", &BehaviorRSSConformant::GetAccelerationLimitsStreetCs)
    .def("__repr__",
      [](const BehaviorRSSConformant& b) {
        return "bark.behavior.BehaviorRSSConformant";
      })
    .def(py::pickle(
      [](const BehaviorRSSConformant& b) {
        // TODO: store safety polygons
        #ifdef RSS
        return py::make_tuple(
          ParamsToPython(b.GetParams()), 
          ParamsToPython(b.GetNominalBehaviorModel()->GetParams()), 
          ParamsToPython(b.GetBehaviorSafetyModel()->GetParams()),
          b.GetLongitudinalResponse(),
          b.GetLateralLeftResponse(),
          b.GetLateralRightResponse(),
          b.GetSafetyPolygons(),
          b.GetAccelerationLimitsVehicleCs(),
          b.GetAccelerationLimitsStreetCs());
        #endif
        return py::make_tuple(
          ParamsToPython(b.GetParams()), 
          ParamsToPython(b.GetNominalBehaviorModel()->GetParams()), 
          ParamsToPython(b.GetBehaviorSafetyModel()->GetParams()));
      },
      [](py::tuple t) {
        int num_params = 3;
        #ifdef RSS
        num_params = 9;
        #endif
        if (t.size() != num_params)
          throw std::runtime_error("Invalid behavior model state!");
        /* Create a new C++ instance */
        auto bm = new BehaviorRSSConformant(
          PythonToParams(t[0].cast<py::tuple>()));
        auto nb = std::make_shared<BehaviorIDMLaneTracking>(
          PythonToParams(t[1].cast<py::tuple>()));
        auto sb = std::make_shared<BehaviorSafety>(
          PythonToParams(t[2].cast<py::tuple>()));
        bm->SetNominalBehaviorModel(nb);
        bm->SetSafetyBehaviorModel(sb);
        #ifdef RSS
        // safety responses
        bm->SetLongitudinalResponse(t[3].cast<bool>());
        bm->SetLateralLeftResponse(t[4].cast<bool>());
        bm->SetLateralRightResponse(t[5].cast<bool>());
        bm->SetSafetyPolygons(t[6].cast<std::vector<SafetyPolygon>>());
        // TODO: load safety polygons
        bm->SetAccelerationLimitsVehicleCs(t[7].cast<AccelerationLimits>());
        bm->SetAccelerationLimitsStreetCs(t[8].cast<AccelerationLimits>());
        #endif
        return bm;
      }));
  
  py::class_<LonLatAction, shared_ptr<LonLatAction>>(m, "LonLatAction")
      .def(py::init<>())
      .def_readwrite("acc_lat", &LonLatAction::acc_lat)
      .def_readwrite("acc_lon", &LonLatAction::acc_lon)
      .def("__repr__",
           [](const LonLatAction& b) { return "bark.behavior.LonLatAction"; })
      .def(py::pickle(
          [](const LonLatAction& a) {
            return py::make_tuple(a.acc_lat, a.acc_lon);
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid LonLatAction model state!");
            /* Create a new C++ instance */
            return new LonLatAction{t[0].cast<Continuous1DAction>(),
                                    t[1].cast<Continuous1DAction>()};
          }));

  m.def("BehaviorMacroActionsFromParamServer",
        &BehaviorMacroActionsFromParamServer);

  python_behavior_plan(m);

  py::enum_<BehaviorStatus>(m, "BehaviorStatus", py::arithmetic())
    .value("NOT_STARTED_YET", NOT_STARTED_YET)
    .value("VALID", VALID)
    .value("EXPIRED", EXPIRED)
    .export_values();
}
