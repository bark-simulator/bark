// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVES_HPP_
#define MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVES_HPP_

#include <memory>
#include "modules/commons/base_type.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/dynamic/integration.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/world/map/road_corridor.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {
namespace primitives {

using dynamic::Trajectory;
using modules::geometry::Point2d;
using modules::models::behavior::BehaviorIDMClassic;
using modules::models::dynamic::StateDefinition;
using world::ObservedWorld;
using world::ObservedWorldPtr;
using world::map::LaneCorridorPtr;

class Primitive : public modules::commons::BaseType {
 public:
  explicit Primitive(const commons::ParamsPtr& params,
                     const DynamicModelPtr& dynamic_model)
      : commons::BaseType(params),
        dynamic_model_(dynamic_model),
        integration_time_delta_(params->GetReal(
            "integration_time_delta",
            "the size of the time steps used within the euler integration loop",
            0.02)) {}

  virtual ~Primitive() {}

  virtual bool IsPreConditionSatisfied(
      const ObservedWorldPtr& observed_world) = 0;
  virtual Trajectory Plan(float delta_time,
                          const ObservedWorld& observed_world) = 0;

 protected:
  float integration_time_delta_;
  DynamicModelPtr dynamic_model_;
};

typedef std::shared_ptr<Primitive> PrimitivePtr;

class PrimitiveLaneFollowing : public Primitive {
 public:
  PrimitiveLaneFollowing(const commons::ParamsPtr& params,
                         const DynamicModelPtr& dynamic_model)
      : Primitive(params, dynamic_model) {}
  void EnsureTargetCorridor(const ObservedWorld& observed_world) {
    // make sure target_corridor_ is not null, as we cannot initialize it before
    if (!target_corridor_)
      target_corridor_ =
          observed_world.GetRoadCorridor()->GetCurrentLaneCorridor(
              observed_world.CurrentEgoPosition());
  }

 protected:
  LaneCorridorPtr target_corridor_;
};

class PrimitiveConstAcceleration : public PrimitiveLaneFollowing {
  // Covers Primitives KeepVelocity, Accelerat, Decelerate
 public:
  PrimitiveConstAcceleration(const commons::ParamsPtr& params,
                             const DynamicModelPtr& dynamic_model,
                             float acceleration, float crosstrack_error_gain)
      : PrimitiveLaneFollowing(params, dynamic_model),
        acceleration_(acceleration),
        crosstrack_error_gain_(crosstrack_error_gain) {}
  bool IsPreConditionSatisfied(const ObservedWorldPtr& observed_world) {
    return true;
  }
  Trajectory Plan(float delta_time, const ObservedWorld& observed_world) {
    PrimitiveLaneFollowing::EnsureTargetCorridor(observed_world);

    auto single_track =
        std::dynamic_pointer_cast<dynamic::SingleTrackModel>(dynamic_model_);
    if (!single_track) {
      LOG(FATAL) << "Only SingleTrack as dynamic model supported!";
    }

    const float dt = integration_time_delta_;
    const int num_trajectory_points =
        static_cast<int>(std::ceil(delta_time / dt)) + 1;

    Trajectory traj(num_trajectory_points,
                    static_cast<int>(StateDefinition::MIN_STATE_SIZE));
    traj.row(0) = observed_world.CurrentEgoState();

    float integration_time;
    for (int i = 1; i < num_trajectory_points; ++i) {
      if (i == num_trajectory_points - 1) {
        // calculate the last time pt, which might not fit to dt
        integration_time = delta_time - (i - 1) * dt;
      } else {
        integration_time = dt;
      }

      float angle = CalculateSteeringAngle(single_track, traj.row(i - 1),
                                           target_corridor_->GetCenterLine(),
                                           crosstrack_error_gain_);
      Input input(2);
      input << acceleration_, angle;

      traj.row(i) = dynamic::euler_int(*single_track, traj.row(i - 1), input,
                                       integration_time);
    }
    return traj;
  }

 private:
  float acceleration_;
  float crosstrack_error_gain_;
};

class PrimitiveGapKeeping : public PrimitiveLaneFollowing, BehaviorIDMClassic {
 public:
  PrimitiveGapKeeping(const commons::ParamsPtr& params,
                      const DynamicModelPtr& dynamic_model)
      : PrimitiveLaneFollowing(params, dynamic_model),
        BehaviorIDMClassic(params) {}
  bool IsPreConditionSatisfied(const ObservedWorldPtr& observed_world) {
    // TODO: which lane to check? should be checked for target lane
    auto leading_vehicle = observed_world->GetAgentInFront();
    bool satisfied = (leading_vehicle.first) ? true : false;
  }
  Trajectory Plan(float delta_time, const ObservedWorld& observed_world) {
    // TODO: better call IDM that tracks a lane using a controller
    auto traj = BehaviorIDMClassic::Plan(delta_time, observed_world);
    return traj;
  }
};

class PrimitiveChangeToLeft : public PrimitiveConstAcceleration {
 public:
  PrimitiveChangeToLeft(const commons::ParamsPtr& params,
                        const DynamicModelPtr& dynamic_model,
                        float crosstrack_error_gain)
      : PrimitiveConstAcceleration(params, dynamic_model, 0,
                                   crosstrack_error_gain) {}
  bool IsPreConditionSatisfied(const ObservedWorldPtr& observed_world) {
    
    // const Point2d ego_pos = observed_world->CurrentEgoPosition();
    // //! agent may not have reached target lane yet, so we match point on target
    // //! lane
    // const Point2d point_on_target_line =
    //     GetNearestPoint(target_corridor_->GetCenterLine(), ego_pos);

    // auto road_corridor = observed_world->GetRoadCorridor();
    // LaneCorridorPtr left_corridor;
    // LaneCorridorPtr right_corridor;
    // std::tie(left_corridor, right_corridor) =
    //     road_corridor->GetLeftRightLaneCorridor(point_on_target_line);

    // bool satisfied = (left_corridor) ? true : false;
    bool satisfied = true;
    return satisfied;
  }
  Trajectory Plan(float delta_time, const ObservedWorld& observed_world) {
    PrimitiveLaneFollowing::EnsureTargetCorridor(observed_world);

    const Point2d ego_pos = observed_world.CurrentEgoPosition();
    //! agent may not have reached target lane yet, so we match point on target
    //! lane
    const Point2d point_on_target_line =
        GetNearestPoint(target_corridor_->GetCenterLine(), ego_pos);

    auto road_corridor = observed_world.GetRoadCorridor();
    LaneCorridorPtr left_corridor;
    LaneCorridorPtr right_corridor;
    std::tie(left_corridor, right_corridor) =
        road_corridor->GetLeftRightLaneCorridor(point_on_target_line);
    target_corridor_ = left_corridor;

    auto traj = PrimitiveConstAcceleration::Plan(delta_time, observed_world);
    return traj;
  }
};

class PrimitiveChangeToRight : public PrimitiveConstAcceleration {
 public:
  PrimitiveChangeToRight(const commons::ParamsPtr& params,
                         const DynamicModelPtr& dynamic_model,
                         float crosstrack_error_gain)
      : PrimitiveConstAcceleration(params, dynamic_model, 0,
                                   crosstrack_error_gain) {}
  bool IsPreConditionSatisfied(const ObservedWorldPtr& observed_world) {

    // const Point2d ego_pos = observed_world->CurrentEgoPosition();
    // //! agent may not have reached target lane yet, so we match point on target
    // //! lane
    // const Point2d point_on_target_line =
    //     GetNearestPoint(target_corridor_->GetCenterLine(), ego_pos);

    // auto road_corridor = observed_world->GetRoadCorridor();
    // LaneCorridorPtr left_corridor;
    // LaneCorridorPtr right_corridor;
    // std::tie(left_corridor, right_corridor) =
    //     road_corridor->GetLeftRightLaneCorridor(point_on_target_line);

    // bool satisfied = (right_corridor) ? true : false;
    bool satisfied = true;
    return satisfied;
  }
  Trajectory Plan(float delta_time, const ObservedWorld& observed_world) {
    PrimitiveLaneFollowing::EnsureTargetCorridor(observed_world);

    const Point2d ego_pos = observed_world.CurrentEgoPosition();
    //! agent may not have reached target lane yet, so we match point on target
    //! lane
    const Point2d point_on_target_line =
        GetNearestPoint(target_corridor_->GetCenterLine(), ego_pos);

    auto road_corridor = observed_world.GetRoadCorridor();
    LaneCorridorPtr left_corridor;
    LaneCorridorPtr right_corridor;
    std::tie(left_corridor, right_corridor) =
        road_corridor->GetLeftRightLaneCorridor(point_on_target_line);
    target_corridor_ = right_corridor;

    auto traj = PrimitiveConstAcceleration::Plan(delta_time, observed_world);
    return traj;
  }
};

}  // namespace primitives
}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVES_HPP_