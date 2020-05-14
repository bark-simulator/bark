// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVES_HPP_
#define MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVES_HPP_

#include <memory>
#include "modules/commons/base_type.hpp"
#include "modules/models/behavior/idm/idm_lane_tracking.hpp"
#include "modules/models/dynamic/integration.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/world/map/road_corridor.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {
namespace primitives {

using dynamic::Trajectory;
using modules::models::dynamic::DynamicModelPtr;
using modules::geometry::Point2d;
using modules::models::behavior::BehaviorIDMLaneTracking;
using modules::models::dynamic::StateDefinition;
using world::ObservedWorld;
using world::ObservedWorldPtr;
using world::map::LaneCorridorPtr;

struct AdjacentLaneCorridors{
  LaneCorridorPtr current;
  LaneCorridorPtr left;
  LaneCorridorPtr right;
};

// TODO(@esterle, @bernhard): Add documentation
class Primitive : public modules::commons::BaseType {
 public:
  explicit Primitive(const commons::ParamsPtr& params)
      : commons::BaseType(params),
        integration_time_delta_(params->GetReal(
            "BehaviorMotionPrimitives::IntegrationTimeDelta",
            "the size of the time steps used within the euler integration loop",
            0.02)) {}

  virtual ~Primitive() = default;

  virtual bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) = 0;
  virtual Trajectory Plan(float delta_time, const ObservedWorld& observed_world,
                          const LaneCorridorPtr& target_corridor) = 0;
  virtual LaneCorridorPtr SelectTargetCorridor(const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) = 0;
 protected:
  float integration_time_delta_;
};

typedef std::shared_ptr<Primitive> PrimitivePtr;

class PrimitiveConstAcceleration : public Primitive, BehaviorIDMLaneTracking {
  // Covers Primitives KeepVelocity, Accelerat, Decelerate
 public:
  PrimitiveConstAcceleration(const commons::ParamsPtr& params,
                             float acceleration)
      : Primitive(params), BehaviorIDMLaneTracking(params),
        acceleration_(acceleration) {}
  explicit PrimitiveConstAcceleration(const commons::ParamsPtr& params)
      : Primitive(params), BehaviorIDMLaneTracking(params),
        acceleration_(
            params->GetReal("PrimitiveConstAcceleration::Acceleration",
                            "Constant acceleration to apply", 0.0)) {}
  bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override {
    auto ego_vel = observed_world.GetEgoAgent()->GetCurrentState()(dynamic::StateDefinition::VEL_POSITION);
    return (acceleration_ < 0.0f && ego_vel > 1.0e-5f) || (acceleration_ > 0.0f && ego_vel >= 50.0f) || acceleration_ == 0.0f;
  }

  // TODO(@esterle, @bernhard): Use BehaviorIDMLaneTracking
  Trajectory Plan(float delta_time, const ObservedWorld& observed_world,
                  const LaneCorridorPtr& target_corridor) {
    SetBehaviorStatus(BehaviorStatus::VALID);

    if (!target_corridor) {
      LOG(INFO) << "Agent " << observed_world.GetEgoAgentId()
                << ": Behavior status has expired!" << std::endl;
      SetBehaviorStatus(BehaviorStatus::EXPIRED);
      return GetLastTrajectory();
    }

    double dt = delta_time / (GetNumTrajectoryTimePoints() - 1);
    // interaction term off and GetTotalAcc returns const. acc.
    IDMRelativeValues rel_values{0., 0., false};
    std::tuple<Trajectory, Action> traj_action =
        GenerateTrajectory(
            observed_world, target_corridor, rel_values, dt);

    // set values
    Trajectory traj = std::get<0>(traj_action);
    Action action = std::get<1>(traj_action);
    SetLastTrajectory(traj);
    SetLastAction(action);
    return traj;
  }

  virtual LaneCorridorPtr SelectTargetCorridor(const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override {
    BARK_EXPECT_TRUE(adjacent_corridors.current);
    return adjacent_corridors.current;
  }

 protected:
  std::pair<double, double> GetTotalAcc(const ObservedWorld& observed_world,
                                   const IDMRelativeValues& rel_values,
                                   double rel_distance,
                                   double dt) const override {
    return {acceleration_, 0.0f};
  }

 private:
  float acceleration_;
};

// TODO(@esterle, @bernhard): Add documentation
class PrimitiveGapKeeping : public Primitive, BehaviorIDMLaneTracking {
 public:
  explicit PrimitiveGapKeeping(const commons::ParamsPtr& params)
      : Primitive(params), BehaviorIDMLaneTracking(params) {}
  bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override {
    return true;
  }
  Trajectory Plan(float delta_time, const ObservedWorld& observed_world,
                  const LaneCorridorPtr& target_corridor) override {
    return BehaviorIDMLaneTracking::Plan(delta_time, observed_world);
  }
  LaneCorridorPtr SelectTargetCorridor(const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override {
    return observed_world.GetRoadCorridor()->GetCurrentLaneCorridor(
        observed_world.CurrentEgoPosition());
  }
};

// TODO(@esterle, @bernhard): Add documentation
class PrimitiveConstAccChangeToLeft : public PrimitiveConstAcceleration {
 public:
  explicit PrimitiveConstAccChangeToLeft(const commons::ParamsPtr& params)
      : PrimitiveConstAcceleration(params),
        min_length_(params->GetReal("MinLength",
                                    "Minimum length of lane to change to",
                                    0.0f)) {}

  LaneCorridorPtr SelectTargetCorridor(const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override {
    if (adjacent_corridors.left) {
      return adjacent_corridors.left;
    }
    LOG(WARNING) << "Called change to left, but left corridor not found!";
    if(!adjacent_corridors.current){
      return
          observed_world.GetRoadCorridor()->GetCurrentLaneCorridor(
              observed_world.CurrentEgoPosition());
    } else {
      return adjacent_corridors.current;
    }
  }

  bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override {
    bool satisfied;
    const Point2d ego_pos = observed_world.CurrentEgoPosition();
    //! agent may not have reached target lane yet, so we match point on target
    //! lane
    const Point2d point_on_target_line =
        GetNearestPoint(adjacent_corridors.left->GetCenterLine(), ego_pos);
    if (adjacent_corridors.left) {
      float remaining_length =
          adjacent_corridors.left->LengthUntilEnd(point_on_target_line);
      satisfied = remaining_length >= min_length_;
    } else {
      satisfied = false;
    }
    return satisfied;
  }

 private:
  float min_length_;
};


// TODO(@esterle, @bernhard): Add documentation
class PrimitiveConstAccChangeToRight : public PrimitiveConstAcceleration {
 public:
  explicit PrimitiveConstAccChangeToRight(const commons::ParamsPtr& params)
      : PrimitiveConstAcceleration(params),
        min_length_(params->GetReal("MinLength",
                                    "Minimum length of lane to change to",
                                    0.0f)) {}

  LaneCorridorPtr SelectTargetCorridor(const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override {
    if (adjacent_corridors.right) {
      return adjacent_corridors.right;
    }
    LOG(WARNING) << "Called change to right, but right corridor not found!";
    if(!adjacent_corridors.current){
      return
          observed_world.GetRoadCorridor()->GetCurrentLaneCorridor(
              observed_world.CurrentEgoPosition());
    } else {
      return adjacent_corridors.current;
    }
  }

  bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) override {
    bool satisfied;
    const Point2d ego_pos = observed_world.CurrentEgoPosition();
    //! agent may not have reached target lane yet, so we match point on target
    //! lane
    const Point2d point_on_target_line =
        GetNearestPoint(adjacent_corridors.right->GetCenterLine(), ego_pos);
    if (adjacent_corridors.right) {
      float remaining_length =
          adjacent_corridors.right->LengthUntilEnd(point_on_target_line);
      satisfied = remaining_length >= min_length_;
    } else {
      satisfied = false;
    }
    return satisfied;
  }

 private:
  float min_length_;
};

}  // namespace primitives
}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVES_HPP_