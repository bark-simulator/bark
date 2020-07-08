// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_HPP_
#define BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_HPP_

#include <memory>
#include "bark/commons/base_type.hpp"
#include "bark/models/behavior/idm/idm_lane_tracking.hpp"
#include "bark/models/dynamic/integration.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/map/road_corridor.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {
namespace primitives {

using bark::geometry::Point2d;
using bark::models::behavior::BehaviorIDMLaneTracking;
using bark::models::dynamic::DynamicModelPtr;
using bark::models::dynamic::StateDefinition;
using dynamic::Trajectory;
using world::ObservedWorld;
using world::ObservedWorldPtr;
using world::map::LaneCorridorPtr;

struct AdjacentLaneCorridors {
  LaneCorridorPtr current;
  LaneCorridorPtr left;
  LaneCorridorPtr right;
};

/**
 * @brief Macro action motion primitive base class
 */
class Primitive : public bark::commons::BaseType {
 public:
  explicit Primitive(const commons::ParamsPtr& params)
      : commons::BaseType(params),
        integration_time_delta_(params->GetReal(
            "BehaviorMotionPrimitives::IntegrationTimeDelta",
            "the size of the time steps used within the euler integration loop",
            0.02)),
        last_action_() {}

  virtual ~Primitive() = default;

  /**
   * @brief Precondition for the motion primitve to be available
   * @param adjacent_corridors Current target corridor and left/right corridors
   * @return True if primitive is available
   */
  virtual bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) = 0;
  virtual Trajectory Plan(float min_planning_time,
                          const ObservedWorld& observed_world,
                          const LaneCorridorPtr& target_corridor) = 0;
  /**
   * @brief Select the new target corridor
   * @param adjacent_corridors Current target corridor and left/right corridors
   * @return The new target corridor
   */
  virtual LaneCorridorPtr SelectTargetCorridor(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) = 0;

  Action GetLastAction() const { return last_action_; };
  void SetLastAction(const Action action) { last_action_ = action; };

 protected:
  float integration_time_delta_;

 private:
  Action last_action_;
};

typedef std::shared_ptr<Primitive> PrimitivePtr;

}  // namespace primitives
}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_HPP_