// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_HPP_
#define MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_HPP_

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
using modules::geometry::Point2d;
using modules::models::behavior::BehaviorIDMLaneTracking;
using modules::models::dynamic::DynamicModelPtr;
using modules::models::dynamic::StateDefinition;
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
class Primitive : public modules::commons::BaseType {
 public:
  explicit Primitive(const commons::ParamsPtr& params)
      : commons::BaseType(params),
        integration_time_delta_(params->GetReal(
            "BehaviorMotionPrimitives::IntegrationTimeDelta",
            "the size of the time steps used within the euler integration loop",
            0.02)) {}

  virtual ~Primitive() = default;

  /**
   * @brief Precondition for the motion primitve to be available
   * @param adjacent_corridors Current target corridor and left/right corridors
   * @return True if primitive is available
   */
  virtual bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) = 0;
  virtual Trajectory Plan(float delta_time, const ObservedWorld& observed_world,
                          const LaneCorridorPtr& target_corridor) = 0;
  /**
   * @brief Select the new target corridor
   * @param adjacent_corridors Current target corridor and left/right corridors
   * @return The new target corridor
   */
  virtual LaneCorridorPtr SelectTargetCorridor(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors) = 0;

 protected:
  float integration_time_delta_;
};

typedef std::shared_ptr<Primitive> PrimitivePtr;

}  // namespace primitives
}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_PRIMITIVES_PRIMITIVE_HPP_