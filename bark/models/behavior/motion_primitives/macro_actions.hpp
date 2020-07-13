// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_MACRO_ACTIONS_HPP_
#define BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_MACRO_ACTIONS_HPP_

#include <vector>

#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive.hpp"
#include "bark/world/map/lane_corridor.hpp"

namespace bark {
namespace models {
namespace behavior {

using commons::ParamsPtr;
using primitives::AdjacentLaneCorridors;
using world::map::LaneCorridorPtr;

class BehaviorMPMacroActions : public BehaviorMotionPrimitives {
 public:
  BehaviorMPMacroActions(
      const ParamsPtr& params,
      const std::vector<primitives::PrimitivePtr>& motion_primitives);
  explicit BehaviorMPMacroActions(const ParamsPtr& params)
      : BehaviorMPMacroActions(params, {}) {}

  ~BehaviorMPMacroActions() = default;

  Trajectory Plan(float min_planning_time,
                  const ObservedWorld& observed_world) override;

  MotionIdx GetNumMotionPrimitives(
      const ObservedWorldPtr& observed_world) override;

  MotionIdx AddMotionPrimitive(const primitives::PrimitivePtr& primitive);

  void ClearMotionPrimitives() { motion_primitives_.clear(); }

  std::shared_ptr<BehaviorModel> Clone() const override;
  const std::vector<primitives::PrimitivePtr>& GetMotionPrimitives() const;
  const std::vector<BehaviorMPMacroActions::MotionIdx>& GetValidPrimitives(
      const ObservedWorldPtr& observed_world);

 private:
  MotionIdx GetNumMotionPrimitivesByCorridors(
      const ObservedWorld& observed_world,
      const AdjacentLaneCorridors& adjacent_corridors);
  std::vector<primitives::PrimitivePtr> motion_primitives_;
  std::vector<MotionIdx> valid_primitives_;
  bool check_validity_in_plan_;
  LaneCorridorPtr target_corridor_;
  AdjacentLaneCorridors GetCorridors(const ObservedWorld& observed_world);
};

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_MACRO_ACTIONS_HPP_
