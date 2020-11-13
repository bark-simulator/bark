// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_RANDOM_MACRO_ACTIONS_HPP_
#define BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_RANDOM_MACRO_ACTIONS_HPP_

#include <vector>
#include <random>

#include "bark/models/behavior/motion_primitives/motion_primitives.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive.hpp"
#include "bark/world/map/lane_corridor.hpp"

namespace bark {
namespace models {
namespace behavior {

using commons::ParamsPtr;
using primitives::AdjacentLaneCorridors;
using world::map::LaneCorridorPtr;

class BehaviorRandomMacroActions : public BehaviorMPMacroActions {
 public:
  BehaviorRandomMacroActions(
      const ParamsPtr& params,
      const std::vector<primitives::PrimitivePtr>& motion_primitives)
       : BehaviorMPMacroActions(params, motion_primitives),
        random_generator_([](){ std::random_device rd; return rd();}()) {}
  explicit BehaviorRandomMacroActions(const ParamsPtr& params)
      : BehaviorRandomMacroActions(params, {}) {}

  ~BehaviorRandomMacroActions() = default;

  Trajectory Plan(float min_planning_time,
                  const ObservedWorld& observed_world) override {
    std::uniform_int_distribution<> distrib(0,
              BehaviorMPMacroActions::GetMotionPrimitives().size() - 1);
    BehaviorMPMacroActions::ActionToBehavior(DiscreteAction(distrib(random_generator_)));
    return BehaviorMPMacroActions::Plan(min_planning_time, observed_world);
  }

  std::shared_ptr<BehaviorModel> Clone() const override;

  private:
     std::mt19937 random_generator_;
};

inline std::shared_ptr<BehaviorModel> BehaviorRandomMacroActions::Clone()
    const {
  return std::make_shared<BehaviorRandomMacroActions>(*this);
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_MOTION_PRIMITIVES_RANDOM_MACRO_ACTIONS_HPP_
