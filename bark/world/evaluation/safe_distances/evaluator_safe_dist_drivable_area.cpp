// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/observed_world.hpp"
#include "bark/world/evaluation/safe_distances/evaluator_safe_dist_drivable_area.hpp"

namespace bark {
namespace world {
namespace evaluation {

using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::geometry::Pose;
using bark::models::dynamic::State;
using bark::models::dynamic::StateDefinition;

EvaluatorSafeDistDrivableArea::EvaluatorSafeDistDrivableArea(const bark::commons::ParamsPtr& params, const AgentId& agent_id) :
   EvaluatorDrivableArea(agent_id), 
   EvaluatorSafeDistBase(agent_id),
   lateral_safety_dist_(params->GetReal("EvaluatorStaticSafeDist::LateralSafeDist",
        "Minimum required lateral distance", 0.5f)),
   longitudinal_safety_dist_(params->GetReal("EvaluatorStaticSafeDist::LongitudinalSafeDist",
         "Minimum required longitudinal distance", 0.5f)) {}
        
bool EvaluatorSafeDistDrivableArea::CheckSafeDistance(const world::ObservedWorld& observed_world) {
  return !boost::get<bool>(EvaluatorDrivableArea::Evaluate(observed_world));
}

EvaluationReturn EvaluatorSafeDistDrivableArea::Evaluate(const world::World& world) {
  return EvaluatorSafeDistBase::Evaluate(world);
}

EvaluationReturn EvaluatorSafeDistDrivableArea::Evaluate(const world::ObservedWorld& observed_world) {
  return EvaluatorSafeDistBase::Evaluate(observed_world);
}

bark::geometry::Polygon EvaluatorSafeDistDrivableArea::GetCollisionShape(const AgentPtr& checked_agent) const {
auto ego_state = checked_agent->GetCurrentState();
    // Evaluation assumes that - at zero orientation - shape of ego agent is oriented such that lateral 
    // coordinate is y and longitudinal coordinate is x
    bark::geometry::Pose agent_pose(ego_state(StateDefinition::X_POSITION),
                    ego_state(StateDefinition::Y_POSITION),
                    ego_state(StateDefinition::THETA_POSITION));
    const auto checked_agent_scaled_shape = std::dynamic_pointer_cast<Polygon>(
          checked_agent->GetShape().Scale(longitudinal_safety_dist_, lateral_safety_dist_)->Transform(agent_pose));
    return *checked_agent_scaled_shape;
}


}  // namespace evaluation
}  // namespace world
}  // namespace bark
