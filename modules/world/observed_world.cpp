// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <limits>

#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {

using modules::geometry::Point2d;
using modules::models::behavior::BehaviorMotionPrimitives;
using modules::models::behavior::BehaviorMotionPrimitivesPtr;
using modules::models::behavior::BehaviorActionStorePtr;
using modules::models::dynamic::State;
using modules::world::AgentMap;

FrontRearAgents ObservedWorld::GetAgentFrontRear() const {
  const auto& lane_corridor = GetLaneCorridor();
  BARK_EXPECT_TRUE(lane_corridor != nullptr);

  AgentId id = GetEgoAgentId();
  FrontRearAgents fr_agent = GetAgentFrontRearForId(id, lane_corridor);

  return fr_agent;
}

AgentFrenetPair ObservedWorld::GetAgentInFront() const {
  FrontRearAgents fr_agent = GetAgentFrontRear();
  return fr_agent.front;
}

AgentFrenetPair ObservedWorld::GetAgentBehind() const {
  FrontRearAgents fr_agent = GetAgentFrontRear();
  return fr_agent.rear;
}

const LaneCorridorPtr ObservedWorld::GetLaneCorridor() const {
  Point2d ego_pos = CurrentEgoPosition();
  const auto& road_corridor = GetRoadCorridor();
  if (!road_corridor) {
    LOG(ERROR) << "No road corridor found.";
    return nullptr;
  }
  const auto& lane_corridor = road_corridor->GetCurrentLaneCorridor(ego_pos);
  if (!lane_corridor) {
    LOG(ERROR) << "No lane corridor found.";
    return nullptr;
  }
  return lane_corridor;
}

void ObservedWorld::SetupPrediction(const PredictionSettings& settings) {
  settings.ApplySettings(*this);
}

ObservedWorldPtr ObservedWorld::Predict(float time_span) const {
  std::shared_ptr<ObservedWorld> next_world =
      std::dynamic_pointer_cast<ObservedWorld>(ObservedWorld::Clone());
  next_world->Step(time_span);
  return next_world;
}

ObservedWorldPtr ObservedWorld::Predict(
    float time_span, const DiscreteAction& ego_action) const {
  std::shared_ptr<ObservedWorld> next_world =
      std::dynamic_pointer_cast<ObservedWorld>(ObservedWorld::Clone());
  std::shared_ptr<BehaviorMotionPrimitives> ego_behavior_model =
      std::dynamic_pointer_cast<BehaviorMotionPrimitives>(
          next_world->GetEgoBehaviorModel());
  if (ego_behavior_model) {
    ego_behavior_model->ActionToBehavior(ego_action);
  } else {
    LOG(ERROR) << "Currently only BehaviorMotionPrimitive model supported for "
                  "ego prediction, adjust prediction settings.";  // NOLINT
  }
  next_world->Step(time_span);
  return next_world;
}

ObservedWorldPtr ObservedWorld::Predict(
    float time_span,
    const std::unordered_map<AgentId, DiscreteAction>& agent_action_map) const {
  std::shared_ptr<ObservedWorld> next_world =
      std::dynamic_pointer_cast<ObservedWorld>(ObservedWorld::Clone());
  for (const auto& agent_action : agent_action_map) {
    if (!next_world->GetAgent(agent_action.first)) {
      continue;
    }
    std::shared_ptr<BehaviorMotionPrimitives> behavior_model =
        std::dynamic_pointer_cast<BehaviorMotionPrimitives>(
            next_world->GetAgent(agent_action.first)->GetBehaviorModel());
    if (behavior_model) {
      behavior_model->ActionToBehavior(agent_action.second);
    } else {
      LOG(ERROR)
          << "Currently only BehaviorMotionPrimitive model supported for "
             "ego prediction, adjust prediction settings.";  // NOLINT
    }
  }
  next_world->Step(time_span);
  return next_world;
}

// Predict each agent with specific behavior model (Note setup required)
 ObservedWorldPtr ObservedWorld::Predict(float time_span, BehaviorModelPtr ego_behavior_model,
         const std::unordered_map<AgentId, BehaviorModelPtr> other_behaviors) const
 {
  std::shared_ptr<ObservedWorld> next_world =
      std::dynamic_pointer_cast<ObservedWorld>(ObservedWorld::Clone());

  auto ego_agent_ptr = next_world->GetEgoAgent();
  if(ego_agent_ptr) {
      ego_agent_ptr->SetBehaviorModel(ego_behavior_model);
  } else {
    LOG(WARNING) << "Ego Agent not existent in observed world during prediction";
  }
  
  for (const auto& agent_pair : other_behaviors) {
    auto agent_ptr = next_world->GetAgent(agent_pair.first);
    if (!agent_ptr) {
      LOG(WARNING) << "Agent Id" << agent_pair.first 
              <<" not existent in observed world during prediction";
      continue;
    }
    agent_ptr->SetBehaviorModel(agent_pair.second);
  }
  next_world->Step(time_span);
  return next_world;
}

EvaluationMap ObservedWorld::Evaluate() const {
  EvaluationMap evaluation_results;
  for (auto const& evaluator : World::GetEvaluators()) {
    evaluation_results[evaluator.first] = evaluator.second->Evaluate(*this);
  }
  return evaluation_results;
}

}  // namespace world
}  // namespace modules
