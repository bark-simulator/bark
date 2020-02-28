// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_OBSERVED_WORLD_HPP_
#define MODULES_WORLD_OBSERVED_WORLD_HPP_

#include <unordered_map>
#include <utility>
#include "modules/geometry/geometry.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/world/prediction/prediction_settings.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace world {

using modules::geometry::Point2d;
using modules::models::behavior::BehaviorModel;
using modules::models::behavior::BehaviorModelPtr;
using modules::models::behavior::DiscreteAction;
using modules::models::behavior::Action;
using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition::X_POSITION;
using modules::models::dynamic::StateDefinition::Y_POSITION;
using modules::world::prediction::PredictionSettings;
using world::map::MapInterfacePtr;
using world::map::RoadCorridorPtr;
using world::objects::Agent;
using world::objects::AgentId;
using world::objects::AgentPtr;

class ObservedWorld : public World {
 public:
  ObservedWorld(const WorldPtr& world, const AgentId& ego_agent_id)
      : World(world), ego_agent_id_(ego_agent_id) {}

  ~ObservedWorld() {}

  FrontRearAgents GetAgentFrontRear() const;

  AgentFrenetPair GetAgentInFront() const;

  AgentFrenetPair GetAgentBehind() const;

  virtual double GetWorldTime() const { return World::GetWorldTime(); }

  const RoadCorridorPtr GetRoadCorridor() const {
    return ObservedWorld::GetEgoAgent()->GetRoadCorridor();
  }

  const LaneCorridorPtr GetLaneCorridor() const;

  std::shared_ptr<const Agent> GetEgoAgent() const {
    return World::GetAgent(ego_agent_id_);
  }

  AgentId GetEgoAgentId() const { return ego_agent_id_; }

  AgentMap GetOtherAgents() const {
    auto tmp_map = World::GetAgents();
    tmp_map.erase(ego_agent_id_);
    return tmp_map;
  }

  const std::shared_ptr<BehaviorModel> GetEgoBehaviorModel() const {
    return World::GetAgents()[ego_agent_id_]->GetBehaviorModel();
  }

  void SetEgoBehaviorModel(const BehaviorModelPtr& behavior_model) const {
    return World::GetAgents()[ego_agent_id_]->SetBehaviorModel(behavior_model);
  }

  void SetBehaviorModel(const AgentId& agent_id,
                        const BehaviorModelPtr& behavior_model) const {
    return World::GetAgents()[agent_id]->SetBehaviorModel(behavior_model);
  }

  const MapInterfacePtr GetMap() const { return World::GetMap(); }

  virtual State CurrentEgoState() const {
    return World::GetAgents()[ego_agent_id_]->GetCurrentState();
  }

  Point2d CurrentEgoPosition() const {
    return World::GetAgents()[ego_agent_id_]->GetCurrentPosition();
  }

  void SetupPrediction(const PredictionSettings& settings);

  ObservedWorldPtr Predict(float time_span,
                           const DiscreteAction& ego_action) const;
  ObservedWorldPtr Predict(float time_span,
                           const std::unordered_map<AgentId, DiscreteAction>&
                               agent_action_map) const;

  ObservedWorldPtr Predict(float time_span) const;


  template<class Behavior, class EgoBehavior>
  ObservedWorldPtr Predict(float time_span,
                           const Action& ego_action) const {
    std::shared_ptr<ObservedWorld> next_obs_world =
      std::dynamic_pointer_cast<ObservedWorld>(ObservedWorld::Clone());
    // for all other agents set Behavior
    for (auto& agent : next_obs_world->GetOtherAgents()) {
      agent.second->SetBehaviorModel(
        std::make_shared<Behavior>(
          agent.second->GetBehaviorModel()->GetParams()));
    }
    std::shared_ptr<EgoBehavior> ego_behavior_model =
      std::dynamic_pointer_cast<EgoBehavior>(
        next_obs_world->GetEgoBehaviorModel());
    ego_behavior_model->SetLastAction(ego_action);
    next_obs_world->Step(time_span);
    return next_obs_world;
  }

  virtual WorldPtr Clone() const {
    WorldPtr world_clone(World::Clone());
    std::shared_ptr<ObservedWorld> observed_world =
        std::make_shared<ObservedWorld>(world_clone, this->ego_agent_id_);
    return std::dynamic_pointer_cast<World>(observed_world);
  }
  virtual EvaluationMap Evaluate() const;

 private:
  AgentId ego_agent_id_;
};

typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;

}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OBSERVED_WORLD_HPP_
