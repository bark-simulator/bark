// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_OBSERVED_WORLD_HPP_
#define MODULES_WORLD_OBSERVED_WORLD_HPP_

#include <unordered_map>
#include <utility>
#include "modules/geometry/geometry.hpp"
#include "modules/world/world.hpp"
#include "modules/world/map/frenet.hpp"
#include "modules/world/prediction/prediction_settings.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"

namespace modules {
namespace world {

using world::objects::AgentId;
using world::objects::AgentPtr;
using world::objects::Agent;
using world::map::MapInterfacePtr;
using world::map::RoadCorridorPtr;
using modules::geometry::Point2d;
using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition::X_POSITION;
using modules::models::dynamic::StateDefinition::Y_POSITION;
using modules::models::behavior::BehaviorModel;
using modules::models::behavior::BehaviorModelPtr;
using modules::models::behavior::DiscreteAction;
using modules::world::prediction::PredictionSettings;


class ObservedWorld : public World {
 public:
    ObservedWorld(const WorldPtr& world,
                  const AgentId& ego_agent_id) :
      World(world),
      ego_agent_id_(ego_agent_id) {}

    ~ObservedWorld() {}

    std::pair<AgentPtr, modules::world::map::Frenet> GetAgentInFront() const;
    virtual double GetWorldTime() const { return World::GetWorldTime(); }

    const RoadCorridorPtr GetRoadCorridor() const {
      return ObservedWorld::GetEgoAgent()->GetRoadCorridor();
    }

    std::shared_ptr<const Agent> GetEgoAgent() const {
      return World::GetAgent(ego_agent_id_);
    }

    AgentMap GetOtherAgents() const {
      auto tmp_map = World::GetAgents();
      tmp_map.erase(ego_agent_id_);
      return tmp_map;
    }

    const std::shared_ptr<BehaviorModel> GetEgoBehaviorModel() const {
      return World::GetAgents()[ego_agent_id_]->GetBehaviorModel();
    }

    void SetEgoBehaviorModel(const BehaviorModelPtr& behavior_model) const {
      return World::GetAgents()[ego_agent_id_]->SetBehaviorModel(
        behavior_model);
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

    WorldPtr Predict(float time_span,
                     const DiscreteAction& ego_action) const;
    WorldPtr Predict(float time_span) const;

    virtual WorldPtr Clone() const {
      WorldPtr world_clone(World::Clone());
      std::shared_ptr<ObservedWorld> observed_world =
        std::make_shared<ObservedWorld>(world_clone, this->ego_agent_id_);
      return std::dynamic_pointer_cast<World>(observed_world);
    }

 private:
    AgentId ego_agent_id_;
};

typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;

}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OBSERVED_WORLD_HPP_
