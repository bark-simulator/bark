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
#include "modules/world/prediction/prediction_settings.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/world/map/local_map.hpp"
#include "modules/world/map/driving_corridor.hpp"

namespace modules {
namespace world {

using world::objects::AgentId;
using world::objects::AgentPtr;
using world::objects::Agent;
using world::map::LocalMap;
using world::map::LocalMapPtr;
using world::map::MapInterfacePtr;
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

    virtual double get_world_time() const { return World::get_world_time(); }


    const LocalMapPtr get_local_map() const {
      return ObservedWorld::get_ego_agent()->get_local_map();
    }

    std::shared_ptr<const Agent> get_ego_agent() const {
      return World::get_agent(ego_agent_id_);
    }

    AgentMap get_other_agents() const {
      auto tmp_map = World::get_agents();
      tmp_map.erase(ego_agent_id_);
      return tmp_map;
    }

    const std::shared_ptr<BehaviorModel> get_ego_behavior_model() const {
      return World::get_agents()[ego_agent_id_]->get_behavior_model();
    }

    void set_ego_behavior_model(const BehaviorModelPtr& behavior_model) const {
      return World::get_agents()[ego_agent_id_]->set_behavior_model(
        behavior_model);
    }

    void set_behavior_model(const AgentId& agent_id,
                            const BehaviorModelPtr& behavior_model) const {
      return World::get_agents()[agent_id]->set_behavior_model(behavior_model);
    }

    const MapInterfacePtr get_map() const { return World::get_map(); }

    virtual State current_ego_state() const {
      return World::get_agents()[ego_agent_id_]->get_current_state();
    }

    Point2d current_ego_position() const {
      return World::get_agents()[ego_agent_id_]->get_current_position();
    }

    std::pair<AgentPtr, modules::world::map::Frenet> get_agent_in_front() const;

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