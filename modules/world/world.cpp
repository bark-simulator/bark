// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "modules/world/world.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {

World::World(commons::Params* params) :
  commons::BaseType(params),
  map_(),
  agents_(),
  world_time_(0.0) {}

World::World(const World& world)  :
         commons::BaseType(world.get_params()), 
         map_(world.get_map()),
         agents_(world.get_agents()),
         objects_(world.get_objects()),
         world_time_(world.get_world_time()) {}

void World::add_agent(const objects::AgentPtr& agent) {
  agents_[agent->agent_id_] = agent; 
}

void World::add_object(const objects::ObjectPtr& object) {
  objects_[object->agent_id_] = object;
}

void World::UpdateLocalRoutes() {
  for (auto agent : agents_) {
    agent.second->UpdateLocalRoute();
  }
}

void World::MoveAgents(float delta_time) {
  WorldPtr current_world_state(this->Clone());
  for (auto agent : agents_) {
      //! clone current world
      ObservedWorld observed_world(*current_world_state,
                                   agent.first);
      agent.second->Move(delta_time, observed_world);
  }
  world_time_ += delta_time;
}

void World::Step(float delta_time) {
  UpdateLocalRoutes();
  MoveAgents(delta_time);
  // TODO(@fortiss): add post world collision check
}

}  // namespace world
}  // namespace modules
