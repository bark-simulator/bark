// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_WORLD_EGO_HPP_
#define MODULES_WORLD_WORLD_EGO_HPP_

#include <unordered_map>

#include "modules/geometry/geometry.hpp"
#include "modules/world/world.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/world/map/route_generator.hpp"

namespace modules {
namespace world {

using world::objects::AgentId;
using world::objects::AgentPtr;
using world::objects::Agent;
using world::map::RouteGenerator;
using world::map::MapInterfacePtr;
using modules::geometry::Point2d;
using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition::X_POSITION;
using modules::models::dynamic::StateDefinition::Y_POSITION;


class ObservedWorld : protected World {
 public:
    ObservedWorld(const World& world, const AgentId& ego_agent_id) :
      World(world),
      ego_agent_id_(ego_agent_id) {}

    ~ObservedWorld() {}

    double get_world_time() const { return World::get_world_time(); }


    const RouteGenerator& get_local_map() const {
      return *World::get_agent(ego_agent_id_)->get_route_generator();
    }

    std::shared_ptr<const Agent> get_ego_agent() const {
      return World::get_agent(ego_agent_id_);
    }

    const MapInterfacePtr get_map() const { return World::get_map(); }

    State get_ego_state() const {
      return World::get_agents()[ego_agent_id_]->get_current_state();
    }

    Point2d get_ego_point() const {
      State ego_state = get_ego_state();
      return Point2d(ego_state(X_POSITION), ego_state(Y_POSITION));
    }

 private:
    AgentId ego_agent_id_;
};


}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_WORLD_HPP_