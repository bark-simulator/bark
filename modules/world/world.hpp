// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_WORLD_HPP_
#define MODULES_WORLD_WORLD_HPP_

#include <unordered_map>

#include "modules/world/opendrive/opendrive.hpp"
#include "modules/world/map/roadgraph.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/objects/object.hpp"
#include "modules/world/collision/base_collision_checker.hpp"

namespace modules {
namespace world {

using world::objects::AgentId;
using world::objects::AgentPtr;
using world::objects::ObjectPtr;
using world::collision::CollisionCheckerPtr;

typedef std::unordered_map<AgentId, AgentPtr> AgentMap;
typedef std::unordered_map<AgentId, ObjectPtr> ObjectMap;
typedef std::vector<CollisionCheckerPtr> CollisionCheckerVector;

class World : public commons::BaseType {
 public:
  explicit World(commons::Params *params);
  explicit World(const World& world);
  virtual ~World() {}

  //! Getter
  double get_world_time() const { return world_time_; }
  world::map::MapInterfacePtr get_map() const { return map_; }
  AgentMap get_agents() const { return agents_; }
  AgentPtr get_agent(AgentId id) const { return agents_.at(id); }
  ObjectMap get_objects() const { return objects_; }
  CollisionCheckerVector get_collision_checkers() const { return collision_checkers_; }

  void set_map(const world::map::MapInterfacePtr& map) { map_ = map; }
  
  void add_agent(const AgentPtr& agent);
  void add_object(const ObjectPtr& agent);

  void add_collision_checker(const CollisionCheckerPtr& cchecker); //{ collision_checker_ = cchecker; }

  void clear_agents() { agents_.clear(); }
  void clear_objects() { objects_.clear(); }
  void clear_all()  {
    clear_agents();
    clear_objects();
  }

  bool CheckCollision() const;

  bool Valid() const;
  std::vector<ObservedWorld> Observe(const std::vector<AgentId>& agent_ids);
  void Step(const float& delta_time);
  void UpdateHorizonDrivingCorridors();
  void MoveAgents(const float& delta_time);

  virtual World *Clone() const;

 private:
  world::map::MapInterfacePtr map_;
  AgentMap agents_;
  ObjectMap objects_;
  CollisionCheckerVector collision_checkers_;
  double world_time_;
};

typedef std::shared_ptr<world::World> WorldPtr;

inline World *World::Clone() const {
  World *new_world = new World(*this);
  new_world->clear_all();
  for (auto agent = agents_.begin(); agent != agents_.end(); ++agent) {
    new_world->add_agent(AgentPtr(agent->second->Clone()));
  }
  for (auto object = objects_.begin(); object != objects_.end(); ++object) {
    new_world->add_object(ObjectPtr(object->second->Clone()));
  }
  return new_world;
}

}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_WORLD_HPP_
