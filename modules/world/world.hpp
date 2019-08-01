// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_WORLD_HPP_
#define MODULES_WORLD_WORLD_HPP_

#include <unordered_map>

#include "modules/world/opendrive/opendrive.hpp"
#include <boost/geometry/index/rtree.hpp>
#include "modules/world/map/roadgraph.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/objects/object.hpp"
#include "modules/world/evaluation/base_evaluator.hpp"

namespace modules {
namespace world {

using world::objects::AgentId;
using world::objects::AgentPtr;
using world::objects::ObjectPtr;
using world::evaluation::EvaluatorPtr;

typedef std::unordered_map<AgentId, AgentPtr> AgentMap;
typedef std::unordered_map<AgentId, ObjectPtr> ObjectMap;

using rtree_agent_model = boost::geometry::model::box<modules::geometry::Point2d>;
using rtree_agent_id = AgentId;
using rtree_agent_value = std::pair<rtree_agent_model, rtree_agent_id>;
using rtree_agent = boost::geometry::index::rtree<rtree_agent_value,
                   boost::geometry::index::linear<16, 4> >;

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
  std::map<std::string, EvaluatorPtr> get_evaluators() const { return evaluators_;}

  void set_map(const world::map::MapInterfacePtr& map) { map_ = map;}

  std::pair<modules::geometry::Point2d, modules::geometry::Point2d> bounding_box() const { return map_->BoundingBox();}
  
  void add_agent(const AgentPtr& agent);
  void add_object(const ObjectPtr& agent);

  void add_evaluator(const std::string& name, const EvaluatorPtr& evaluator); 

  void clear_agents() { agents_.clear(); }
  void clear_objects() { objects_.clear(); }
  void clear_all()  {
    clear_agents();
    clear_objects();
  }

  typedef std::map<std::string, modules::world::evaluation::EvaluationReturn> EvaluationMap ;
  EvaluationMap Evaluate() const;

  bool Valid() const;
  std::vector<ObservedWorld> Observe(const std::vector<AgentId>& agent_ids);
  void Step(const float& delta_time);
  void UpdateHorizonDrivingCorridors();
  void MoveAgents(const float& delta_time);

  void UpdateAgentRTree();
  AgentMap GetNearestAgents(const modules::geometry::Point2d& position, const unsigned int& num_agents) const; 
  AgentMap GetAgentsIntersectingPolygon(const modules::geometry::Polygon& polygon) const;

  virtual World *Clone() const;
  std::shared_ptr<World> WorldExecutionAtTime(const float& execution_time) const;

 private:
  world::map::MapInterfacePtr map_;
  AgentMap agents_;
  ObjectMap objects_;
  std::map<std::string, EvaluatorPtr> evaluators_;
  double world_time_;
  rtree_agent rtree_agents_;
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
