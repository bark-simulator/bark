// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_WORLD_HPP_
#define MODULES_WORLD_WORLD_HPP_

#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/geometry/index/rtree.hpp>
#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/map/roadgraph.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/objects/object.hpp"
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/commons/transformation/frenet.hpp"

namespace modules {
namespace world {

using world::evaluation::EvaluatorPtr;
using world::objects::Agent;
using world::objects::AgentId;
using world::objects::AgentPtr;
using world::objects::ObjectPtr;
using world::map::LaneCorridorPtr;
using world::map::MapInterfacePtr;
using modules::commons::transformation::FrenetPosition;
using models::behavior::StateActionPair;

typedef std::map<AgentId, AgentPtr> AgentMap;
typedef std::map<AgentId, ObjectPtr> ObjectMap;
typedef std::map<std::string, modules::world::evaluation::EvaluationReturn>
  EvaluationMap;
typedef std::map<AgentId, models::dynamic::State> AgentStateMap;
typedef std::unordered_map<AgentId, models::dynamic::Trajectory> AgentTrajectoryMap;

using rtree_agent_model =
  boost::geometry::model::box<modules::geometry::Point2d>;
using rtree_agent_id = AgentId;
using rtree_agent_value = std::pair<rtree_agent_model, rtree_agent_id>;
using AgentRTree =
  boost::geometry::index::rtree<rtree_agent_value,
                                boost::geometry::index::linear<16, 4> >;

typedef std::pair<AgentPtr, FrenetPosition> AgentFrenetPair;

struct FrontRearAgents {
  AgentFrenetPair front;
  AgentFrenetPair rear;
};

class World : public commons::BaseType {
 public:
  explicit World(const commons::ParamsPtr& params);
  explicit World(const std::shared_ptr<World>& world);
  virtual ~World() {}

  /**
   * @brief  Steps the BARK world
   * @param  delta_time: world step time
   */
  void Step(const float& delta_time);

  /**
   * @brief Calls the behavior and execution model of the agents
   * @param  delta_time: minimum planning time
   */
  void PlanAgents(const float& delta_time);

  /**
   * @brief  Updates the agent states
   */
  void Execute(const float& world_time);

  /**
   * @brief Get world for a specific time
   * @param  execution_time: world_time
   */
  std::shared_ptr<World> GetWorldAtTime(
    const float& world_time) const;

  /**
   * @brief  calls all added evaluators of the world
   */
  virtual EvaluationMap Evaluate() const;

  /**
   * @brief  Generates and ObservedWorld for the specified agents
   */
  std::vector<ObservedWorld> Observe(const std::vector<AgentId>& agent_ids);

  /**
   * @brief  Updates the agent r-tree
   */
  void UpdateAgentRTree();

  /**
   * @brief  Removes invalid agents from the world
   */
  void RemoveInvalidAgents();

  //! Getter
  double GetWorldTime() const { return world_time_; }
  void SetWorldTime(const double& world_time) { world_time_ = world_time; }
  world::map::MapInterfacePtr GetMap() const { return map_; }
  AgentMap GetAgents() const { return agents_; }
  AgentMap GetValidAgents() const;
  AgentPtr GetAgent(AgentId id) const {
    auto agent_it = agents_.find(id);
    if (agent_it != agents_.end()) {
      return agents_.at(id);
    } else {
      return AgentPtr(nullptr);
    }
  }
  ObjectMap GetObjects() const { return objects_; }
  std::map<std::string, EvaluatorPtr> GetEvaluators() const {
    return evaluators_;
  }

  bool GetRemoveAgents() const { return remove_agents_; }

  AgentMap GetNearestAgents(const modules::geometry::Point2d& position,
                            const unsigned int& num_agents) const;

  AgentMap GetAgentsIntersectingPolygon(
      const modules::geometry::Polygon& polygon) const;

  FrontRearAgents GetAgentFrontRearForId(
      const AgentId& agent_id, const LaneCorridorPtr& lane_corridor) const;

  //! Setter
  void SetMap(const world::map::MapInterfacePtr& map) { map_ = map; }

  std::pair<modules::geometry::Point2d, modules::geometry::Point2d>
  BoundingBox() const {
    return map_->BoundingBox();
  }

  void AddAgent(const AgentPtr& agent);

  void AddObject(const ObjectPtr& object);

  void AddEvaluator(const std::string& name, const EvaluatorPtr& evaluator);

  //! Functions
  void ClearEvaluators() { evaluators_.clear(); }
  void ClearAgents() { agents_.clear(); }
  void ClearObjects() { objects_.clear(); }
  void ClearAll() {
    ClearAgents();
    ClearObjects();
    ClearEvaluators();
  }

  virtual std::shared_ptr<World> Clone() const;

 private:
  MapInterfacePtr map_;
  AgentMap agents_;
  ObjectMap objects_;
  std::map<std::string, EvaluatorPtr> evaluators_;
  double world_time_;
  AgentRTree rtree_agents_;
  bool remove_agents_;
};

typedef std::shared_ptr<world::World> WorldPtr;

inline WorldPtr World::Clone() const {
  WorldPtr new_world = std::make_shared<World>(*this);
  new_world->ClearAll();
  for (auto agent = agents_.begin(); agent != agents_.end(); ++agent) {
    new_world->AddAgent(
        std::dynamic_pointer_cast<Agent>(agent->second->Clone()));
  }
  for (auto object = objects_.begin(); object != objects_.end(); ++object) {
    new_world->AddObject(object->second->Clone());
  }
  for (const auto& evaluator : evaluators_) {
    new_world->AddEvaluator(evaluator.first, evaluator.second);
  }
  return new_world;
}

}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_WORLD_HPP_
