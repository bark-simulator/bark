// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_WORLD_HPP_
#define BARK_WORLD_WORLD_HPP_

#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>    // std::count

#include <boost/geometry/index/rtree.hpp>
#include "bark/commons/transformation/frenet.hpp"
#include "bark/commons/transformation/frenet_state.hpp"
#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/models/observer/observer_model.hpp"
#include "bark/world/map/roadgraph.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/objects/object.hpp"
#include "bark/world/renderer/renderer.hpp"
#include "bark/world/opendrive/opendrive.hpp"

namespace bark {
namespace world {

using bark::commons::transformation::FrenetPosition;
using bark::commons::transformation::FrenetState;
using bark::commons::transformation::FrenetStateDifference;
using models::behavior::StateActionPair;
using world::evaluation::EvaluatorPtr;
using world::map::LaneCorridorPtr;
using world::map::MapInterfacePtr;
using world::objects::Agent;
using world::objects::AgentId;
using world::objects::AgentPtr;
using world::objects::ObjectPtr;
using world::renderer::RendererPtr;
using bark::models::observer::ObserverModelPtr;

typedef std::map<AgentId, AgentPtr> AgentMap;
typedef std::map<AgentId, ObjectPtr> ObjectMap;
typedef std::map<std::string, bark::world::evaluation::EvaluationReturn>
    EvaluationMap;
typedef std::map<AgentId, models::dynamic::State> AgentStateMap;
typedef std::unordered_map<AgentId, models::dynamic::Trajectory>
    AgentTrajectoryMap;

using rtree_agent_model = boost::geometry::model::box<bark::geometry::Point2d>;
using rtree_agent_id = AgentId;
using rtree_agent_value = std::pair<rtree_agent_model, rtree_agent_id>;
using AgentRTree =
    boost::geometry::index::rtree<rtree_agent_value,
                                  boost::geometry::index::linear<16, 4> >;

typedef std::pair<AgentPtr, FrenetStateDifference> AgentFrenetPair;

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
  void Step(const double& delta_time);

  /**
   * @brief Calls the behavior and execution model of the agents
   * @param  delta_time: minimum planning time
   */
  void PlanAgents(const double& delta_time);

  /**
   * @brief Calls the behavior and execution model of the agents and step world
   * @param  delta_time: minimum planning time
   * @param agent_ids: the specific agents to be planned and executed
   */
  void PlanAndExecuteAgentsWithID(const double& delta_time, const std::vector<AgentId>& agent_ids);

  /**
   * @brief  Updates the agent states
   */
  void Execute(const double& delta_time);

  /**
   * @brief Get world for a specific time
   * @param  execution_time: world_time
   */
  std::shared_ptr<World> GetWorldAtTime(const double& world_time) const;

  /**
   * @brief  calls all added evaluators of the world
   */
  virtual EvaluationMap Evaluate() const;

  /**
   * @brief  Generates and ObservedWorld for the specified agents
   */
  std::vector<ObservedWorld> Observe(const std::vector<AgentId>& agent_ids) const;

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
  RendererPtr GetRenderer() const { return renderer_; }
  void SetWorldTime(const double& world_time) { world_time_ = world_time; }
  world::map::MapInterfacePtr GetMap() const { return map_; }
  virtual AgentMap GetAgents() const { return agents_; }
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
  ObserverModelPtr GetObserverModel() const {
    return observer_;
  }
  
  bool GetRemoveAgents() { return remove_agents_; }

  double GetLateralDifferenceThreshold() const { return lateral_difference_threshold_; }

  unsigned GetMaxAgentsFrontRear() const { return max_agents_front_rear_; }

  void SetRemoveAgents(const bool& remove_agents) {
    remove_agents_ = remove_agents;
  }

  void SetLateralDifferenceThreshold(double lateral_difference_threshold) {
    lateral_difference_threshold_ = lateral_difference_threshold;
  } 

  AgentMap GetNearestAgents(const bark::geometry::Point2d& position,
                            const unsigned int& num_agents) const;

  AgentMap GetAgentsIntersectingPolygon(
      const bark::geometry::Polygon& polygon) const;

  /**
   * @brief Get the front and rear agent for a given agent
   *
   * @param agent_id agent id for which to calculate front&rear agent
   * @param lane_corridor lane corridor in which to calculate front&rear agent
   * @param lateral_difference_threshold Lane Width Fraction for maximum allowed lateral
   * offset in, should be larger than 0. value of 2 means that all agents
   * intersecting with lane will be considered
   * @param angle_difference_threshold what is maximul frenet angle to other vehicles
   *                                  to consider it as front
   * @param must_be_in_corridor check only if in driving, and neglect threshold based detection
   * @param lon_diff_treshold longitudinal difference from when a vehicle is considered as front,
   *                          otherwise considered as rear
   * @return FrontRearAgents
   */
  FrontRearAgents GetAgentFrontRearForId(const AgentId& agent_id,
                                         const LaneCorridorPtr& lane_corridor,
                                         double lateral_difference_threshold,
                                         double angle_difference_threshold = bark::geometry::B_PI,
                                         bool must_be_in_corridor = false,
                                         double longitudinal_difference_treshold = 0.0) const;

  //! Setter
  void SetMap(const world::map::MapInterfacePtr& map) { map_ = map; }
  void SetObserverModel(const ObserverModelPtr& observer) {
    observer_ = observer;
  }

  std::pair<bark::geometry::Point2d, bark::geometry::Point2d> BoundingBox()
      const {
    return map_->BoundingBox();
  }
  void SetRenderer(const RendererPtr& renderer) {
    renderer_ = renderer;
  }

  void AddAgent(const AgentPtr& agent);

  void RemoveAgentById(AgentId agent_id);

  void AddObject(const ObjectPtr& object);

  void AddEvaluator(const std::string& name, const EvaluatorPtr& evaluator);

  void UpdateAgentStateFromExtern(const float& delta_time,
                                  const AgentStateMap& state_map,
                                  const std::vector<AgentId>& exclude_agent_ids);
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
  RendererPtr renderer_;
  std::map<std::string, EvaluatorPtr> evaluators_;
  ObserverModelPtr observer_;
  double world_time_;
  AgentRTree rtree_agents_;
  bool remove_agents_;
  double lateral_difference_threshold_;
  unsigned max_agents_front_rear_;
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
}  // namespace bark

#endif  // BARK_WORLD_WORLD_HPP_
