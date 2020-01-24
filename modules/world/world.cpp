// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <string>
#include "modules/world/world.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {

World::World(commons::Params* params) :
  commons::BaseType(params),
  map_(),
  agents_(),
  world_time_(0.0),
  remove_agents_(
    params->get_bool("World::remove_agents_out_of_map",
      "Whether agents should be removed outside the bounding box.",
      false)) {}

World::World(const std::shared_ptr<World>& world)  :
  commons::BaseType(world->get_params()),
  map_(world->GetMap()),
  agents_(world->GetAgents()),
  objects_(world->GetObjects()),
  evaluators_(world->GetEvaluators()),
  world_time_(world->GetWorldTime()),
  remove_agents_(world->GetRemoveAgents()),
  rtree_agents_(world->rtree_agents_) {}

void World::AddAgent(const objects::AgentPtr& agent) {
  agents_[agent->agent_id_] = agent;
}

void World::AddObject(const objects::ObjectPtr& object) {
  objects_[object->agent_id_] = object;
}

void World::AddEvaluator(const std::string& name,
                          const EvaluatorPtr& evaluator) {
  evaluators_[name] = evaluator;
}

void World::DoPlanning(const float& delta_time) {
  UpdateAgentRTree();
  WorldPtr current_world(this->Clone());

  // Behavioral and execution planning
  for (auto agent : agents_) {
    //! clone current world
    ObservedWorld observed_world(current_world,
                                  agent.first);
    agent.second->BehaviorPlan(delta_time, observed_world);
    agent.second->ExecutionPlan(delta_time);
  }
}

void World::DoExecution(const float& delta_time) {
  world_time_ += delta_time;
  // Execute motion
  for (auto agent : agents_) {
      agent.second->Execute(world_time_);
  }
  if (remove_agents_) {
    RemoveOutOfMapAgents();
  }
}

WorldPtr World::WorldExecutionAtTime(const float& execution_time) const {
  WorldPtr current_world_state(this->Clone());
  for (auto agent : current_world_state->GetAgents()) {
      agent.second->Execute(execution_time);
  }
  return current_world_state;
}

EvaluationMap World::Evaluate() const {
  EvaluationMap evaluation_results;
  for (auto const& evaluator : evaluators_) {
      evaluation_results[evaluator.first] = evaluator.second->Evaluate(*this);
    }
  return evaluation_results;
}


void World::Step(const float& delta_time) {
  DoPlanning(delta_time);
  DoExecution(delta_time);
}

std::vector<ObservedWorld> World::Observe(
  const std::vector<AgentId>& agent_ids) {
  WorldPtr current_world_state(this->Clone());
  std::vector<ObservedWorld> observed_worlds;
  for (auto agent_id : agent_ids) {
    if (agents_.find(agent_id) == agents_.end()) {
      LOG(ERROR) << "Invalid agent id " <<
                agent_id << ". Skipping ...." << std::endl;
      continue;
    }
    ObservedWorld observed_world(current_world_state,
                                  agent_id);
    observed_worlds.push_back(observed_world);
  }
  return observed_worlds;
}

void World::UpdateAgentRTree() {
  rtree_agents_.clear();
  for (auto &agent : agents_) {
    auto obj = agent.second->GetPolygonFromState(
      agent.second->GetCurrentState()).obj_;
    rtree_agent_model box;
    boost::geometry::envelope(obj, box);
    boost::geometry::correct(box);
    rtree_agents_.insert(std::make_pair(box, agent.first));
  }
}

void World::RemoveOutOfMapAgents() {
  std::vector<rtree_agent_value> query_results;
  auto bounding_box = this->bounding_box();
  boost::geometry::model::box<modules::geometry::Point2d>
         query_box(bounding_box.first, bounding_box.second);

  rtree_agents_.query(!boost::geometry::index::within(query_box),
            std::back_inserter(query_results));
  for (auto &result_pair : query_results) {
    agents_.erase(result_pair.second);
  }
  UpdateAgentRTree();
}

AgentMap World::GetNearestAgents(const modules::geometry::Point2d& position,
                                 const unsigned int& num_agents) const {
  std::vector<rtree_agent_value> results_n;

  rtree_agents_.query(boost::geometry::index::nearest(position, num_agents),
            std::back_inserter(results_n));

  AgentMap nearest_agents;
  for (auto &result_pair : results_n) {
    nearest_agents[result_pair.second] =  GetAgents()[result_pair.second];
  }
  return nearest_agents;
}

AgentMap World::GetAgentsIntersectingPolygon(
  const modules::geometry::Polygon& polygon) const {
  std::vector<rtree_agent_value> query_results;
  auto bounding_box = polygon.bounding_box();
  boost::geometry::model::box<modules::geometry::Point2d>
         query_box(bounding_box.first, bounding_box.second);

  rtree_agents_.query(boost::geometry::index::intersects(query_box),
            std::back_inserter(query_results));

  AgentMap intersecting_agents;
  for (auto &result_pair : query_results) {
    auto agent = GetAgents()[result_pair.second];
    if (modules::geometry::Collide(agent->GetPolygonFromState(
      agent->GetCurrentState()), polygon)) {
      intersecting_agents[result_pair.second] = agent;
    }
  }
  return intersecting_agents;
}


}  // namespace world
}  // namespace modules
