// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <csignal>
#include <string>

#include "bark/commons/util/segfault_handler.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace world {

using models::behavior::BehaviorStatus;
using models::execution::ExecutionStatus;

World::World(const commons::ParamsPtr& params)
    : commons::BaseType(params),
      map_(),
      agents_(),
      world_time_(0.0),
      remove_agents_(params->GetBool(
          "World::remove_agents_out_of_map",
          "Whether agents should be removed outside the bounding box.",
          false)) {
  //! segfault handler
  std::signal(SIGSEGV, bark::commons::SegfaultHandler);
}

World::World(const std::shared_ptr<World>& world)
    : commons::BaseType(world->GetParams()),
      map_(world->GetMap()),
      agents_(world->GetAgents()),
      objects_(world->GetObjects()),
      evaluators_(world->GetEvaluators()),
      world_time_(world->GetWorldTime()),
      remove_agents_(world->GetRemoveAgents()),
      rtree_agents_(world->rtree_agents_) {
  //! segfault handler
  std::signal(SIGSEGV, bark::commons::SegfaultHandler);
}

void World::Step(const float& delta_time) {
  const float inc_world_time = world_time_ + delta_time;
  PlanAgents(delta_time);
  Execute(inc_world_time);
  world_time_ = inc_world_time;
}

void World::PlanAgents(const float& delta_time) {
  UpdateAgentRTree();
  WorldPtr current_world(this->Clone());
  const float inc_world_time = world_time_ + delta_time;
  for (auto agent : agents_) {
    ObservedWorld observed_world(current_world, agent.first);
    agent.second->PlanBehavior(delta_time, observed_world);
    if (agent.second->GetBehaviorStatus() == BehaviorStatus::VALID)
      agent.second->PlanExecution(inc_world_time);
  }
}

void World::Execute(const float& world_time) {
  using models::dynamic::StateDefinition::TIME_POSITION;
  for (auto agent : agents_) {
    if (agent.second->GetBehaviorStatus() == BehaviorStatus::VALID &&
        agent.second->GetExecutionStatus() == ExecutionStatus::VALID) {
      agent.second->UpdateStateAction();
      // make sure all agents have the same world time
      // otherwise the simulation is not correct
      const auto& agent_state = agent.second->GetCurrentState();
      BARK_EXPECT_TRUE(fabs(agent_state(TIME_POSITION) - world_time) < 0.01);
    }
  }
  RemoveInvalidAgents();
}

WorldPtr World::GetWorldAtTime(const float& world_time) const {
  WorldPtr current_world_state(this->Clone());
  for (auto agent : current_world_state->GetAgents()) {
    if (agent.second->GetBehaviorStatus() == BehaviorStatus::VALID)
      agent.second->PlanExecution(world_time);
    agent.second->UpdateStateAction();
  }
  return current_world_state;
}

AgentMap World::GetValidAgents() const {
  AgentMap agents_valid(agents_);
  AgentMap::iterator it;
  for (it = agents_valid.begin(); it != agents_valid.end();) {
    if ((*it).second->GetBehaviorStatus() != BehaviorStatus::VALID) {
      agents_valid.erase(it++);
    } else {
      ++it;
    }
  }
  return agents_valid;
}

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

EvaluationMap World::Evaluate() const {
  EvaluationMap evaluation_results;
  for (auto const& evaluator : evaluators_) {
    evaluation_results[evaluator.first] = evaluator.second->Evaluate(*this);
  }
  return evaluation_results;
}

std::vector<ObservedWorld> World::Observe(
    const std::vector<AgentId>& agent_ids) {
  WorldPtr current_world_state(this->Clone());
  std::vector<ObservedWorld> observed_worlds;
  for (auto agent_id : agent_ids) {
    if (agents_.find(agent_id) == agents_.end()) {
      LOG(ERROR) << "Invalid agent id " << agent_id << ". Skipping ...."
                 << std::endl;
      continue;
    }
    ObservedWorld observed_world(current_world_state, agent_id);
    observed_worlds.push_back(observed_world);
  }
  return observed_worlds;
}

void World::UpdateAgentRTree() {
  rtree_agents_.clear();
  for (auto& agent : agents_) {
    auto obj =
        agent.second->GetPolygonFromState(agent.second->GetCurrentState()).obj_;
    rtree_agent_model box;
    boost::geometry::envelope(obj, box);
    boost::geometry::correct(box);
    rtree_agents_.insert(std::make_pair(box, agent.first));
  }
}

void World::RemoveInvalidAgents() {
  if (remove_agents_) {
    std::vector<rtree_agent_value> query_results;
    auto bounding_box = this->BoundingBox();
    boost::geometry::model::box<bark::geometry::Point2d> query_box(
        bounding_box.first, bounding_box.second);

    rtree_agents_.query(!boost::geometry::index::within(query_box),
                        std::back_inserter(query_results));
    for (auto& result_pair : query_results) {
      agents_.erase(result_pair.second);
    }
  }

  for (auto& agent : agents_) {
    if (agent.second->GetBehaviorStatus() == BehaviorStatus::EXPIRED) {
      agents_.erase(agent.first);
    }
  }

  UpdateAgentRTree();
}

AgentMap World::GetNearestAgents(const bark::geometry::Point2d& position,
                                 const unsigned int& num_agents) const {
  std::vector<rtree_agent_value> results_n;

  rtree_agents_.query(boost::geometry::index::nearest(position, num_agents),
                      std::back_inserter(results_n));

  AgentMap nearest_agents;
  for (auto& result_pair : results_n) {
    nearest_agents[result_pair.second] = GetAgent(result_pair.second);
  }
  return nearest_agents;
}

AgentMap World::GetAgentsIntersectingPolygon(
    const bark::geometry::Polygon& polygon) const {
  std::vector<rtree_agent_value> query_results;
  auto bounding_box = polygon.BoundingBox();
  boost::geometry::model::box<bark::geometry::Point2d> query_box(
      bounding_box.first, bounding_box.second);

  rtree_agents_.query(boost::geometry::index::intersects(query_box),
                      std::back_inserter(query_results));

  AgentMap intersecting_agents;
  for (auto& result_pair : query_results) {
    auto agent = GetAgent(result_pair.second);
    if (bark::geometry::Collide(
            agent->GetPolygonFromState(agent->GetCurrentState()), polygon) &&
        agent->GetBehaviorStatus() == BehaviorStatus::VALID) {
      intersecting_agents[result_pair.second] = agent;
    }
  }
  return intersecting_agents;
}

FrontRearAgents World::GetAgentFrontRearForId(
    const AgentId& agent_id, const LaneCorridorPtr& lane_corridor) const {
  using bark::geometry::Line;
  using bark::geometry::Polygon;

  FrontRearAgents fr_agents;
  Point2d ego_position = World::GetAgent(agent_id)->GetCurrentPosition();

  const Polygon& corridor_polygon = lane_corridor->GetMergedPolygon();
  const Line& center_line = lane_corridor->GetCenterLine();
  AgentMap intersecting_agents = GetAgentsIntersectingPolygon(corridor_polygon);
  if (intersecting_agents.size() == 0) {
    fr_agents.front = std::make_pair(AgentPtr(nullptr), FrenetPosition(0, 0));
    fr_agents.rear = fr_agents.front;
    return fr_agents;
  }

  FrenetPosition frenet_ego(ego_position, center_line);
  const double numeric_max = std::numeric_limits<double>::max();

  double nearest_lon_front = numeric_max, nearest_lat_front = numeric_max,
         nearest_lon_rear = numeric_max, nearest_lat_rear = numeric_max;

  AgentPtr nearest_agent_front(nullptr);
  AgentPtr nearest_agent_rear(nullptr);

  for (auto it = intersecting_agents.begin(); it != intersecting_agents.end();
       ++it) {
    if (it->second->GetAgentId() == agent_id ||
        it->second->GetBehaviorStatus() != BehaviorStatus::VALID) {
      continue;
    }

    FrenetPosition frenet_other(it->second->GetCurrentPosition(), center_line);
    float width = lane_corridor->GetLaneWidth(it->second->GetCurrentPosition());
    if (std::abs(frenet_other.lat) > width / 2) {
      // agent seems to be not really in same lane
      continue;
    }
    double long_dist = frenet_other.lon - frenet_ego.lon;
    double lat_dist = frenet_other.lat - frenet_ego.lat;

    if (long_dist > 0.0f && long_dist < nearest_lon_front) {
      nearest_lon_front = long_dist;
      nearest_lat_front = lat_dist;
      nearest_agent_front = it->second;
    } else if (long_dist < 0.0f &&
               std::abs(long_dist) < std::abs(nearest_lon_rear)) {
      nearest_lon_rear = long_dist;
      nearest_lat_rear = lat_dist;
      nearest_agent_rear = it->second;
    }
  }

  FrenetPosition frenet_front =
      FrenetPosition(nearest_lon_front, nearest_lat_front);
  fr_agents.front = std::make_pair(nearest_agent_front, frenet_front);
  FrenetPosition frenet_rear =
      FrenetPosition(nearest_lon_rear, nearest_lat_rear);
  fr_agents.rear = std::make_pair(nearest_agent_rear, frenet_rear);

  return fr_agents;
}

void World::RemoveAgentById(AgentId agent_id) {
  size_t erased_elems = agents_.erase(agent_id);
  LOG_IF(ERROR, erased_elems == 0)
      << "Could not remove non-existent agent with Id " << agent_id << " !";
}

}  // namespace world
}  // namespace bark
