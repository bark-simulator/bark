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
#include "bark/models/observer/observer_model.hpp"
#include "bark/models/observer/observer_model_none.hpp"
#include "bark/world/renderer/renderer.hpp"

namespace bark {
namespace world {

using models::behavior::BehaviorStatus;
using models::execution::ExecutionStatus;
using bark::models::observer::ObserverModelNone;
using bark::world::renderer::Renderer;

World::World(const commons::ParamsPtr& params)
    : commons::BaseType(params),
      map_(),
      agents_(),
      world_time_(0.0),
      observer_(new ObserverModelNone(params)),
      renderer_(new Renderer()),
      remove_agents_(params->GetBool(
          "World::remove_agents_out_of_map",
          "Whether agents should be removed outside the bounding box.", false)),
      lateral_difference_threshold_(params->GetReal("World::LateralDifferenceThreshold",
          "Lateral distance between shapes of vehicles considering orientation of shapes with respect to center line"
                "for FrontRearAgent Calculation",
          0.0)),
      max_agents_front_rear_(params->GetInt("World::MaxAgentsFrontRear",
          "How many nearest agents are considered to search for front/rear agents",
          4)) {
  //! segfault handler
  std::signal(SIGSEGV, bark::commons::SegfaultHandler);
}

World::World(const std::shared_ptr<World>& world)
    : commons::BaseType(world->GetParams()),
      map_(world->GetMap()),
      agents_(world->GetAgents()),
      objects_(world->GetObjects()),
      evaluators_(world->GetEvaluators()),
      observer_(world->GetObserverModel()),
      world_time_(world->GetWorldTime()),
      renderer_(world->GetRenderer()),
      remove_agents_(world->GetRemoveAgents()),
      lateral_difference_threshold_(world->GetLateralDifferenceThreshold()),
      max_agents_front_rear_(world->GetMaxAgentsFrontRear()),
      rtree_agents_(world->rtree_agents_) {
  //! segfault handler
  std::signal(SIGSEGV, bark::commons::SegfaultHandler);
}

void World::Step(const double& delta_time) {
  PlanAgents(delta_time);
  Execute(delta_time);
}

void World::PlanAgents(const double& delta_time) {
  UpdateAgentRTree();
  WorldPtr current_world(this->Clone());
  const double inc_world_time = world_time_ + delta_time;
  for (auto agent : agents_) {
    if (agent.second->IsValidAtTime(world_time_)) {
      ObservedWorld observed_world = observer_->Observe(
        current_world, agent.first);
      agent.second->SetSensedWorld(std::make_shared<ObservedWorld>(observed_world));
      agent.second->PlanBehavior(delta_time, observed_world);
      if (agent.second->GetBehaviorStatus() == BehaviorStatus::VALID)
        agent.second->PlanExecution(inc_world_time);
    }
  }
}

void World::Execute(const double& delta_time) {
  const double inc_world_time = world_time_ + delta_time;
  using models::dynamic::StateDefinition::TIME_POSITION;
  for (auto agent : agents_) {
    if (agent.second->IsValidAtTime(world_time_) &&
        agent.second->GetBehaviorStatus() == BehaviorStatus::VALID &&
        agent.second->GetExecutionStatus() == ExecutionStatus::VALID) {
      agent.second->UpdateStateAction();
      // make sure all agents have the same world time
      // otherwise the simulation is not correct
      const auto& agent_state = agent.second->GetCurrentState();
      BARK_EXPECT_TRUE(fabs(agent_state(TIME_POSITION) - inc_world_time) <
                       0.01);
    }
  }
  RemoveInvalidAgents();

  world_time_ = inc_world_time;
}

WorldPtr World::GetWorldAtTime(const double& world_time) const {
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
    } else if ((*it).second->IsValidAtTime(world_time_) == false) {
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

std::vector<ObservedWorld> World::Observe (
    const std::vector<AgentId>& agent_ids) const {
  WorldPtr current_world(this->Clone());
  std::vector<ObservedWorld> observed_worlds;
  for (auto agent_id : agent_ids) {
    if (agents_.find(agent_id) == agents_.end()) {
      LOG(ERROR) << "Invalid agent id " << agent_id << ". Skipping ...."
                 << std::endl;
      continue;
    }
    ObservedWorld observed_world = observer_->Observe(
      current_world, agent_id);
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
      AgentPtr agent = GetAgent(result_pair.second);
      if (agent && agent->GetBehaviorStatus() == BehaviorStatus::VALID &&
          agent->IsValidAtTime(world_time_)) {
        agents_.erase(result_pair.second);
      }
    }
  }

  for (auto it = agents_.cbegin(); it != agents_.cend();) {
    if (it->second->GetBehaviorStatus() == BehaviorStatus::EXPIRED) {
      agents_.erase(it++);
    } else {
      ++it;
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
    AgentPtr agent = GetAgent(result_pair.second);
    if (agent->GetBehaviorStatus() == BehaviorStatus::VALID &&
        agent->IsValidAtTime(world_time_)) {
      nearest_agents[result_pair.second] = agent;
    }
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
        agent->GetBehaviorStatus() == BehaviorStatus::VALID &&
        agent->IsValidAtTime(world_time_)) {
      intersecting_agents[result_pair.second] = agent;
    }
  }
  return intersecting_agents;
}

FrontRearAgents World::GetAgentFrontRearForId(
    const AgentId& agent_id, const LaneCorridorPtr& lane_corridor,
    double lateral_difference_threshold, bool must_be_in_corridor) const {
  using bark::geometry::Line;
  using bark::geometry::Polygon;

  AgentMap considered_agents;
  if(must_be_in_corridor) {
    const Polygon& corridor_polygon = lane_corridor->GetMergedPolygon();
    const Line& center_line = lane_corridor->GetCenterLine();
    considered_agents = GetAgentsIntersectingPolygon(corridor_polygon);
  } else {
    const auto& ego_pos = World::GetAgent(agent_id)->GetCurrentPosition();
    considered_agents = GetNearestAgents(ego_pos,
    max_agents_front_rear_+1); // one more since ego agent is included
  }

  FrontRearAgents fr_agents;
  if (considered_agents.size() == 0) {
    fr_agents.front = std::make_pair(AgentPtr(nullptr), FrenetStateDifference());
    fr_agents.rear = fr_agents.front;
    return fr_agents;
  }

  const auto& ego_state = World::GetAgent(agent_id)->GetCurrentState();
  const Line& center_line = lane_corridor->GetCenterLine();
  FrenetState frenet_ego(ego_state, center_line);
  const auto ego_polygon = World::GetAgent(agent_id)->GetShape();
  const double numeric_max = std::numeric_limits<double>::max();

  AgentPtr nearest_agent_front(nullptr);
  AgentPtr nearest_agent_rear(nullptr);
  FrenetStateDifference nearest_difference_front;
  FrenetStateDifference nearest_difference_rear;

  for (auto it = considered_agents.begin(); it != considered_agents.end();
       ++it) {
    if (it->second->GetAgentId() == agent_id ||
        it->second->GetBehaviorStatus() != BehaviorStatus::VALID ||
        it->second->IsValidAtTime(world_time_) == false) {
      continue;
    }

    FrenetState frenet_other(it->second->GetCurrentState(), center_line);
    FrenetStateDifference difference(frenet_ego, ego_polygon, frenet_other, it->second->GetShape());
    double lat_difference = difference.lat_zeroed ? 0.0 : difference.lat;
    if (std::abs(lat_difference) > lateral_difference_threshold) {
      // agent seems to be not really in same lane
      continue;
    }

    if (difference.lon >= 0.0f && difference.lon < nearest_difference_front.lon) {
      nearest_difference_front = difference;
      nearest_agent_front = it->second;
    } else if (difference.lon < 0.0f &&
               std::abs(difference.lon) < std::abs(nearest_difference_rear.lon)) {
      nearest_difference_rear = difference;
      nearest_agent_rear = it->second;
    }
  }

  fr_agents.front = std::make_pair(nearest_agent_front, nearest_difference_front);
  fr_agents.rear = std::make_pair(nearest_agent_rear, nearest_difference_rear);

  return fr_agents;
}

void World::RemoveAgentById(AgentId agent_id) {
  size_t erased_elems = agents_.erase(agent_id);
  LOG_IF(ERROR, erased_elems == 0)
      << "Could not remove non-existent agent with Id " << agent_id << " !";
}

}  // namespace world
}  // namespace bark
