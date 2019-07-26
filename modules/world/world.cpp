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

void World::add_evaluator(const std::string& name, const EvaluatorPtr& evaluator) {
  evaluators_[name] = evaluator;
}

void World::MoveAgents(const float& delta_time) {
  WorldPtr current_world_state(this->Clone());
  for (auto agent : agents_) {
      //! clone current world
      ObservedWorld observed_world(*current_world_state,
                                   agent.first);
      agent.second->Move(delta_time, observed_world);
  }
  world_time_ += delta_time;
}

World::EvaluationMap World::Evaluate() const {
  World::EvaluationMap evaluation_results;
  for(auto const& evaluator : evaluators_) {
      evaluation_results[evaluator.first] = evaluator.second->Evaluate(*this); 
    }
  return evaluation_results;
}

void World::UpdateHorizonDrivingCorridors() {
  for (auto agent : agents_) {
    // TODO(@hart): parameter
    // TODO(@hart): check if update is required
    agent.second->UpdateDrivingCorridor(40.0);
  }
}

void World::Step(const float& delta_time) {
  UpdateHorizonDrivingCorridors();
  MoveAgents(delta_time);
  // TODO(@fortiss): add post world collision check
}

std::vector<ObservedWorld> World::Observe(const std::vector<AgentId>& agent_ids) {
  WorldPtr current_world_state(this->Clone());
  std::vector<ObservedWorld> observed_worlds;
  for (auto agent_id : agent_ids) {
      if(agents_.find(agent_id) == agents_.end()) {
        std::cout << "Unvalid agent id " << agent_id << ". Skipping ...." << std::endl;
        continue;
      }
      ObservedWorld observed_world(*current_world_state,
                                   agent_id);
      observed_worlds.push_back(observed_world);
  }
  return observed_worlds;
}


void World::UpdateAgentRTree() {
  rtree_agents_.clear();
  for(auto &agent : agents_) {
    auto obj = agent.second->GetPolygonFromState(agent.second->get_current_state()).obj_;
    rtree_agent_model box;
    boost::geometry::envelope(obj, box);
    boost::geometry::correct(box);
    rtree_agents_.insert(std::make_pair(box, agent.first));
  }

}

AgentMap World::GetNearestAgents(const modules::geometry::Point2d& position, const unsigned int& num_agents) {
  std::vector<rtree_agent_value> results_n;

  rtree_agents_.query(boost::geometry::index::nearest(position, num_agents),
            std::back_inserter(results_n));

  AgentMap nearest_agents;
  for (auto &result_pair : results_n) {
    nearest_agents[result_pair.second] =  agents_[result_pair.second];
  }
  return nearest_agents;
}


}  // namespace world
}  // namespace modules
