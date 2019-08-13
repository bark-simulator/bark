#include "modules/world/prediction/prediction.hpp"

namespace modules {
namespace world {
namespace prediction {

using goal_definition::GoalDefinition;

Prediction::Prediction(commons::Params *params, const ObservedWorld &observed_world, const std::map<AgentId, BehaviorModelPtr> &assumed_agent_behaviors)
    : commons::BaseType(params), observed_world_(observed_world) {
  // TODO(@AKreutz): Change behavior of agents that should not be observed
  AgentMap agents = observed_world_.get_other_agents();
  observed_world_.clear_agents();
  uint32_t n_agents = 10;
  for (auto const &agent : agents) {
    real_agents_to_predictions_.insert(std::map<AgentId, std::vector<AgentId>>::value_type(agent.first, {}));
    State agent_state = agent.second->get_current_state();
    std::vector<std::list<LaneId>> possible_paths = FindPossibleGoalLanes(
      geometry::Point2d(agent_state(StateDefinition::X_POSITION), agent_state(StateDefinition::Y_POSITION)),
      observed_world_.get_map());
    for (auto const possible_path : possible_paths) {
      AgentPtr prediction_agent = std::make_shared<Agent>(*agent.second->Clone());
      prediction_agent->set_behavior_model(assumed_agent_behaviors.at(agent.first));
      geometry::Line goal_line = observed_world_.get_map()->get_roadgraph()->get_laneptr(possible_path.back())->get_line();
      geometry::Point2d goal_point = geometry::get_point_at_s(goal_line, goal_line.length());
      geometry::Pose predicted_goal_state(boost::geometry::get<0>(goal_point), boost::geometry::get<1>(goal_point), 0);
      GoalDefinition predicted_goal(geometry::Polygon(predicted_goal_state, {Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1), Point2d(-1, -1)}));
      prediction_agent->set_goal_definition(predicted_goal);

      real_agents_to_predictions_.at(agent.first).push_back(n_agents);
      prediction_agent->set_agent_id(n_agents++);

      observed_world_.add_agent(prediction_agent);
    }
  }
}

void Prediction::Step(const float time_step) {
  observed_world_.Step(time_step);
}

AgentPrediction Prediction::get_agent_prediction(const AgentId agent_id) {
  std::vector<AgentId> predicted_agent_ids = real_agents_to_predictions_.at(agent_id);
  AgentPtr agent;
  std::vector<MotionHypothesis> motion_hypotheses;
  uint32_t n_hypotheses = 0;
  for (auto const &agent_it : observed_world_.get_other_agents()) {
    if (find(predicted_agent_ids.begin(), predicted_agent_ids.end(), agent_it.first) != predicted_agent_ids.end()) {
      agent = agent_it.second;
      StochasticState predicted_state = {agent_it.second->get_current_state(), StateCovariance::Identity()};
      std::vector<LaneId> followed_lanes;
      for (auto const lane_id_it : agent_it.second->get_local_map()->get_driving_corridor().lane_ids_) {
        followed_lanes.push_back(lane_id_it.second);
      }
      motion_hypotheses.push_back({n_hypotheses++, 1, followed_lanes, predicted_state});
    }
  }
  return AgentPrediction(*agent, motion_hypotheses);
}

std::vector<std::list<LaneId>> FindPossiblePath(const LaneId current_lane_id, const MapInterfacePtr map_interface) {
  std::vector<std::list<LaneId>> possible_paths;
  std::vector<LaneId> successors = map_interface->get_successor_lanes(current_lane_id);
  if (successors.empty()) {
    return {{current_lane_id}};
  }
  for (auto const successor : successors) {
    std::vector<std::list<LaneId>> successor_paths = FindPossiblePath(successor, map_interface);
    for (auto successor_path : successor_paths) {
      successor_path.push_front(current_lane_id);
      possible_paths.push_back(successor_path);
    }
  }
  return possible_paths;
}

std::vector<std::list<LaneId>> Prediction::FindPossibleGoalLanes(const geometry::Point2d &position, const MapInterfacePtr map_interface) const {
  std::vector<opendrive::LanePtr> lanes;
  map_interface->FindNearestLanes(position, 1, lanes);
  LaneId current_lane = lanes.front()->get_id();

  std::vector<std::list<LaneId>> possible_paths = FindPossiblePath(current_lane, map_interface);

  std::vector<std::list<LaneId>> paths_without_duplicates;
  for (auto const &possible_path : possible_paths) {
    bool is_new_path = true;
    for (auto const &path_without_duplicates : paths_without_duplicates) {
      bool is_duplicate_of_this = true;
      auto possible_path_it = possible_path.begin();
      auto path_without_duplicates_it = path_without_duplicates.begin();
      while (possible_path_it != possible_path.end() && path_without_duplicates_it != path_without_duplicates.end()) {
        if (*possible_path_it != *path_without_duplicates_it) {
          is_duplicate_of_this = false;
          break;
        }
        ++possible_path_it;
        ++path_without_duplicates_it;
      }
      if (is_duplicate_of_this) {
        is_new_path = false;
        break;
      }
    }
    if (is_new_path) {
      paths_without_duplicates.push_back(possible_path);
    }
  }
  return paths_without_duplicates;
}

} // namespace prediction
} // namespace world
} // namespace modules
