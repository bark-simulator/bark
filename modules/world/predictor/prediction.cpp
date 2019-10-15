#include "modules/world/predictor/prediction.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/behavior/pure_pursuit/pure_pursuit.hpp"

namespace modules {
namespace world {
namespace prediction {

using goal_definition::GoalDefinition;
using goal_definition::GoalDefinitionPtr;

Prediction::Prediction(commons::Params *params, const ObservedWorld &observed_world, const std::map<AgentId, BehaviorModelPtr> &assumed_agent_behaviors)
    : commons::BaseType(params), observed_world_(observed_world) {
  // TODO(@AKreutz): Change behavior of agents that should not be observed
  AgentMap agents = observed_world_.get_other_agents();
  observed_world_.clear_agents();
  uint32_t n_agents = 10;
  for (auto const &agent : agents) {
    real_agents_to_predictions_.insert(std::map<AgentId, std::vector<AgentId>>::value_type(agent.first, {}));
    predictions_for_all_agents_.insert(std::map<AgentId, AgentPrediction>::value_type(agent.first, AgentPrediction(agent.second->get_agent_id(), agent.second->get_shape())));
    AddAgentsForIntersectionDecisions(agent.second, n_agents);
    AddAgentsForLaneChangeDecisions(agent.second, n_agents);
  }
}

void Prediction::Step(const float time_step) {
  for (auto const &real_agent_it : real_agents_to_predictions_) {
    for (auto const &agent_it : observed_world_.get_other_agents()) {
      if (find(real_agent_it.second.begin(), real_agent_it.second.end(), agent_it.first) != real_agent_it.second.end()) {
        StochasticState predicted_state = {agent_it.second->get_current_state(), StateCovariance::Identity()};
        predictions_for_all_agents_.at(real_agent_it.first).update_hypothesis(agent_it.first, predicted_state);
      }
    }
  }
  observed_world_.Step(time_step);
}

std::map<AgentId, AgentPrediction> Prediction::get_predictions_for_all_agents() {
  return predictions_for_all_agents_;
}

void Prediction::AddAgentsForIntersectionDecisions(const AgentPtr agent, uint32_t &n_agents) {
  State agent_state = agent->get_current_state();
  std::vector<std::list<LaneId>> possible_paths = FindPossibleGoalLanes(
    geometry::Point2d(agent_state(StateDefinition::X_POSITION), agent_state(StateDefinition::Y_POSITION)),
    observed_world_.get_map());
  for (auto const possible_path : possible_paths) {
    AgentPtr prediction_agent = AgentPtr(agent->Clone());
    BehaviorModelPtr behavior_model = std::make_shared<models::behavior::BehaviorConstantVelocity>(get_params());
    prediction_agent->set_behavior_model(behavior_model);
    geometry::Line goal_line = observed_world_.get_map()->get_roadgraph()->get_laneptr(possible_path.back())->get_line();
    geometry::Point2d goal_point = geometry::get_point_at_s(goal_line, goal_line.length());
    geometry::Pose predicted_goal_state(boost::geometry::get<0>(goal_point), boost::geometry::get<1>(goal_point), 0);
    GoalDefinitionPtr predicted_goal = std::make_shared<goal_definition::GoalDefinitionPolygon>(
      geometry::Polygon(predicted_goal_state, {Point2d(-1, -1), Point2d(-1, 1), Point2d(1, 1), Point2d(1, -1), Point2d(-1, -1)}));
    prediction_agent->set_goal_definition(predicted_goal);

    real_agents_to_predictions_.at(agent->get_agent_id()).push_back(n_agents);
    std::vector<LaneId> followed_lane;
    for (auto const lane_id : possible_path) {
      followed_lane.push_back(lane_id);
    }
    float likelihood = possible_path.size() == 1 ? 0.9f : 1.0f / possible_paths.size();
    MotionHypothesis motion_hypothesis = {n_agents, likelihood, followed_lane, {}};
    predictions_for_all_agents_.at(agent->get_agent_id()).add_hypothesis(motion_hypothesis);
    prediction_agent->set_agent_id(n_agents++);

    observed_world_.add_agent(prediction_agent);
  }
}

void Prediction::AddAgentsForLaneChangeDecisions(const AgentPtr agent, uint32_t &n_agents) {
  State agent_state = agent->get_current_state();
  geometry::Point2d position(agent_state(StateDefinition::X_POSITION), agent_state(StateDefinition::Y_POSITION));
  opendrive::LanePtr current_lane = FindNearestLane(position, observed_world_.get_map());

  std::pair<opendrive::LanePtr, bool> inner_neighbor = observed_world_.get_map()->get_inner_neighbor(current_lane->get_id());
  if (inner_neighbor.second && inner_neighbor.first->get_lane_position() != 0 && inner_neighbor.first->get_lane_type() == opendrive::LaneType::DRIVING) {
    // Can change lane to inside lane
    geometry::Line inner_line = observed_world_.get_map()->get_inner_neighbor(inner_neighbor.first->get_id()).first->get_line();
    geometry::Line outer_line = inner_neighbor.first->get_line();
    geometry::Line followed_line = geometry::ComputeCenterLine(outer_line, inner_line);

    AgentPtr prediction_agent = AgentPtr(agent->Clone());
    models::behavior::BehaviorPurePursuit *pure_pursuit_model = new models::behavior::BehaviorPurePursuit(get_params());
    pure_pursuit_model->set_followed_line(followed_line);
    prediction_agent->set_behavior_model(BehaviorModelPtr(pure_pursuit_model));

    real_agents_to_predictions_.at(agent->get_agent_id()).push_back(n_agents);
    MotionHypothesis motion_hypothesis = {n_agents, 0.1f, {}, {}};
    predictions_for_all_agents_.at(agent->get_agent_id()).add_hypothesis(motion_hypothesis);
    prediction_agent->set_agent_id(n_agents++);

    observed_world_.add_agent(prediction_agent);
  }
  std::pair<opendrive::LanePtr, bool> outer_neighbor = observed_world_.get_map()->get_outer_neighbor(current_lane->get_id());
  if (outer_neighbor.second && outer_neighbor.first->get_lane_type() == opendrive::LaneType::DRIVING) {
    // Can change lane to outside lane
    geometry::Line inner_line = current_lane->get_line();
    geometry::Line outer_line = outer_neighbor.first->get_line();
    geometry::Line followed_line = geometry::ComputeCenterLine(outer_line, inner_line);

    AgentPtr prediction_agent = AgentPtr(agent->Clone());
    models::behavior::BehaviorPurePursuit *pure_pursuit_model = new models::behavior::BehaviorPurePursuit(get_params());
    pure_pursuit_model->set_followed_line(followed_line);
    prediction_agent->set_behavior_model(BehaviorModelPtr(pure_pursuit_model));

    real_agents_to_predictions_.at(agent->get_agent_id()).push_back(n_agents);
    MotionHypothesis motion_hypothesis = {n_agents, 0.1f, {}, {}};
    predictions_for_all_agents_.at(agent->get_agent_id()).add_hypothesis(motion_hypothesis);
    prediction_agent->set_agent_id(n_agents++);

    observed_world_.add_agent(prediction_agent);
  }
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
  opendrive::LanePtr current_lane = FindNearestLane(position, map_interface);

  std::vector<std::list<LaneId>> possible_paths = FindPossiblePath(current_lane->get_id(), map_interface);

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

opendrive::LanePtr Prediction::FindNearestLane(const geometry::Point2d &position, const MapInterfacePtr map_interface) const {
  std::vector<opendrive::LanePtr> lanes;
  map_interface->FindNearestLanes(position, 2, lanes);
  opendrive::LanePtr inner_lane, outer_lane;
  if (abs(lanes.front()->get_lane_position()) < abs(lanes.back()->get_lane_position())) {
    inner_lane = lanes.front();
    outer_lane = lanes.back();
  } else {
    inner_lane = lanes.back();
    outer_lane = lanes.front();
  }
  if (geometry::distance(outer_lane->get_line(), position) > geometry::distance(inner_lane->get_line(), outer_lane->get_line())) {
    // Position is between lane 0 and 1, but the two nearest lanes are 1 and 2, because 0 is never returned -> current_lane is the inner_lane
    return inner_lane;
  } else {
    return outer_lane;
  }
}

} // namespace prediction
} // namespace world
} // namespace modules
