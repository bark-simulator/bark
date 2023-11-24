// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/objects/agent.hpp"
#include <cmath>
#include <limits>
#include "bark/world/objects/object.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {
namespace objects {

Agent::Agent(const State& initial_state,
             const BehaviorModelPtr& behavior_model_ptr,
             const DynamicModelPtr& dynamic_model_ptr,
             const ExecutionModelPtr& execution_model, const Polygon& shape,
             const commons::ParamsPtr& params,
             const GoalDefinitionPtr& goal_definition,
             const MapInterfacePtr& map_interface, const Model3D& model_3d)
    : Object(shape, params, model_3d),
      behavior_model_(behavior_model_ptr),
      dynamic_model_(dynamic_model_ptr),
      execution_model_(execution_model),
      history_(),
      max_history_length_(30),
      first_valid_timestamp_(0.0),
      goal_definition_(goal_definition),
      sensed_world_() {
  if (params) {
    max_history_length_ = params->GetInt(
        "MaxHistoryLength",
        "Maximum number of state-input pairs in state-input history", 50);
    first_valid_timestamp_ = params->GetReal(
        "FirstValidTimestamp",
        "First valid time stamp at which agent shall be simulated", 0.0);
  }

  models::behavior::StateActionPair pair;
  pair.first = initial_state;

  // Initially select the action given in the behavior model
  if (behavior_model_ptr) {
    pair.second = behavior_model_ptr->GetLastAction();
  } else {
    pair.second = Action(DiscreteAction(0));
  }

  history_.push_back(pair);

  if (map_interface) {
    if (!GenerateRoadCorridor(map_interface)) {
      LOG(ERROR) << "Failed to generate road corridor for agent "
                 << GetAgentId() << ".";
    } 
    // else {
    //   LOG(INFO) << "Generated road corridor for agent "
    //              << GetAgentId() << ".";
    // }
  }
}

Agent::Agent(const Agent& other_agent)
    : Object(other_agent),
      behavior_model_(other_agent.behavior_model_),
      dynamic_model_(other_agent.dynamic_model_),
      execution_model_(other_agent.execution_model_),
      road_corridor_(other_agent.road_corridor_),
      history_(other_agent.history_),
      max_history_length_(other_agent.max_history_length_),
      first_valid_timestamp_(other_agent.first_valid_timestamp_),
      goal_definition_(other_agent.goal_definition_),
      road_corridor_road_ids_(other_agent.road_corridor_road_ids_),
      road_corridor_driving_direction_(
          other_agent.road_corridor_driving_direction_),
      sensed_world_(other_agent.sensed_world_) {}

void Agent::PlanBehavior(const double& min_planning_dt,
                         const ObservedWorld& observed_world) {
  behavior_model_->PlanBehavior(min_planning_dt, observed_world);
}

void Agent::PlanExecution(const double& world_time) {
  execution_model_->Execute(world_time, behavior_model_->GetLastTrajectory(),
                            dynamic_model_);
}

void Agent::UpdateStateAction() {
  models::behavior::StateActionPair state_action_pair(
      execution_model_->GetExecutedState(), behavior_model_->GetLastAction());
  history_.push_back(state_action_pair);

  //! remove states if queue becomes too large
  if (history_.size() > max_history_length_) {
    history_.erase(history_.begin());
  }
}

bool Agent::GenerateRoadCorridor(const MapInterfacePtr& map_interface) {
  if (!map_interface->GetRoadFromCsvTable()) {  // default xodr
    VLOG(6) << "Map Interface from XODR";
    if (goal_definition_ && road_corridor_road_ids_.empty()) {
      // LOG(INFO) << "Agent has Goal definition but no valid road ID found! Generating road corridors";
      road_corridor_ = map_interface->GenerateRoadCorridor(
          GetCurrentPosition(), goal_definition_->GetShape());
      road_corridor_road_ids_ = road_corridor_->GetRoadIds();
      road_corridor_driving_direction_ = road_corridor_->GetDrivingDirection();
    } else if (!road_corridor_road_ids_.empty()) {
      // LOG(INFO) << "Agent has valid road ID!";
      VLOG(6) << "Road corridor from ids" << road_corridor_road_ids_;
      map_interface->GenerateRoadCorridor(road_corridor_road_ids_,
                                          road_corridor_driving_direction_);
      road_corridor_ = map_interface->GetRoadCorridor(
          road_corridor_road_ids_, road_corridor_driving_direction_);
    } else {
      LOG(INFO) << "Agent has map interface but no information to generate "
                   "road corridor.";
      return false;
    }
  } else {  // road from csv
    VLOG(6) << "Map Interface from CSV";
    road_corridor_road_ids_ = {static_cast<world::map::XodrRoadId>(0)};
    road_corridor_driving_direction_ = bark::world::opendrive::
      XodrDrivingDirection::FORWARD;
    road_corridor_ = map_interface->GetRoadCorridor(
      road_corridor_road_ids_, road_corridor_driving_direction_);
  }

  if (!road_corridor_) {
    LOG(INFO) << "No corridor for agent found.";
    return false;
  }
  return true;
}

FrenetPosition Agent::CurrentFrenetPosition() const {
  const Point2d pos = GetCurrentPosition();
  const auto& lane_corridor = GetRoadCorridor()->GetCurrentLaneCorridor(pos);
  if (!lane_corridor) {
    // assume vehicle is far far away on same lane
    // (until better failure handling implemented)
    return FrenetPosition(0.0, std::numeric_limits<double>::max());
  }
  FrenetPosition frenet_pos(pos, lane_corridor->GetCenterLine());
  return frenet_pos;
}

Polygon Agent::GetPolygonFromState(const State& state) const {
  Pose agent_pose(state(StateDefinition::X_POSITION),
                  state(StateDefinition::Y_POSITION),
                  state(StateDefinition::THETA_POSITION));
  std::shared_ptr<Polygon> polygon(std::dynamic_pointer_cast<Polygon>(
      this->GetShape().Transform(agent_pose)));
  return *polygon;
}

bool Agent::AtGoal() const {
  BARK_EXPECT_TRUE((bool)goal_definition_);
  return goal_definition_->AtGoal(*this);
}

bool Agent::InsideRoadCorridor() const {
  if (!road_corridor_) {
    return false;
  } else {
    Polygon agent_poly = GetPolygonFromState(GetCurrentState());
    bool inside = Within(agent_poly, road_corridor_->GetPolygon());
    return inside;
  }
}

/**
 * @brief checks validity of agent. feature is required with simulating datasets
 * in closed loop.
 *
 * @param world_time ... current world time
 * @return true if agent is valid
 */
bool Agent::IsValidAtTime(const double& world_time) const {
  return isgreaterequal(world_time + std::numeric_limits<double>::epsilon(),
                        first_valid_timestamp_);
}

std::shared_ptr<Object> Agent::Clone() const {
  std::shared_ptr<Agent> new_agent = std::make_shared<Agent>(*this);
  new_agent->SetAgentId(this->GetAgentId());
  if (behavior_model_) {
    new_agent->behavior_model_ = behavior_model_->Clone();
  }
  if (dynamic_model_) {
    new_agent->dynamic_model_ = dynamic_model_->Clone();
  }
  if (execution_model_) {
    new_agent->execution_model_ = execution_model_->Clone();
  }
  if (goal_definition_) {
    new_agent->goal_definition_ = goal_definition_->Clone();
  }
  return std::dynamic_pointer_cast<Object>(new_agent);
}

}  // namespace objects
}  // namespace world
}  // namespace bark
