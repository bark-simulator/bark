// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <limits>
#include <cmath>
#include "modules/world/objects/agent.hpp"
#include "modules/world/objects/object.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {
namespace objects {

using models::dynamic::StateDefinition;
using modules::geometry::Point2d;
using modules::world::map::MapInterfacePtr;
using StateDefinition::TIME_POSITION;
using modules::commons::transformation::FrenetPosition;
using modules::world::opendrive::XodrDrivingDirection;

Agent::Agent(const State& initial_state,
        const BehaviorModelPtr& behavior_model_ptr,
        const DynamicModelPtr& dynamic_model_ptr,
        const ExecutionModelPtr& execution_model,
        const geometry::Polygon& shape,
        const commons::ParamsPtr& params,
        const GoalDefinitionPtr& goal_definition,
        const MapInterfacePtr& map_interface,
        const geometry::Model3D& model_3d) :
Object(shape, params, model_3d),
behavior_model_(behavior_model_ptr),
dynamic_model_(dynamic_model_ptr),
execution_model_(execution_model),
history_(),
max_history_length_(10),
goal_definition_(goal_definition),
road_corridor_road_ids_() {
  if (params) {
    max_history_length_ = params->GetInt(
    "MaxHistoryLength",
    "Maximum number of state-input pairs in state-input history",
     50);
  }

  models::behavior::StateActionPair pair;
  pair.first = initial_state; 
  pair.second = behavior_model_->GetLastAction(); // init with behavior models init action
  history_.push_back(pair);

  if(map_interface) {
     if(!GenerateRoadCorridor(map_interface)) {
       LOG(ERROR) << "Failed to generate road corridor for agent " << GetAgentId();
     }
  }

}


Agent::Agent(const Agent& other_agent) :
  Object(other_agent),
  behavior_model_(other_agent.behavior_model_),
  dynamic_model_(other_agent.dynamic_model_),
  execution_model_(other_agent.execution_model_),
  road_corridor_(other_agent.road_corridor_),
  history_(other_agent.history_),
  max_history_length_(other_agent.max_history_length_),
  goal_definition_(other_agent.goal_definition_),
  road_corridor_road_ids_(other_agent.road_corridor_road_ids_),
  road_corridor_driving_direction_(other_agent.road_corridor_driving_direction_) {}


void Agent::BehaviorPlan(const float &dt, const ObservedWorld &observed_world) {
  //! plan behavior for given horizon T using step-size dt
  behavior_model_->Plan(dt, observed_world);
}

void Agent::ExecutionPlan(const float &dt) {
  execution_model_->Execute(dt,
                            behavior_model_->GetLastTrajectory(),
                            dynamic_model_,
                            history_.back().first);
}

void Agent::Execute(const float& world_time) {
  //! find closest state in execution-trajectory
  int index_world_time = 0;
  float min_time_diff = std::numeric_limits<float>::max();
  Trajectory last_trajectory = execution_model_->GetLastTrajectory();
  if(!models::dynamic::IsValid(last_trajectory)) {
    LOG(FATAL) << "Invalid execution trajectory of agent " << GetAgentId() << " : \n" << last_trajectory;
  }
  for (int i = 0; i < last_trajectory.rows(); i++) {
    float diff_time = fabs(last_trajectory(i, TIME_POSITION) - world_time);
    if (diff_time < min_time_diff) {
      index_world_time = i;
      min_time_diff = diff_time;
    }
  }
  models::behavior::StateActionPair state_action_pair(
      State(execution_model_->GetLastTrajectory().row(index_world_time)),
      behavior_model_->GetLastAction());
  history_.push_back(state_action_pair);

  //! remove states if queue becomes to large
  if (history_.size() > max_history_length_) {
    history_.erase(history_.begin());
  }
}


bool Agent::GenerateRoadCorridor(const MapInterfacePtr& map_interface) { 
  if (goal_definition_ && road_corridor_road_ids_.empty()) {
    road_corridor_ = map_interface->GenerateRoadCorridor(
    GetCurrentPosition(),
    goal_definition_->GetShape());
    road_corridor_road_ids_ = road_corridor_->GetRoadIds();
    road_corridor_driving_direction_ = road_corridor_->GetDrivingDirection();
  } else if(!road_corridor_road_ids_.empty()) {
    map_interface->GenerateRoadCorridor(road_corridor_road_ids_,
                                  road_corridor_driving_direction_);
    road_corridor_ = map_interface->GetRoadCorridor(road_corridor_road_ids_, 
                                              road_corridor_driving_direction_);
  } else {
    LOG(INFO) << "Agent has map interface but no information to generate road corridor.";
    return false;
  }

  if(!road_corridor_) {
    LOG(INFO) << "No corridor for agent found.";
    return false;
  }
  return true;
}

FrenetPosition Agent::CurrentFrenetPosition() const {  
  const modules::geometry::Point2d pos = GetCurrentPosition();
  const auto& lane_corridor = GetRoadCorridor()->GetCurrentLaneCorridor(pos);
  if(!lane_corridor) {
    // assume vehicle is far far away on same lane (until better failure handling implemented)
    return FrenetPosition(0.0f, std::numeric_limits<double>::max());
  }
  FrenetPosition frenet_pos(pos, lane_corridor->GetCenterLine());
  return frenet_pos;
}

geometry::Polygon Agent::GetPolygonFromState(const State& state) const {
  using namespace modules::geometry;
  using namespace modules::geometry::standard_shapes;
  Pose agent_pose(state(StateDefinition::X_POSITION),
                  state(StateDefinition::Y_POSITION),
                  state(StateDefinition::THETA_POSITION));
  std::shared_ptr<geometry::Polygon> polygon(
    std::dynamic_pointer_cast<geometry::Polygon>(
      this->GetShape().Transform(agent_pose)));
  return *polygon;
}

bool Agent::AtGoal() const {
  BARK_EXPECT_TRUE((bool)goal_definition_);
  return goal_definition_->AtGoal(*this);
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
  return std::dynamic_pointer_cast<Object>(new_agent);
}

}  // namespace objects
}  // namespace world
}  // namespace modules
