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

Agent::Agent(const State& initial_state,
        const BehaviorModelPtr& behavior_model_ptr,
        const DynamicModelPtr& dynamic_model_ptr,
        const ExecutionModelPtr& execution_model,
        const geometry::Polygon& shape,
        commons::Params* params,
        const GoalDefinitionPtr& goal_definition,
        const MapInterfacePtr& map_interface,
        const geometry::Model3D& model_3d) :
Object(shape, params, model_3d),
behavior_model_(behavior_model_ptr),
dynamic_model_(dynamic_model_ptr),
execution_model_(execution_model),
history_(),
max_history_length_(10),
goal_definition_(goal_definition) {
  if (params) {
    max_history_length_ = params->get_int(
    "MaxHistoryLength",
    "Maximum number of state-input pairs in state-input history",
     50);
  }

  models::behavior::StateActionPair pair;
  pair.first = initial_state; 
  pair.second = modules::models::behavior::Action(
    modules::models::behavior::DiscreteAction(0)); // Initially select a DiscreteAction  of zero
  history_.push_back(pair);

  if(map_interface) {
     if(!GenerateRoadCorridor(map_interface)) {
       LOG(ERROR) << "Failed to generate road corridor for agent " << get_agent_id();
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
  goal_definition_(other_agent.goal_definition_) {}


void Agent::BehaviorPlan(const float &dt, const ObservedWorld &observed_world) {
  //! plan behavior for given horizon T using step-size dt
  behavior_model_->Plan(dt, observed_world);
}

void Agent::ExecutionPlan(const float &dt) {
  execution_model_->Execute(dt,
                            behavior_model_->get_last_trajectory(),
                            dynamic_model_,
                            history_.back().first);
}

void Agent::Execute(const float& world_time) {
  //! find closest state in execution-trajectory
  int index_world_time = 0;
  float min_time_diff = std::numeric_limits<float>::max();
  Trajectory last_trajectory = execution_model_->get_last_trajectory();
  for (int i = 0; i < last_trajectory.rows(); i++) {
    float diff_time = fabs(last_trajectory(i, TIME_POSITION) - world_time);
    if (diff_time < min_time_diff) {
      index_world_time = i;
      min_time_diff = diff_time;
    }
  }
  models::behavior::StateActionPair state_action_pair(
      State(execution_model_->get_last_trajectory().row(index_world_time)),
      behavior_model_->get_last_action());
  history_.push_back(state_action_pair);

  //! remove states if queue becomes to large
  if (history_.size() > max_history_length_) {
    history_.erase(history_.begin());
  }
}


bool Agent::GenerateRoadCorridor(const MapInterfacePtr& map_interface) {
  if (!goal_definition_) {
    return false;
  }
  road_corridor_ = map_interface->GenerateRoadCorridor(
  get_current_position(),
  goal_definition_->get_shape());
  if(!road_corridor_) {
    return false;
  }
  return true;
}

geometry::Polygon Agent::GetPolygonFromState(const State& state) const {
  using namespace modules::geometry;
  using namespace modules::geometry::standard_shapes;
  Pose agent_pose(state(StateDefinition::X_POSITION),
                  state(StateDefinition::Y_POSITION),
                  state(StateDefinition::THETA_POSITION));
  std::shared_ptr<geometry::Polygon> polygon(
    std::dynamic_pointer_cast<geometry::Polygon>(
      this->get_shape().transform(agent_pose)));
  return *polygon;
}

bool Agent::AtGoal() const {
  BARK_EXPECT_TRUE((bool)goal_definition_);
  return goal_definition_->AtGoal(*this);
}


std::shared_ptr<Object> Agent::Clone() const {
  std::shared_ptr<Agent> new_agent = std::make_shared<Agent>(*this);
  new_agent->set_agent_id(this->get_agent_id());
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
