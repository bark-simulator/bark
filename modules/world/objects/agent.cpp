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

Agent::Agent(const State &initial_state,
        const BehaviorModelPtr &behavior_model_ptr,
        const DynamicModelPtr &dynamic_model_ptr,
        const ExecutionModelPtr &execution_model,
        const geometry::Polygon &shape,
        commons::Params *params,
        LaneId goal_lane_id,
        const MapInterfacePtr& map_interface,
        const geometry::Model3D &model_3d) :
Object(shape, params, model_3d),
behavior_model_(behavior_model_ptr),
dynamic_model_(dynamic_model_ptr),
execution_model_(execution_model),
history_(),
route_generator_(new RouteGenerator(goal_lane_id, map_interface)),
goal_lane_id_(goal_lane_id) {
  max_history_length_ = params->get_int(
    "MaxHistoryLength",
    "Maximum number of state-input pairs in state-input history",
     50);
  models::dynamic::StateInputPair pair;
  pair.first = initial_state;  //! TODO(fortiss): check for state dimensions
  history_.push(pair);
}

Agent::Agent(const Agent& other_agent) :
  Object(other_agent),
  behavior_model_(other_agent.behavior_model_),
  dynamic_model_(other_agent.dynamic_model_),
  execution_model_(other_agent.execution_model_),
  history_(other_agent.history_),
  route_generator_(other_agent.route_generator_),
  goal_lane_id_(other_agent.goal_lane_id_) {}


void Agent::Move(const float &dt, const ObservedWorld &observed_world) {
  //! plan behavior for given horizon T using step-size dt
  behavior_model_->Plan(dt, observed_world);

  //! make dynamically feasible/validate
  // TODO(@hart): observed world has ego-agent, thus: history_ is known
  execution_model_->Execute(observed_world.get_world_time(),
                            behavior_model_->get_last_trajectory(),
                            dynamic_model_,
                            history_.back().first);


  //! find closest state in execution-trajectory
  int index_new_world_time = 0;
  float new_world_time = observed_world.get_world_time() + dt;
  float min_time_diff = std::numeric_limits<float>::max();
  Trajectory last_trajectory = execution_model_->get_last_trajectory();
  for (int i = 0; i < last_trajectory.rows(); i++) {
    float diff_time = fabs(last_trajectory(i, TIME_POSITION) - new_world_time);
    if (diff_time < min_time_diff) {
      index_new_world_time = i;
      min_time_diff = diff_time;
    }
  }

  // TODO(fortiss): Input should not be zero!
  models::dynamic::StateInputPair state_input_pair(
      State(execution_model_->get_last_trajectory().row(index_new_world_time)),
      models::dynamic::Input::Zero(1, 1));
  history_.push(state_input_pair);

  //! remove states if queue becomes to large
  if (history_.size() > max_history_length_) {
    history_.pop();
  }
}

geometry::Polygon Agent::GetPolygonFromState(const State& state) const {

  using namespace modules::geometry;
  using namespace modules::geometry::standard_shapes;

  Pose agent_pose(state(StateDefinition::X_POSITION), state(StateDefinition::Y_POSITION), state(StateDefinition::THETA_POSITION));
  
  geometry::Polygon *polygon = dynamic_cast<Polygon *>(this->get_shape().transform(agent_pose));

  return *polygon;  
}

void Agent::UpdateLocalRoute() {
  State agent_state = get_current_state();
  Point2d agent_xy(agent_state(StateDefinition::X_POSITION),
                   agent_state(StateDefinition::Y_POSITION));
  if (!route_generator_->generate(agent_xy, goal_lane_id_)) {
    std::cout << "Routing generation for agent "
              << get_agent_id() << " failed." << std::endl;
  }
}

Agent* Agent::Clone() const {
  Agent *new_agent = new Agent(*this);
  new_agent->behavior_model_.reset(behavior_model_->Clone());
  new_agent->dynamic_model_.reset(dynamic_model_->Clone());
  return new_agent;
}

}  // namespace objects
}  // namespace world
}  // namespace modules
