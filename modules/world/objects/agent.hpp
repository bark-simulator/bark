// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_OBJECTS_AGENT_HPP_
#define MODULES_WORLD_OBJECTS_AGENT_HPP_

#include "modules/commons/base_type.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/world/map/local_map.hpp"
#include "modules/world/goal_definition/goal_definition.hpp"
#include "modules/world/objects/object.hpp"
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/models/execution/execution_model.hpp"
#include "modules/world/opendrive/opendrive.hpp"

namespace modules {
namespace world {
class ObservedWorld;
namespace objects {

typedef unsigned int AgentId;
using models::dynamic::State;
using models::behavior::BehaviorModelPtr;
using models::dynamic::DynamicModelPtr;
using models::execution::ExecutionModelPtr;
using models::behavior::StateActionHistory;
using models::dynamic::Trajectory;
using modules::world::opendrive::LaneId;
using modules::world::map::MapInterfacePtr;
using modules::world::goal_definition::GoalDefinition;
using modules::world::goal_definition::GoalDefinitionPtr;
using models::dynamic::StateDefinition;

class Agent : public Object {
 public:
  friend class World;

  // TODO(@fortiss): specify goal in a more detailed way
  Agent(const State &initial_state,
        const BehaviorModelPtr &behavior_model_ptr,
        const DynamicModelPtr &dynamic_model_ptr,
        const ExecutionModelPtr &execution_model,
        const geometry::Polygon &shape,
        commons::Params *params,
        const GoalDefinitionPtr& goal_definition = GoalDefinitionPtr(),
        const MapInterfacePtr& map_interface = MapInterfacePtr(),
        const geometry::Model3D &model_3d = geometry::Model3D());

  virtual ~Agent() {}

  Agent(const Agent& other_agent);

  BehaviorModelPtr get_behavior_model() const { return behavior_model_; }

  ExecutionModelPtr get_execution_model() const { return execution_model_; }

  DynamicModelPtr get_dynamic_model() const { return dynamic_model_; }

  StateActionHistory get_state_input_history() const { return history_; }

  GoalDefinitionPtr get_goal_definition() const {return goal_definition_;}

  Trajectory get_execution_trajectory() const {
    return execution_model_->get_last_trajectory();
  }

  Trajectory get_behavior_trajectory() const {
    return behavior_model_->get_last_trajectory();
  }

  State get_current_state() const { return history_.back().first; }

  modules::geometry::Point2d get_current_position() const {   const State& state = get_current_state();
      return modules::geometry::Point2d(state(StateDefinition::X_POSITION), state(StateDefinition::Y_POSITION));}

  geometry::Polygon GetPolygonFromState(const State& state) const;

  const modules::world::map::LocalMapPtr& get_local_map() const {
    return local_map_;
  }

  void set_behavior_model(const BehaviorModelPtr &behavior_model_ptr) {
    behavior_model_ = behavior_model_ptr;
  }

  void set_goal_definition(const GoalDefinitionPtr &goal_definition) {
    goal_definition_ = goal_definition;
  }

  void set_local_map(const modules::world::map::LocalMapPtr& local_map) {local_map_ = local_map; }

  void BehaviorPlan(const float &dt, const ObservedWorld &observed_world);

  void ExecutionPlan(const float &dt);

  void Execute(const float& world_time);

  bool AtGoal() const;

  void GenerateLocalMap();
  void UpdateDrivingCorridor(double horizon);

  virtual std::shared_ptr<Object> Clone() const;

 private:
  models::behavior::BehaviorModelPtr behavior_model_;
  models::dynamic::DynamicModelPtr dynamic_model_;
  models::execution::ExecutionModelPtr execution_model_;

  // TODO(@fortiss): this should be the local map the planners work with
  modules::world::map::LocalMapPtr local_map_;

  models::behavior::StateActionHistory history_;
  // TODO(fortiss): move max_history_length_ to parameter
  uint32_t max_history_length_;
  modules::world::goal_definition::GoalDefinitionPtr goal_definition_;
};

typedef std::shared_ptr<Agent> AgentPtr;

}  // namespace objects
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OBJECTS_AGENT_HPP_
