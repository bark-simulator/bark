// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_OBJECTS_AGENT_HPP_
#define MODULES_WORLD_OBJECTS_AGENT_HPP_

#include "modules/commons/base_type.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/world/map/road_corridor.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/world/goal_definition/goal_definition.hpp"
#include "modules/world/objects/object.hpp"
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/models/execution/execution_model.hpp"
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/commons/transformation/frenet.hpp"

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
using modules::world::opendrive::XodrLaneId;
using modules::world::map::MapInterfacePtr;
using modules::world::map::RoadCorridorPtr;
using modules::world::goal_definition::GoalDefinition;
using modules::world::goal_definition::GoalDefinitionPtr;
using models::dynamic::StateDefinition;

class Agent : public Object {
 public:
  friend class World;

  Agent(const State &initial_state,
        const BehaviorModelPtr &behavior_model_ptr,
        const DynamicModelPtr &dynamic_model_ptr,
        const ExecutionModelPtr &execution_model,
        const geometry::Polygon &shape,
        const commons::ParamsPtr& params,
        const GoalDefinitionPtr& goal_definition = GoalDefinitionPtr(),
        const MapInterfacePtr& map_interface = MapInterfacePtr(),
        const geometry::Model3D &model_3d = geometry::Model3D());

  virtual ~Agent() {}

  Agent(const Agent& other_agent);

  BehaviorModelPtr GetBehaviorModel() const { return behavior_model_; }

  ExecutionModelPtr GetExecutionModel() const { return execution_model_; }

  DynamicModelPtr GetDynamicModel() const { return dynamic_model_; }

  StateActionHistory GetStateInputHistory() const { return history_; }

  GoalDefinitionPtr GetGoalDefinition() const {return goal_definition_;}

  Trajectory GetExecutionTrajectory() const {
    return execution_model_->GetLastTrajectory();
  }

  Trajectory GetBehaviorTrajectory() const {
    return behavior_model_->GetLastTrajectory();
  }

  State GetCurrentState() const { return history_.back().first; }

  modules::geometry::Point2d GetCurrentPosition() const {
    const State& state = GetCurrentState();
    return modules::geometry::Point2d(
      state(StateDefinition::X_POSITION),
      state(StateDefinition::Y_POSITION));
  }

  modules::commons::transformation::FrenetPosition CurrentFrenetPosition() const;

  geometry::Polygon GetPolygonFromState(const State& state) const;

  const RoadCorridorPtr GetRoadCorridor() const {
    return road_corridor_;
  }

  void SetBehaviorModel(const BehaviorModelPtr &behavior_model_ptr) {
    behavior_model_ = behavior_model_ptr;
  }

  void SetGoalDefinition(const GoalDefinitionPtr &goal_definition) {
    goal_definition_ = goal_definition;
  }

  void SetStateInputHistory(const StateActionHistory& history) {
    history_ = history;
  }

  bool GenerateRoadCorridor(const MapInterfacePtr& map_interface);

  void SetRoadCorridor(const RoadCorridorPtr road_corridor) {
    road_corridor_ = road_corridor;
  }

  void BehaviorPlan(const float &dt, const ObservedWorld &observed_world);

  void ExecutionPlan(const float &dt);

  void Execute(const float& world_time);

  bool AtGoal() const;

  virtual std::shared_ptr<Object> Clone() const;

 private:
  models::behavior::BehaviorModelPtr behavior_model_;
  models::dynamic::DynamicModelPtr dynamic_model_;
  models::execution::ExecutionModelPtr execution_model_;
  modules::world::map::RoadCorridorPtr road_corridor_;
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
