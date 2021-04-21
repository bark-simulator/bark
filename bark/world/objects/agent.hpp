// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_OBJECTS_AGENT_HPP_
#define BARK_WORLD_OBJECTS_AGENT_HPP_

#include "bark/commons/base_type.hpp"
#include "bark/commons/transformation/frenet.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"
#include "bark/models/execution/execution_model.hpp"
#include "bark/world/goal_definition/goal_definition.hpp"
#include "bark/world/map/map_interface.hpp"
#include "bark/world/map/road_corridor.hpp"
#include "bark/world/objects/object.hpp"
#include "bark/world/opendrive/opendrive.hpp"

namespace bark {
namespace world {
class ObservedWorld;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;
namespace objects {

typedef unsigned int AgentId;
using bark::commons::transformation::FrenetPosition;
using bark::world::goal_definition::GoalDefinition;
using bark::world::goal_definition::GoalDefinitionPtr;
using bark::world::map::MapInterfacePtr;
using bark::world::map::RoadCorridorPtr;
using bark::world::opendrive::XodrLaneId;
using models::behavior::Action;
using models::behavior::BehaviorModelPtr;
using models::behavior::BehaviorStatus;
using models::behavior::StateActionHistory;
using models::behavior::StateActionPair;
using models::dynamic::DynamicModelPtr;
using models::dynamic::State;
using models::dynamic::StateDefinition;
using models::dynamic::Trajectory;
using models::execution::ExecutionModelPtr;
using models::execution::ExecutionStatus;
using StateHistory = std::vector<State>;
using ActionHistory = std::vector<Action>;
using bark::geometry::Model3D;
using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::geometry::Pose;
using bark::geometry::Within;

class Agent : public Object {
 public:
  friend class World;

  Agent(const State& initial_state, const BehaviorModelPtr& behavior_model_ptr,
        const DynamicModelPtr& dynamic_model_ptr,
        const ExecutionModelPtr& execution_model, const Polygon& shape,
        const commons::ParamsPtr& params,
        const GoalDefinitionPtr& goal_definition = GoalDefinitionPtr(),
        const MapInterfacePtr& map_interface = MapInterfacePtr(),
        const Model3D& model_3d = Model3D());

  virtual ~Agent() {}

  Agent(const Agent& other_agent);

  /**
   * @brief  Generates the RoadCorridor for the agent using
   *         its current position and goal
   */
  bool GenerateRoadCorridor(const MapInterfacePtr& map_interface);

  /**
   * @brief  Calls the behavior model of the agent
   */
  void PlanBehavior(const double& dt, const ObservedWorld& observed_world);

  /**
   * @brief  Calls the execution model of the agent
   */
  void PlanExecution(const double& world_time);

  /**
   * @brief  Updates the agent states based on the execution model
   */
  void UpdateStateAction();

  /**
   * @brief  Checks whether the agent has reached its goal
   */
  bool AtGoal() const;

  /**
   * @brief  Checks is valid at the given world time
   */
  bool IsValidAtTime(const double& world_time) const;

  //! Getter
  BehaviorModelPtr GetBehaviorModel() const { return behavior_model_; }

  ExecutionModelPtr GetExecutionModel() const { return execution_model_; }

  DynamicModelPtr GetDynamicModel() const { return dynamic_model_; }

  StateActionHistory GetStateInputHistory() const { return history_; }

  GoalDefinitionPtr GetGoalDefinition() const { return goal_definition_; }

  Trajectory GetExecutionTrajectory() const {
    return execution_model_->GetLastTrajectory();
  }

  Trajectory GetBehaviorTrajectory() const {
    return behavior_model_->GetLastTrajectory();
  }

  State GetCurrentState() const { return history_.back().first; }

  Point2d GetCurrentPosition() const {
    const State& state = GetCurrentState();
    return Point2d(state(StateDefinition::X_POSITION),
                   state(StateDefinition::Y_POSITION));
  }

  FrenetPosition CurrentFrenetPosition() const;

  Polygon GetPolygonFromState(const State& state) const;

  const RoadCorridorPtr GetRoadCorridor() const { return road_corridor_; }

  BehaviorStatus GetBehaviorStatus() const {
    return behavior_model_->GetBehaviorStatus();
  }

  bool InsideRoadCorridor() const;

  ExecutionStatus GetExecutionStatus() const {
    return execution_model_->GetExecutionStatus();
  }
  double GetFirstValidTimestamp() const {
    return first_valid_timestamp_;
  }

  int GetMaxHistoryLength() const {
    return max_history_length_;
  }

  //! Setter
  void SetBehaviorModel(const BehaviorModelPtr& behavior_model_ptr) {
    behavior_model_ = behavior_model_ptr;
  }

  void SetExecutionModel(const ExecutionModelPtr& execution_model_ptr) {
    execution_model_ = execution_model_ptr;
  }

  void SetDynamicModel(const DynamicModelPtr& dynamic_model_ptr) {
    dynamic_model_ = dynamic_model_ptr;
  }

  void SetGoalDefinition(const GoalDefinitionPtr& goal_definition) {
    goal_definition_ = goal_definition;
  }

  void SetStateInputHistory(const StateActionHistory& history) {
    history_ = history;
  }

  void SetCurrentState(const State& current_state) {
    history_.back().first = current_state;
  }

  void SetRoadCorridor(const RoadCorridorPtr road_corridor) {
    road_corridor_ = road_corridor;
    road_corridor_road_ids_ = road_corridor_->GetRoadIds();
    road_corridor_driving_direction_ = road_corridor_->GetDrivingDirection();
  }

  void SetFirstValidTimestamp(const double first_valid_timestamp) {
    first_valid_timestamp_ = first_valid_timestamp;
  }

  std::vector<world::map::XodrRoadId> GetRoadCorridorRoadIds() const {
    return road_corridor_road_ids_;
  }

  world::map::XodrDrivingDirection GetRoadCorridorDrivingDirection() const {
    return road_corridor_driving_direction_;
  }

  void SetRoadCorridorRoadIds(const std::vector<world::map::XodrRoadId>& road_corridor_road_ids) {
    road_corridor_road_ids_ = road_corridor_road_ids;
  }

  void SetRoadCorridorDrivingDirection(const world::map::XodrDrivingDirection& driving_direction) {
    road_corridor_driving_direction_ = driving_direction;

  void SetSensedWorld(const ObservedWorldPtr& observed_world) {
    sensed_world_ = observed_world;
  }

  ObservedWorldPtr GetSensedWorld() const {
    return sensed_world_;
  }

  virtual std::shared_ptr<Object> Clone() const;

 private:
  BehaviorModelPtr behavior_model_;
  DynamicModelPtr dynamic_model_;
  ExecutionModelPtr execution_model_;
  RoadCorridorPtr road_corridor_;
  StateActionHistory history_;
  uint32_t max_history_length_;
  GoalDefinitionPtr goal_definition_;

  double first_valid_timestamp_;
  std::vector<world::map::XodrRoadId> road_corridor_road_ids_;
  world::map::XodrDrivingDirection road_corridor_driving_direction_;

  ObservedWorldPtr sensed_world_;
};

typedef std::shared_ptr<Agent> AgentPtr;

}  // namespace objects
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_OBJECTS_AGENT_HPP_
