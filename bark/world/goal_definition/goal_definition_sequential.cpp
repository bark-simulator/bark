// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/goal_definition/goal_definition_sequential.hpp"
#include "bark/world/objects/agent.hpp"

namespace bark {
namespace world {
namespace goal_definition {

bool GoalDefinitionSequential::AtGoal(
    const bark::world::objects::Agent& agent) {
  BARK_EXPECT_TRUE(!sequential_goals_.empty());
  // First goal reached?
  if (last_sequential_goal_reached_ == NO_GOAL_REACHED) {
    if (sequential_goals_[0]->AtGoal(agent)) {
      last_sequential_goal_reached_ = 0;
    }
    // Check if next goal reached
  } else {
    // But first check if there are more goals
    if (sequential_goals_.size() - 1 >= last_sequential_goal_reached_ + 1) {
      if (sequential_goals_[last_sequential_goal_reached_ + 1]->AtGoal(agent)) {
        last_sequential_goal_reached_ += 1;
      }
    }
  }

  // If we arived at the last goal then at goal = true
  if (last_sequential_goal_reached_ == sequential_goals_.size() - 1) {
    return true;
  } else {
    return false;
  }
}

GoalDefinitionPtr GoalDefinitionSequential::GetCurrentGoal() const {
  BARK_EXPECT_TRUE(!sequential_goals_.empty());
  if (last_sequential_goal_reached_ == NO_GOAL_REACHED)
    return sequential_goals_.at(0);
  return sequential_goals_.at(last_sequential_goal_reached_);
}

GoalDefinitionPtr GoalDefinitionSequential::GetNextGoal() const {
  BARK_EXPECT_TRUE(!sequential_goals_.empty());
  if (last_sequential_goal_reached_ == NO_GOAL_REACHED)
    return sequential_goals_.at(0);

  if (last_sequential_goal_reached_ + 1 < sequential_goals_.size())
    return sequential_goals_.at(last_sequential_goal_reached_ + 1);
  return nullptr;
}

}  // namespace goal_definition
}  // namespace world
}  // namespace bark
