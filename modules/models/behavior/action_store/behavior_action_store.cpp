// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/action_store/behavior_action_store.hpp


namespace modules {
namespace models {
namespace behavior {

using dynamic::Trajectory;


ActionHash BehaviorActionStore::Store(const Action& action, const Trajectory& trajectory) {
  const auto action_hash = ActionToHash(action);
  auto it = trajectory_store_.find(action_hash);
  if(it == trajectory_store_.end()) {
    trajectory_store_.emplace(std::make_pair(action_hash, trajectory));
  } else {
    LOG(WARNING) << "Storing action which is already stored.";
  }
  return action_hash;
}

const Trajectory& BehaviorActionStore::Retrieve(const ActionHash& action_hash) const {
  auto it = trajectory_store_.find(action_hash);
  if(it != trajectory_store_.end()) {
    return it->second;
  } else {
    LOG(ERROR) << "Could not retrieve trajectory with hash " << action_hash <<
      "Returning empty trajectory.";
    return Trajectory();
  }
}
