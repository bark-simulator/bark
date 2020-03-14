// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_ACTION_STORE_BEHAVIOR_ACTION_STORE_HPP_
#define MODULES_MODELS_BEHAVIOR_ACTION_STORE_BEHAVIOR_ACTION_STORE_HPP_

#include <boost/functional/hash.hpp>
#include <unordered_map>

#include "modules/models/behavior/behavior_model.hpp"

namespace modules {
namespace world {
class ObservedWorld;
}  
namespace models {
namespace behavior {


using dynamic::Trajectory;
using dynamic::Input;

class BehaviorActionStore : public BehaviorModel {
 public:
  BehaviorActionStore(const commons::ParamsPtr& params)
      : BehaviorModel(params),
      trajectory_store_(),
      active_behavior_() {}

  virtual ~BehaviorActionStore() {}

  ActionHash Store(const Action& action, const Trajectory& trajectory);
  Trajectory Retrieve(const ActionHash& action_hash) const;

  void MakeBehaviorActive(const ActionHash& action_hash) {
    active_behavior_ = action_hash;
  }

  virtual BehaviorModelPtr Clone() const;

  virtual Trajectory Plan(float delta_time, const modules::world::ObservedWorld& observed_world);

  private:
    std::unordered_map<ActionHash, Trajectory> trajectory_store_;
    ActionHash active_behavior_;
};

inline BehaviorModelPtr BehaviorActionStore::Clone() const {
  std::shared_ptr<BehaviorActionStore> model_ptr =
      std::make_shared<BehaviorActionStore>(*this);
  return model_ptr;
}


typedef std::shared_ptr<BehaviorActionStore> BehaviorActionStorePtr;

struct ActionHasher : public boost::static_visitor<std::size_t>
{
    template<typename T>
    std::size_t operator()(const T& action) const { return boost::hash<T>()(action); }
    std::size_t operator()(const Input& x) const { return std::size_t(); }
};

inline ActionHash ActionToHash(const Action& action) {
  return boost::apply_visitor(ActionHasher(), action);
}


}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_ACTION_STORE_BEHAVIOR_ACTION_STORE_HPP_
