#ifndef MODULES_MODELS_BEHAVIOR_NOP_NOP_HPP_
#define MODULES_MODELS_BEHAVIOR_NOP_NOP_HPP_

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/observed_world.hpp"


namespace modules {
namespace models {
namespace behavior {

class BehaviorNOP : public BehaviorModel {
  public:
    explicit BehaviorNOP(commons::Params *params) : BehaviorModel(params) {}
    virtual ~BehaviorNOP() {}

    Trajectory Plan(float delta_time, const world::ObservedWorld &observed_world) {
      Trajectory traj(1, static_cast<int>(dynamic::StateDefinition::MIN_STATE_SIZE));

      traj.row(0) = observed_world.current_ego_state();
      traj(0, dynamic::StateDefinition::TIME_POSITION) = observed_world.get_world_time() + delta_time;

      set_last_trajectory(traj);
      return traj;
    }

    BehaviorModel *Clone() const {
      return new BehaviorNOP(get_params());
    }
};

} // namespace behavior
} // namespace models
} // namespace modules

#endif // MODULES_MODELS_BEHAVIOR_NOP_NOP_HPP_