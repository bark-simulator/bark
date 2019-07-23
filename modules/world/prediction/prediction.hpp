#ifndef MODULES_WORLD_PREDICTION_PREDICTION_HPP_
#define MODULES_WORLD_PREDICTION_PREDICTION_HPP_

#include "modules/commons/base_type.hpp"
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/observed_world.hpp"


namespace modules {
namespace world {
namespace prediction {

using models::behavior::BehaviorModelPtr;
using models::dynamic::StateDefinition;

typedef Eigen::Matrix<float, StateDefinition::MIN_STATE_SIZE, StateDefinition::MIN_STATE_SIZE> StateCovariance;
struct StochasticState{
  State mean;
  StateCovariance covariance;
};

class Prediction : public commons::BaseType {
  public:
    Prediction(commons::Params *params, const ObservedWorld &observed_world, const std::map<AgentId, BehaviorModelPtr> &assumed_agent_behaviors);

    void Step(const float time_step);

    StochasticState PredictAgentState(const AgentId agent_id);

  private:
    ObservedWorld observed_world_;
};

} // namespace prediction
} // namespace world
} // namespace modules

#endif // MODULES_WORLD_PREDICTION_PREDICTION_HPP_
