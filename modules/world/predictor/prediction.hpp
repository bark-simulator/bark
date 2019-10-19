#ifndef MODULES_WORLD_PREDICTION_PREDICTION_HPP_
#define MODULES_WORLD_PREDICTION_PREDICTION_HPP_

#include "modules/commons/base_type.hpp"
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/world/predictor/commons.hpp"


namespace modules {
namespace world {
namespace prediction {

using opendrive::LaneId;
using models::behavior::BehaviorModelPtr;

class Prediction : public commons::BaseType {
  public:
    Prediction(commons::Params *params, const ObservedWorld &observed_world, const float time_step);

    void Predict(const uint n_steps, const std::map<AgentId, std::pair<BehaviorModelPtr, float>> &assumed_agent_behaviors);

    AgentPredictions get_predictions_for_all_agents() { return agent_predictions_; }

  private:
    ObservedWorld observed_world_;
    AgentPredictions agent_predictions_;

    float time_step_;
};

} // namespace prediction
} // namespace world
} // namespace modules

#endif // MODULES_WORLD_PREDICTION_PREDICTION_HPP_
