#ifndef MODULES_WORLD_PREDICTION_PREDICTION_HPP_
#define MODULES_WORLD_PREDICTION_PREDICTION_HPP_

#include "modules/commons/base_type.hpp"
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/world/prediction/agent_prediction.hpp"


namespace modules {
namespace world {
namespace prediction {

using opendrive::LaneId;
using models::behavior::BehaviorModelPtr;

class Prediction : public commons::BaseType {
  public:
    Prediction(commons::Params *params, const ObservedWorld &observed_world, const std::map<AgentId, BehaviorModelPtr> &assumed_agent_behaviors);

    void Step(const float time_step);

    std::map<AgentId, AgentPrediction> get_predictions_for_all_agents();

  private:
    std::vector<std::list<LaneId>> FindPossibleGoalLanes(const geometry::Point2d &position, const MapInterfacePtr map_interface) const;
    ObservedWorld observed_world_;
    std::map<AgentId, std::vector<AgentId>> real_agents_to_predictions_;
    std::map<AgentId, AgentPrediction> predictions_for_all_agents_;
};

} // namespace prediction
} // namespace world
} // namespace modules

#endif // MODULES_WORLD_PREDICTION_PREDICTION_HPP_
