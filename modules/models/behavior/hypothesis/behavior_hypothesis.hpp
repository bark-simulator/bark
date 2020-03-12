// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_HYPOTHESIS_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_HYPOTHESIS_HPP_

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/commons/distribution/distribution.hpp"

namespace modules {
namespace world {
namespace objects {
class Agent;
typedef std::shared_ptr<Agent> AgentPtr;
typedef unsigned int AgentId;
}  // namespace objects
class ObservedWorld;
}  // namespace world
namespace models {
namespace behavior {


class BehaviorHypothesis : public virtual BehaviorModel {
  public:
    BehaviorHypothesis(const commons::ParamsPtr& params) : BehaviorModel(params) {}
    virtual ~BehaviorHypothesis() {}

    virtual modules::commons::Probability GetProbability(const Action& action,
                             const world::ObservedWorld& observed_world,
                             const AgentId& agent_id) const = 0;
};

typedef std::shared_ptr<BehaviorHypothesis> BehaviorHypothesisPtr;


}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_HYPOTHESIS_HPP_