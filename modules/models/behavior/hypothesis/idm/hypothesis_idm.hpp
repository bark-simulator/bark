// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_HYPOTHESIS_hypothesis_idm_HPP_
#define MODULES_MODELS_BEHAVIOR_HYPOTHESIS_hypothesis_idm_HPP_

#include "modules/models/behavior/idm/stochastic/idm_stochastic.hpp"
#include "modules/models/behavior/hypothesis/behavior_hypothesis.hpp"

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


class BehaviorHypothesisIDM : public BehaviorIDMStochastic, public BehaviorHypothesis {
  public:
  explicit BehaviorHypothesisIDM(const commons::ParamsPtr& params);
  virtual ~BehaviorHypothesisIDM()  {}

  virtual modules::commons::Probability GetProbability(const Action& action,
                             const world::ObservedWorld& observed_world,
                             const AgentId& agent_id) const;

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  virtual Trajectory Plan(float delta_time, const ObservedWorld& observed_world) {
    return BehaviorIDMStochastic::Plan(delta_time, observed_world);
  };

  private: 
    const unsigned int num_samples_;
    const unsigned int num_buckets_;
    const Continuous1DAction buckets_upper_bound_;
    const Continuous1DAction buckets_lower_bound_; 
};


inline std::shared_ptr<BehaviorModel> BehaviorHypothesisIDM::Clone() const {
  std::shared_ptr<BehaviorHypothesisIDM> model_ptr =
      std::make_shared<BehaviorHypothesisIDM>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_HYPOTHESIS_hypothesis_idm_HPP_