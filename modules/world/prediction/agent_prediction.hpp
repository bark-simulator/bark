#ifndef MODULES_WORLD_PREDICTION_AGENT_PREDICTION_HPP_
#define MODULES_WORLD_PREDICTION_AGENT_PREDICTION_HPP_

#include "modules/world/objects/agent.hpp"
#include "modules/world/prediction/commons.hpp"


namespace modules {
namespace world {
namespace prediction {

using models::dynamic::StateDefinition;
using objects::Agent;
using objects::AgentId;

class AgentPrediction : protected Agent {
  public:
    explicit AgentPrediction(const Agent &agent) : Agent(agent) {}

    AgentId get_agent_id() const { return Agent::get_agent_id(); }
    geometry::Polygon get_shape() const { return Agent::get_shape(); }
    std::map<HypothesisId, MotionHypothesis> get_motion_hypotheses() const { return motion_hypotheses_; }

    void add_hypothesis(const HypothesisId hypothesis_id, const MotionHypothesis &motion_hypothesis) {
      motion_hypotheses_.insert(std::map<HypothesisId, MotionHypothesis>::value_type(hypothesis_id, motion_hypothesis));
    }

    void update_hypothesis(const HypothesisId hypothesis_id, const StochasticState &stochastic_state) {
      motion_hypotheses_.at(hypothesis_id).states.push_back(stochastic_state);
    }

  private:
    std::map<HypothesisId, MotionHypothesis> motion_hypotheses_;
};

} // prediction
} // world
} // modules

#endif // MODULES_WORLD_PREDICTION_AGENT_PREDICTION_HPP_
