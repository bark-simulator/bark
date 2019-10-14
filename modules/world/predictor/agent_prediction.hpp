#ifndef MODULES_WORLD_PREDICTION_AGENT_PREDICTION_HPP_
#define MODULES_WORLD_PREDICTION_AGENT_PREDICTION_HPP_

#include "modules/world/objects/agent.hpp"
#include "modules/world/predictor/commons.hpp"


namespace modules {
namespace world {
namespace prediction {

using models::dynamic::StateDefinition;
using objects::Agent;
using objects::AgentId;

class AgentPrediction {
  public:
    explicit AgentPrediction(const AgentId agent_id, const geometry::Polygon &agent_shape) : agent_id_(agent_id), agent_shape_(agent_shape) {}

    AgentId get_agent_id() const { return agent_id_; }
    geometry::Polygon get_shape() const { return agent_shape_; }
    std::map<HypothesisId, MotionHypothesis> get_motion_hypotheses() const { return motion_hypotheses_; }

    void add_hypothesis(const MotionHypothesis &motion_hypothesis) {
      motion_hypotheses_.insert(std::map<HypothesisId, MotionHypothesis>::value_type(motion_hypothesis.id, motion_hypothesis));
    }

    void update_hypothesis(const HypothesisId hypothesis_id, const StochasticState &stochastic_state) {
      motion_hypotheses_.at(hypothesis_id).states.push_back(stochastic_state);
    }

  private:
    AgentId agent_id_;
    geometry::Polygon agent_shape_;
    std::map<HypothesisId, MotionHypothesis> motion_hypotheses_;
};

} // prediction
} // world
} // modules

#endif // MODULES_WORLD_PREDICTION_AGENT_PREDICTION_HPP_
