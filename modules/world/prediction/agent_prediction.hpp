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
    explicit AgentPrediction(const Agent &agent, const std::vector<MotionHypothesis> &motion_hypotheses) : Agent(agent), motion_hypotheses_(motion_hypotheses) {}

    AgentId get_agent_id() const { return Agent::get_agent_id(); }
    geometry::Polygon get_shape() const { return Agent::get_shape(); }
    std::vector<MotionHypothesis> get_motion_hypotheses() const { return motion_hypotheses_; }

  private:
    std::vector<MotionHypothesis> motion_hypotheses_;
};

} // prediction
} // world
} // modules

#endif // MODULES_WORLD_PREDICTION_AGENT_PREDICTION_HPP_
