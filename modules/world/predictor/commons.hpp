#ifndef MODULES_WORLD_PREDICTION_COMMONS_HPP_
#define MODULES_WORLD_PREDICTION_COMMONS_HPP_

#include "modules/geometry/polygon.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/world/opendrive/lane.hpp"
#include "modules/world/objects/agent.hpp"


namespace modules {
namespace world {
namespace prediction {

typedef Eigen::Matrix<float, models::dynamic::StateDefinition::MIN_STATE_SIZE, models::dynamic::StateDefinition::MIN_STATE_SIZE> StateCovariance;

struct StochasticState{
  models::dynamic::State mean;
  StateCovariance covariance;
};

typedef uint32_t HypothesisId;

struct MotionHypothesis {
  HypothesisId id;
  float likelihood;
  std::vector<StochasticState> states;
};

struct AgentPrediction {
  objects::AgentId agent_id_;
  geometry::Polygon agent_shape_;
  std::map<HypothesisId, MotionHypothesis> motion_hypotheses_;
};
typedef std::map<objects::AgentId, AgentPrediction> AgentPredictions;

} // namespace prediction
} // namespace world
} // namespace modules

#endif // MODULES_WORLD_PREDICTION_COMMONS_HPP_
