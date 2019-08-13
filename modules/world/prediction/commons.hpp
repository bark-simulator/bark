#ifndef MODULES_WORLD_PREDICTION_COMMONS_HPP_
#define MODULES_WORLD_PREDICTION_COMMONS_HPP_

#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/world/opendrive/lane.hpp"


namespace modules {
namespace world {
namespace prediction {

using models::dynamic::State;
using models::dynamic::StateDefinition;

typedef Eigen::Matrix<float, StateDefinition::MIN_STATE_SIZE, StateDefinition::MIN_STATE_SIZE> StateCovariance;

struct StochasticState{
  State mean;
  StateCovariance covariance;
};

typedef uint32_t HypothesisId;

struct MotionHypothesis {
  HypothesisId id;
  float likelihood;
  std::vector<opendrive::LaneId> following_lane;
  StochasticState state;
};

} // namespace prediction
} // namespace world
} // namespace modules

#endif // MODULES_WORLD_PREDICTION_COMMONS_HPP_
