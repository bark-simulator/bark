// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_EVALUATOR_RSS_HPP_
#define BARK_WORLD_EVALUATION_EVALUATOR_RSS_HPP_

#include <limits>
#include <memory>
#include <string>

#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/evaluation/rss_interface.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace world {
namespace evaluation {

class EvaluatorRss : public BaseEvaluator {
 public:
  EvaluatorRss() : agent_id_(std::numeric_limits<AgentId>::max()) {}

  explicit EvaluatorRss(
      const AgentId& agent_id, const std::string& opendrive_file_name,
      const std::vector<float>& default_vehicle_dynamics =
          std::vector<float>{3.5, -8., -4., -3., 0.2, -0.8, 0.1, 1.},
      const std::unordered_map<AgentId, std::vector<float>>&
          agents_vehicle_dynamics =
              std::unordered_map<AgentId, std::vector<float>>(),
      const float& discretize_step=1., const float& checking_relevent_range = 1.,
      const float& route_predict_range = 50.)
      : agent_id_(agent_id),
        rss_(opendrive_file_name, default_vehicle_dynamics,
             agents_vehicle_dynamics, discretize_step, checking_relevent_range,
             route_predict_range) {}

  virtual ~EvaluatorRss() {}

  // Returns a boolean indicating the safety response of the specified agent.
  // True if for each nearby agents, at least one of the all possible RSS
  // situations is safe, false if unsafe, uninitialized (none in python) if no
  // Rss check can be performed.
  virtual EvaluationReturn Evaluate(const World& world) {
    return rss_.GetSafetyReponse(world, agent_id_);
  };

  // Returns an unorder_map indicating the pairwise safety respone of the
  // specified agent to every other nearby agents. Key is AgentId of an nearby
  // agent, value is true if at least one of the all possible RSS situations
  // between the specified and the nearby agent is safe, false
  // otherwise.
  // Return empty map if no agent is nearby or no Rss check can be performed.
  virtual PairwiseEvaluationReturn PairwiseEvaluate(const World& world) {
    return rss_.GetPairwiseSafetyReponse(world, agent_id_);
  };

  // Returns an unorder_map indicating the pairwise directional safety respone
  // of the specified agent to every other nearby agents. Key is AgentId of an
  // nearby agent, value is a pair of directional safety response:
  //
  // 1. longitudinal safety response
  // 2. latitudinal safety response
  //
  // It is true if at least one of the all possible RSS situations in the
  // direction between the specified and the nearby agent is safe, false
  // otherwise, respectively.
  // Return empty map if no agent is nearby or no Rss check can be performed.
  virtual PairwiseDirectionalEvaluationReturn PairwiseDirectionalEvaluate(
      const World& world) {
    return rss_.GetPairwiseDirectionalSafetyReponse(world, agent_id_);
  };

 private:
  AgentId agent_id_;
  RssInterface rss_;
};
}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_EVALUATOR_RSS_HPP_
