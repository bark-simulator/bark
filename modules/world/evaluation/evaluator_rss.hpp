// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_EVALUATOR_RSS_HPP_
#define MODULES_WORLD_EVALUATION_EVALUATOR_RSS_HPP_

#include <limits>
#include <memory>
#include <string>

#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/evaluation/rss_interface.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace world {
namespace evaluation {

class EvaluatorRss : public BaseEvaluator {
 public:
  EvaluatorRss() : agent_id_(std::numeric_limits<AgentId>::max()) {}
  explicit EvaluatorRss(const AgentId& agent_id,
                        const std::string& opendrive_file_name)
      : agent_id_(agent_id), rss_(opendrive_file_name) {}
  virtual ~EvaluatorRss() {}
  virtual EvaluationReturn Evaluate(const World& world) {
    return rss_.GetSafetyReponse(world, agent_id_);
  };
  virtual PairwiseEvaluationReturn PairwiseEvaluate(const World& world){
    return rss_.GetPairwiseSafetyReponse(world, agent_id_);
  };

 private:
  AgentId agent_id_;
  RssInterface rss_;
};
}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_EVALUATOR_RSS_HPP_
