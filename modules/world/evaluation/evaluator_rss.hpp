// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// #ifndef MODULES_WORLD_EVALUATION_EVALUATOR_RSS_HPP_
// #define MODULES_WORLD_EVALUATION_EVALUATOR_RSS_HPP_

#include <memory>
#include <limits>

#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/world.hpp"
#include "modules/world/observed_world.hpp"

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"
#include <ad/map/landmark/LandmarkIdSet.hpp>
#include <ad/map/match/Object.hpp>
#include <ad/map/route/FullRoute.hpp>
#include <ad/rss/core/RssCheck.hpp>
#include <ad/rss/map/RssSceneCreation.hpp>
#include <ad/rss/situation/SituationSnapshot.hpp>
#include <ad/rss/state/ProperResponse.hpp>
#include <ad/rss/state/RssStateSnapshot.hpp>

namespace modules {
namespace world {
namespace evaluation {

class EvaluatorRss : public BaseEvaluator {
 public:
  EvaluatorRss() :
    agent_id_(std::numeric_limits<AgentId>::max()) {}
  explicit EvaluatorRss(const AgentId& agent_id) :
    agent_id_(agent_id) {}
  virtual ~EvaluatorRss() { }
  virtual EvaluationReturn Evaluate(const world::World& world);

 private:
  AgentId agent_id_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

// #endif  // MODULES_WORLD_EVALUATION_EVALUATOR_RSS_HPP_
