// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_EVALUATOR_RSS_HPP_
#define BARK_WORLD_EVALUATION_EVALUATOR_RSS_HPP_

#include <limits>
#include <memory>
#include <string>

#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/world.hpp"

#include "bark/world/evaluation/rss/rss_interface.hpp"

namespace bark {
namespace world {
namespace evaluation {

class EvaluatorRss : public BaseEvaluator {
 public:
  EvaluatorRss() : agent_id_(std::numeric_limits<AgentId>::max()) {}

  explicit EvaluatorRss(
      const AgentId& agent_id, const std::string& opendrive_file_name,
      const std::vector<float>& default_vehicle_dynamics,
      const std::unordered_map<AgentId, std::vector<float>>&
          agents_vehicle_dynamics =
              std::unordered_map<AgentId, std::vector<float>>(),
      const float& checking_relevent_range = 1.,
      const float& route_predict_range = 50.)
      : agent_id_(agent_id),
        rss_(opendrive_file_name, default_vehicle_dynamics,
             agents_vehicle_dynamics, checking_relevent_range,
             route_predict_range) {}

  explicit EvaluatorRss(const AgentId& agent_id,
                        const commons::ParamsPtr& params)
      : agent_id_(agent_id),
        rss_(params->GetString("EvalutaorRss::MapFilename",
                               "Map path for loading into Rss", ""),
             params->GetListFloat(
                 "EvalutaorRss::DefaultVehicleDynamics",
                 "The default values of the vehicle dynamics using in Rss",
                 std::vector<float>()),
             params->GetMapAgentIdListFloat(
                 "EvalutaorRss::SpecificAgentVehicleDynamics",
                 "The values of the vehicle dynamics of a specific value",
                 std::unordered_map<AgentId, std::vector<float>>()),
             params->GetReal("EvalutaorRss::CheckingRelevantRange",
                             "Controlling the searching distance between the "
                             "evaluating agent "
                             "and other agents to perform RSS check",
                             1),
             params->GetReal("EvalutaorRss::RoutePredictRange",
                             "Describle the distance for returning all routes "
                             "having less than the distance, will be used when "
                             "a route to the goal cannnot be found",
                             50)) {}

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

  virtual ~EvaluatorRss() {}

 private:
  AgentId agent_id_;

  RssInterface rss_;
};
}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_EVALUATOR_RSS_HPP_
