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

#ifdef RSS
#include "bark/world/evaluation/rss/rss_interface.hpp"
#endif

namespace bark {
namespace world {
namespace evaluation {

class EvaluatorRSS : public BaseEvaluator {
 public:
  EvaluatorRSS() : agent_id_(std::numeric_limits<AgentId>::max()) {}
#ifdef RSS
  explicit EvaluatorRSS(
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

  explicit EvaluatorRSS(const AgentId& agent_id,
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
  // True if for each nearby agents, at least one of the two directional RSS
  // situations (longitude and lateral) is safe, false if unsafe, uninitialized
  // (none in python) if rss check can not be performed (only in rare cases).
  // A directional RSS situation considers only the safety in that direction.
  //
  // For example, if the ego agent is following another agent in the same lane
  // at a safe distance, the longitudinal RSS situtation is safe but the
  // lateral one is unsafety.
  virtual EvaluationReturn Evaluate(const World& world) {
    WorldPtr cloned_world = world.Clone();
    ObservedWorld observed_world = cloned_world->Observe({agent_id_})[0];
    return rss_.GetSafetyReponse(observed_world);
  };

  virtual EvaluationReturn Evaluate(const ObservedWorld& observed_world) {
    return rss_.GetSafetyReponse(observed_world);
  };

  // Returns an unorder_map indicating the pairwise safety respone of the
  // specified agent to every other nearby agents. Key is AgentId of an nearby
  // agent, value is true if at least one of the two directional RSS
  // situations between the specified and the nearby agent is safe, false
  // otherwise.
  // Return empty map if no agent is nearby or no Rss check can be performed.
  virtual PairwiseEvaluationReturn PairwiseEvaluate(const World& world) {
    WorldPtr cloned_world = world.Clone();
    ObservedWorld observed_world = cloned_world->Observe({agent_id_})[0];
    return rss_.GetPairwiseSafetyReponse(observed_world);
  };

  virtual PairwiseEvaluationReturn PairwiseEvaluate(
      const ObservedWorld& observed_world) {
    return rss_.GetPairwiseSafetyReponse(observed_world);
  };

  // Returns an unorder_map indicating the pairwise directional safety respone
  // of the specified agent to every other nearby agents. Key is AgentId of an
  // nearby agent, value is a pair of directional safety response:
  //
  // 1. longitudinal safety response
  // 2. lateral safety response
  //
  // It is true if at least one of the two directional RSS situations between
  // the specified and the nearby agent is safe, false otherwise, respectively
  // in each direction.
  // Return empty map if no agent is nearby or no Rss check can be performed.
  virtual PairwiseDirectionalEvaluationReturn PairwiseDirectionalEvaluate(
      const World& world) {
    WorldPtr cloned_world = world.Clone();
    ObservedWorld observed_world = cloned_world->Observe({agent_id_})[0];
    return rss_.GetPairwiseDirectionalSafetyReponse(observed_world);
  };

  virtual PairwiseDirectionalEvaluationReturn PairwiseDirectionalEvaluate(
      const ObservedWorld& observed_world) {
    return rss_.GetPairwiseDirectionalSafetyReponse(observed_world);
  };

  virtual ~EvaluatorRSS() {}

 private:
  RssInterface rss_;
#endif
  AgentId agent_id_;
};
}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_EVALUATOR_RSS_HPP_
