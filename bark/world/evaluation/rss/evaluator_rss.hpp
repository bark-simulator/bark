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
#include "bark/geometry/polygon.hpp"

#ifdef RSS
#include "bark/world/evaluation/rss/rss_interface.hpp"
#endif
#include "bark/world/evaluation/rss/safety_polygon.hpp"

namespace bark {
namespace world {
namespace evaluation {

using geometry::Polygon;

class EvaluatorRSS : public BaseEvaluator {
 public:
  EvaluatorRSS() {}
#ifdef RSS
  explicit EvaluatorRSS(const AgentId& agent_id,
                        const commons::ParamsPtr& params)
      : agent_id_(agent_id),
        rss_(params->GetString("EvaluatorRss::MapFilename", "Map path", ""),
             params) {}
  explicit EvaluatorRSS(const commons::ParamsPtr& params)
      : EvaluatorRSS(std::numeric_limits<AgentId>::max(), params) {}

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
    if (world.GetAgent(agent_id_)) {
      std::vector<ObservedWorld> observed_worlds =
          cloned_world->Observe({agent_id_});
      if (observed_worlds.size() > 0) {
        return rss_.GetSafetyReponse(observed_worlds[0]);
      } else {
        LOG(INFO) << "EvaluatorRSS not possible for agent " << agent_id_;
        return false;
      }
    } else {
      LOG(INFO) << "EvaluatorRSS not possible for agent " << agent_id_;
      return false;
    }
  }

  double GetSafeDistance(const ::ad::rss::state::LongitudinalRssState& rss_state) {
    return rss_state.rssStateInformation.safeDistance;
  }

  double GetSafeDistance(const ::ad::rss::state::LateralRssState& rss_state) {
    return rss_state.rssStateInformation.safeDistance;
  }

  void ComputeSafetyPolygon(
    SafetyPolygon& safe_poly, const ObservedWorld& observed_world) {
    // TODO: generate and fill safety polygon here
    // auto ego_agent = observed_world.Get
    // 1. calculate bounding box
    // 2. enlarge bounding box in longitudinal
  }

  void GenerateSafetyPolygons(const ObservedWorld& observed_world) {
    safety_polygons_.clear();
    for (auto& rss_state : rss_state_snapshot_.individualResponses) {
      SafetyPolygon safe_poly;
      safe_poly.lon_safety_distance = GetSafeDistance(rss_state.longitudinalState);
      safe_poly.lat_left_safety_distance = GetSafeDistance(rss_state.lateralStateLeft);
      safe_poly.lat_right_safety_distance = GetSafeDistance(rss_state.lateralStateRight);
      ComputeSafetyPolygon(safe_poly, observed_world);
      // std::cout << safe_poly << std::endl;
      safety_polygons_.push_back(safe_poly);
    }
  }

  virtual EvaluationReturn Evaluate(const ObservedWorld& observed_world) {
    auto result = rss_.GetSafetyReponse(observed_world);
    rss_proper_response_ = rss_.GetRSSResponse();
    rss_state_snapshot_ = rss_.GetRSSStateSnapshot();
    GenerateSafetyPolygons(observed_world);
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

  ::ad::rss::state::ProperResponse GetRSSProperResponse() const {
    return rss_proper_response_;
  }

  std::vector<SafetyPolygon> GetSafetyPolygons() const {
    return safety_polygons_;
  }
  
  virtual ~EvaluatorRSS() {}

 private:
  RssInterface rss_;
  ::ad::rss::state::ProperResponse rss_proper_response_;
  ::ad::rss::state::RssStateSnapshot rss_state_snapshot_;
  std::vector<SafetyPolygon> safety_polygons_;

#endif
  AgentId agent_id_;
};
}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_EVALUATOR_RSS_HPP_
