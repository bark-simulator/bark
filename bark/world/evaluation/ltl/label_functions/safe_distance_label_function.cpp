// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "safe_distance_label_function.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/commons/transformation/frenet_state.hpp"
#include "bark/models/dynamic/single_track.hpp"

namespace bark {
namespace world {
namespace evaluation {

using bark::models::dynamic::StateDefinition;
using bark::world::objects::AgentId;
using bark::world::objects::AgentPtr;
using bark::world::FrontRearAgents;

SafeDistanceLabelFunction::SafeDistanceLabelFunction(const std::string& label_str, bool to_rear,
                            double delta_ego, double delta_others, double a_e, double a_o,
                            bool consider_crossing_corridors,
                            unsigned int max_agents_for_crossing,
                            bool use_frac_param_from_world,
                            double lateral_difference_threshold,
                            double angle_difference_threshold,
                            bool check_lateral_dist)
    : BaseLabelFunction(label_str),
      to_rear_(to_rear),
      delta_ego_(delta_ego),
      delta_others_(delta_others),
      a_e_(a_e),
      a_o_(a_o),
      use_frac_param_from_world_(use_frac_param_from_world),
      lateral_difference_threshold_(lateral_difference_threshold),
      angle_difference_threshold_(angle_difference_threshold),
      consider_crossing_corridors_(consider_crossing_corridors),
      max_agents_for_crossing_(max_agents_for_crossing),
      check_lateral_dist_(check_lateral_dist) {}

LabelMap SafeDistanceLabelFunction::Evaluate(const world::ObservedWorld& observed_world) const {
  bool safe_dist = true;
  if(consider_crossing_corridors_) {
    safe_dist = EvaluateCrossingCorridors(observed_world);
  }
  
  return {{GetLabel(), safe_dist}};
}

bool SafeDistanceLabelFunction::EvaluateCrossingCorridors(
  const world::ObservedWorld& observed_world) const {
  const auto ego_pos = observed_world.CurrentEgoPosition();
  const auto nearest_agents = observed_world.GetNearestAgents(ego_pos,
                          max_agents_for_crossing_ + 1); // one more since ego agent is included
  double frac;
  if (use_frac_param_from_world_) {
    frac = observed_world.GetLateralDifferenceThreshold();
  } else {
    frac = lateral_difference_threshold_;
  }

  for (const auto nearest_agent : nearest_agents) {
    if(nearest_agent.first == observed_world.GetEgoAgentId()) continue;
    // Find this agents front and back agent
    auto lane_corridor =
        nearest_agent.second->GetRoadCorridor()
                ->GetNearestLaneCorridor(nearest_agent.second->GetCurrentPosition());
    auto fr_agents =
        observed_world.GetAgentFrontRearForId(
            nearest_agent.second->GetAgentId(), lane_corridor, frac, angle_difference_threshold_);

    VLOG(5) << "Checking safe dist for nearest agent with id " << nearest_agent.first;
    bool distance_safe = CheckSafeDistanceLongitudinal(fr_agents, observed_world.GetEgoAgent());
    if(!distance_safe && check_lateral_dist_) {
          distance_safe = CheckSafeDistanceLateral(fr_agents, observed_world.GetEgoAgent());
    }
    if(!distance_safe) return distance_safe; // Early termination of loop over nearest agents when violated
  }
  
  return true;
}

bool SafeDistanceLabelFunction::CheckSafeDistanceLongitudinal(FrontRearAgents& fr_agents, const AgentPtr& ego_agent) const {
  double v_rear_lon, v_front_lon, dist_lon, delta_lon;
  AgentId checked_id;
  if(fr_agents.front.first && fr_agents.front.first->GetAgentId() == ego_agent->GetAgentId()) {
    // Ego agent is front agent
    checked_id = fr_agents.front.first->GetAgentId();
    v_rear_lon = fr_agents.front.second.from.vlon;
    v_front_lon = fr_agents.front.second.to.vlon;
    dist_lon = fr_agents.front.second.lon_zeroed ? 0.0 : fr_agents.front.second.lon;
    delta_lon = delta_others_;
  } else if(fr_agents.rear.first && fr_agents.rear.first->GetAgentId() == ego_agent->GetAgentId()) {
    // Ego agent is rear agent
    checked_id = fr_agents.rear.first->GetAgentId();
    v_rear_lon = fr_agents.rear.second.to.vlon;
    v_front_lon = fr_agents.rear.second.from.vlon;
    dist_lon = fr_agents.rear.second.lon_zeroed ? 0.0 : std::abs(fr_agents.rear.second.lon);
    delta_lon = delta_ego_;
  } else {
    return true;
  }

  VLOG(5) << "Checking longitudinal safety for " << checked_id << ", v_long_f=" << v_front_lon << ", v_long_r=" << v_rear_lon
        << ", d=" << dist_lon << ", a_o=" << a_o_ << ", a_e=" << a_e_; 
  
  bool distance_safe = CheckSafeDistanceLongitudinal(v_front_lon, v_rear_lon,
                                    dist_lon, a_o_, a_e_, delta_lon);
  return distance_safe;
}

bool SafeDistanceLabelFunction::CheckSafeDistanceLongitudinal(
    const float v_f, const float v_r, const float dist,
    const double a_r,  const double a_f, const double delta) const {
  if (dist < 0.0) {
    return true;
  }

  double v_f_star = CalcVelFrontStar(v_f, a_f, delta);
  double t_stop_f_star = -v_f_star / a_r;
  double t_stop_r = -v_r / a_r;
  double t_stop_f = -v_f / a_f;

  bool distance_safe;
  auto ZeroToPositive = [](double safe_dist) {
    return safe_dist < 0.0 ? 0.0 : safe_dist;
  };
  double safe_dist_0 = ZeroToPositive(CalcSafeDistance0(v_r, a_r, delta));
  double safe_dist_1 = ZeroToPositive(CalcSafeDistance1(v_r, v_f, a_r, a_f, delta));
  double safe_dist_2 = ZeroToPositive(CalcSafeDistance2(v_r, v_f, a_r, a_f, delta));
  double safe_dist_3 = ZeroToPositive(CalcSafeDistance3(v_r, v_f, a_r, a_f, delta));

  VLOG(5) << "sf0=" << safe_dist_0 << ", sf1=" << safe_dist_1 << ", sf2=" << safe_dist_2 << 
        ", sf3=" << safe_dist_3;

  if (dist > safe_dist_0 || (delta <= t_stop_f && dist > safe_dist_3)) {
    distance_safe = true;
  } else if (delta <= t_stop_f && a_f > a_r && v_f_star < v_r &&
             t_stop_r < t_stop_f_star) {
    distance_safe = dist > safe_dist_2;
  } else {
    distance_safe = dist > safe_dist_1;
  }
  return distance_safe;
}

bool SafeDistanceLabelFunction::CheckSafeDistanceLateral(FrontRearAgents& fr_agents, const AgentPtr& ego_agent) const { 
  auto GetMaxAccLat = [](const AgentPtr& agent,
                           const double& road_angle,
                           const double& max_total_acc,
                           const bool& on_left_side_of_center_line) {
    auto single_track = std::dynamic_pointer_cast<models::dynamic::SingleTrackModel>(
      agent->GetDynamicModel());
    BARK_EXPECT_TRUE(bool(single_track));
    const auto state = agent->GetCurrentState();
    const double a_max_lat = single_track->CalculateLatAccelerationMaxAtFrenetAngle(
                        state(StateDefinition::VEL_POSITION), 
                        state(StateDefinition::THETA_POSITION), 
                        road_angle, max_total_acc,
                        on_left_side_of_center_line);
    return a_max_lat;
  };

  double v_2_lat, v_1_lat, dist_lat_zeroed, dist_lat, a_1_lat, a_2_lat;
  AgentId checked_agent;
  if(fr_agents.front.first && fr_agents.front.first->GetAgentId() == ego_agent->GetAgentId()) {
    // Ego agent is front agent
    checked_agent = fr_agents.front.first->GetAgentId();
    v_1_lat = fr_agents.front.second.from.vlat; // other agent
    v_2_lat = fr_agents.front.second.to.vlat; // ego agent
    dist_lat = fr_agents.front.second.lat;
    dist_lat_zeroed = fr_agents.front.second.lat_zeroed ? 0.0 : dist_lat; 
    a_1_lat = GetMaxAccLat(fr_agents.front.first, fr_agents.front.second.from.angleRoad, a_o_, dist_lat >= 0);
    a_2_lat = GetMaxAccLat(ego_agent, fr_agents.front.second.to.angleRoad, a_e_, dist_lat >= 0);
  } else if(fr_agents.rear.first && fr_agents.rear.first->GetAgentId() == ego_agent->GetAgentId()) {
    // Ego agent is rear agent
    checked_agent = fr_agents.rear.first->GetAgentId();
    v_1_lat = fr_agents.rear.second.from.vlat; // other agent
    v_2_lat = fr_agents.rear.second.to.vlat; // ego agent
    dist_lat = fr_agents.rear.second.lat;
    dist_lat_zeroed = fr_agents.rear.second.lat_zeroed ? 0.0 : dist_lat;
    a_1_lat = GetMaxAccLat(fr_agents.rear.first, fr_agents.rear.second.from.angleRoad, a_o_, dist_lat >= 0);
    a_2_lat = GetMaxAccLat(ego_agent, fr_agents.rear.second.to.angleRoad, a_e_, dist_lat >= 0);
  } else {
    return true;
  }

  VLOG(5) << "Checking lateral safe dist for " << checked_agent << ", v_lat_e = " << v_2_lat << ", v_lat_o = " << v_1_lat << 
    ", dist_lat = " << dist_lat_zeroed << ", acc_lat_1" << a_1_lat << ", acc_lat_2" << a_2_lat; 

  double delta1 = delta_others_; // First vehicle is other agent
  double delta2 = delta_ego_;

  bool distance_safe = CheckSafeDistanceLateral(v_1_lat, v_2_lat, dist_lat_zeroed, a_1_lat,  a_2_lat, delta1, delta2);

  return distance_safe;
}

/**
 * @brief Checks lateral safe distance according to lateral safe distance formulation in
 *        S. Shalev-Shwartz, S. Shammah, and A. Shashua,
 *        “On a Formal Model of Safe and Scalable Self-driving Cars,” 2017.
 *        Lemma 4. Yet, we do not consider additional maximum lateral acceleration 
 *        within response time delta.
 * 
 * @param v_1_lat Lateral velocity from frenet state of vehicle 1
 * @param v_2_lat Lateral velocity from frenet state of vehicle 2
 *                 with respect to center line of vehicle 1
 * @param dist_lat Lateral distance between vehicle 1 and 2 (shape-based)
 * @param a_1_lat Maximum lateral accleration of vehicle 1 towards vehicle 2
 * @param a_2_lat Maximum lateral acceleration of vehicle 2 towards vehicle 1
 * @param delta1 Response times of vehicle 1
 * @param delta2 Response times of vehicle 2
 * @return true if lateral safe distance satisfied
 * @return false if lateral safe distance violated
 */
bool SafeDistanceLabelFunction::CheckSafeDistanceLateral(
    const float v_1_lat, const float v_2_lat, const float dist_lat,
    const double a_1_lat,  const double a_2_lat, const double delta1,
     const double delta2) const {
    if(dist_lat == 0.0) {
      return false;
    // Positive lateral velocity means driving from right to left with respect to center
    // Negative lateral velocity means driving from left to right with respect to center
    } else if(v_1_lat >= 0.0 && v_2_lat <= 0.0 && dist_lat < 0.0) { // Vehicles move laterally away from each other...
      // vehicle 2 to the right of vehicle 1
      return true;
    } else if (v_1_lat <= 0.0 && v_2_lat >= 0.0 && dist_lat > 0.0) {
      // vehicle 2 to the left of vehicle 1
      return true;
    } else { // vehicles move laterally towards each other or into common lateral direction
      // For convention of RSS paper, make v_1_lat be larger (e.g. positive compared to v_2_lat) ...
      float v_1_lat_non_const = v_1_lat;
      float v_2_lat_non_const = v_2_lat;

      float delta1_non_const = delta1;
      float delta2_non_const = delta2;

      float a_1_lat_non_const = a_1_lat;
      float a_2_lat_non_const = a_2_lat;

      if (v_1_lat < v_2_lat) {
        std::swap(v_1_lat_non_const, v_2_lat_non_const);
        std::swap(delta1_non_const, delta2_non_const);
        std::swap(a_1_lat_non_const, a_2_lat_non_const);
      }
      // ... and lateral distance positive 
      const double lateral_positive = std::abs(dist_lat);

      const double min_lat_safe_dist = v_1_lat_non_const*delta1_non_const 
                                        + (v_1_lat_non_const == 0.0 ? 0.0 : v_1_lat_non_const*v_1_lat_non_const / (2 * a_1_lat_non_const)) 
                                        - (v_2_lat_non_const*delta2_non_const 
                                        - (v_2_lat_non_const == 0.0 ? 0.0 : v_2_lat_non_const*v_2_lat_non_const / (2 * a_2_lat_non_const)));
      VLOG(5) << "Min lateral safe distance: " << min_lat_safe_dist;

      return lateral_positive > min_lat_safe_dist;
    }
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
