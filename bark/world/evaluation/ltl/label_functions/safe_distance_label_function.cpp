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

namespace bark {
namespace world {
namespace evaluation {

using bark::models::dynamic::StateDefinition;

SafeDistanceLabelFunction::SafeDistanceLabelFunction(const std::string& label_str, bool to_rear,
                            double delta_ego, double delta_others, double a_e, double a_o,
                            bool consider_crossing_corridors,
                            unsigned int max_agents_for_crossing,
                            bool use_frac_param_from_world,
                            double lateral_difference_threshold)
    : BaseLabelFunction(label_str),
      to_rear_(to_rear),
      delta_ego_(delta_ego),
      delta_others_(delta_others),
      a_e_(a_e),
      a_o_(a_o),
      use_frac_param_from_world_(use_frac_param_from_world),
      lateral_difference_threshold_(lateral_difference_threshold),
      consider_crossing_corridors_(consider_crossing_corridors),
      max_agents_for_crossing_(max_agents_for_crossing) {}

LabelMap SafeDistanceLabelFunction::Evaluate(const world::ObservedWorld& observed_world) const {
  if(!EvaluateEgoCorridor(observed_world)) {
    return {{GetLabel(), false}};; 
  }
  bool safe_dist = true;
  if(consider_crossing_corridors_) {
    safe_dist = EvaluateCrossingCorridors(observed_world);
  }
  return {{GetLabel(), safe_dist}};
}

bool SafeDistanceLabelFunction::IsOncomingVehicle(const bark::world::AgentPtr& front_agent,
           const bark::world::AgentPtr& rear_agent) const {
  // Angular deviation allowed around orientation difference of 180
  using bark::models::dynamic::StateDefinition;
  const double max_angular_deviation = 5.0 / 360.0 * bark::geometry::B_2PI;
  const double lower_angular_range = bark::geometry::B_PI - max_angular_deviation;
  const double upper_angular_range = bark::geometry::B_PI + max_angular_deviation;
  const double angle_difference = front_agent->GetCurrentState()[StateDefinition::THETA_POSITION] - 
                                  rear_agent->GetCurrentState()[StateDefinition::THETA_POSITION];
  const double normed_angle = bark::geometry::Norm0To2PI(angle_difference);
  const bool is_oncoming = normed_angle < upper_angular_range &&
         normed_angle > lower_angular_range;
  VLOG(5) << "oncoming?:" << is_oncoming << ", lar=" << lower_angular_range << ", uar=" 
          << upper_angular_range << ", na=" << normed_angle;
  return is_oncoming;
}


bool SafeDistanceLabelFunction::EvaluateEgoCorridor(
    const world::ObservedWorld& observed_world) const {
  auto ego = std::const_pointer_cast<Agent>(observed_world.GetEgoAgent());
  auto lane_corr =
      ego->GetRoadCorridor()->GetNearestLaneCorridor(ego->GetCurrentPosition());
  double frac;
  if (use_frac_param_from_world_) {
    frac = observed_world.GetLateralDifferenceThreshold();
  } else {
    frac = lateral_difference_threshold_;
  }
  auto fr_agents = observed_world.GetAgentFrontRearForId(
        ego->GetAgentId(), lane_corr, frac);
  bool distance_safe = true;

  if (to_rear_ && fr_agents.rear.first) {
    double v_r = fr_agents.rear.second.to.vlon;
    double v_f = ego->GetCurrentState()(StateDefinition::VEL_POSITION);
    double dist = fr_agents.front.second.lon;;
    distance_safe = CheckSafeDistance(v_f, v_r,
                                      dist, a_o_, a_e_, delta_others_);
  } 
  
  if (fr_agents.front.first) {
    if (IsOncomingVehicle(fr_agents.front.first, ego)) {
      return true;
    }
    double v_r = ego->GetCurrentState()(StateDefinition::VEL_POSITION);
    double v_f = fr_agents.front.second.to.vlon;
    double dist = fr_agents.front.second.lon;
    distance_safe = distance_safe && CheckSafeDistance(v_f, v_r,
                                      dist, a_e_, a_o_, delta_ego_);
  }
  return distance_safe;
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
            nearest_agent.second->GetAgentId(), lane_corridor, frac);

    // front agent is ego agent
    double v_rear;
    double v_front; 
    double dist; 
    double delta;
    VLOG(5) << "Nearest = " << nearest_agent.first;
    if(fr_agents.front.first && fr_agents.front.first->GetAgentId() == observed_world.GetEgoAgentId()) {
      v_rear = nearest_agent.second->GetCurrentState()(StateDefinition::VEL_POSITION);
      v_front = fr_agents.front.second.to.vlon;
      dist = fr_agents.front.second.lon;
      delta = delta_others_;
    } else if(fr_agents.rear.first && fr_agents.rear.first->GetAgentId() == observed_world.GetEgoAgentId()) {
      v_rear = fr_agents.rear.second.to.vlon;
      v_front = nearest_agent.second->GetCurrentState()(StateDefinition::VEL_POSITION);
      dist = std::abs(fr_agents.rear.second.lon);
      delta = delta_ego_;
    } else {
      continue;
    }

    VLOG(5) << "Checking dist for " << nearest_agent.first << ", ve=" << v_front << ", vr=" << v_rear
         << ", d=" << dist << ", a_o=" << a_o_ << ", a_e=" << a_e_; 
    bool distance_safe = CheckSafeDistance(v_front, v_rear,
                                    dist, a_o_, a_e_, delta);
    if(!distance_safe) return distance_safe;
  }
  return true;
}

bool SafeDistanceLabelFunction::CheckSafeDistance(
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
  double safe_dist_0 = CalcSafeDistance0(v_r, a_r, delta);
  double safe_dist_1 = CalcSafeDistance1(v_r, v_f, a_r, a_f, delta);
  double safe_dist_2 = CalcSafeDistance2(v_r, v_f, a_r, a_f, delta);
  double safe_dist_3 = CalcSafeDistance3(v_r, v_f, a_r, a_f, delta);

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

double SafeDistanceLabelFunction::CalcVelFrontStar(const double v_f,
                                                   const double a_f,
                                                   double delta) const {
  // see Theorem 4 in "Formalising and Monitoring Traffic Rules for Autonomous
  // Vehicles in Isabelle/HOL"
  double v_f_star;
  double t_stop_f = -v_f / a_f;
  if (delta <= t_stop_f) {
    v_f_star = v_f + a_f * delta;
  } else {
    v_f_star = 0.0;
  }
  return v_f_star;
}

double SafeDistanceLabelFunction::CalcSafeDistance0(const double v_r,
                                                    const double a_r,
                                                    const double delta) const {
  return v_r * delta - pow(v_r, 2) / (2.0 * a_r);
}

double SafeDistanceLabelFunction::CalcSafeDistance1(const double v_r,
                                                    const double v_f,
                                                    const double a_r,
                                                    const double a_f,
                                                    const double delta) const {
  return v_r * delta - pow(v_r, 2) / (2.0 * a_r) + pow(v_f, 2) / (2.0 * a_f);
}

double SafeDistanceLabelFunction::CalcSafeDistance2(const double v_r,
                                                    const double v_f,
                                                    const double a_r,
                                                    const double a_f,
                                                    const double delta) const {
  double sqrt_numerator = v_f + a_f * delta - v_r;
  return pow(sqrt_numerator, 2) / (2.0 * (a_f - a_r)) - v_f * delta -
         0.5 * a_f * pow(delta, 2) + v_r * delta;
}

double SafeDistanceLabelFunction::CalcSafeDistance3(const double v_r,
                                                    const double v_f,
                                                    const double a_r,
                                                    const double a_f,
                                                    const double delta) const {
  return v_r * delta - pow(v_r, 2) / (2.0 * a_r) - v_f * delta -
         a_f * pow(delta, 2) / 2.0;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark
