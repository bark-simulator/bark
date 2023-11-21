// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_LTL_LABELS_SAFE_DISTANCE_LABEL_FUNCTION_HPP_
#define BARK_WORLD_EVALUATION_LTL_LABELS_SAFE_DISTANCE_LABEL_FUNCTION_HPP_

#include <string>
#include <vector>

#include "bark/world/evaluation/ltl/label_functions/base_label_function.hpp"
#include "bark/commons/transformation/frenet_state.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/objects/object.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace world {
namespace evaluation {

using bark::commons::transformation::FrenetPosition;
using bark::world::objects::AgentPtr;
using bark::world::FrontRearAgents;

class SafeDistanceLabelFunction : public BaseLabelFunction {
 public:
  SafeDistanceLabelFunction(const std::string& label_str, bool to_rear,
                            double delta_ego, double delta_others, double a_e, double a_o,
                            bool consider_crossing_corridors,
                            unsigned int max_agents_for_crossing,
                            bool use_frac_param_from_world,
                            double lateral_difference_threshold,
                            double angle_difference_threshold,
                            bool check_lateral_dist);
  virtual LabelMap Evaluate(const world::ObservedWorld& observed_world) const override;

  bool EvaluateEgoCorridor(const world::ObservedWorld& observed_world) const;
  bool EvaluateCrossingCorridors(const world::ObservedWorld& observed_world) const;

  bool IsOncomingVehicle(const bark::world::objects::AgentPtr& front_agent,
           const bark::world::objects::AgentPtr& rear_agent) const;

  bool GetToRear() const { return to_rear_; }
  double GetDeltaOthers() const { return delta_others_; }
  double GetDeltaEgo() const { return delta_ego_; }
  double GetMaxDecelEgo() const { return a_e_; }
  double GetMaxDecelOther() const { return a_o_; }
  bool GetUseFracLateralOffsetParam() const {
    return use_frac_param_from_world_;
  }
  double GetLateralDifferenceThreshold() const { return lateral_difference_threshold_; }
  double GetAngleDifferenceThreshold() const { return lateral_difference_threshold_; }
  bool GetConsiderCrossingCorridors() const { return consider_crossing_corridors_; }
  unsigned int GetMaxAgentsForCrossing() const { return max_agents_for_crossing_; }
  bool GetCheckLateralDist() const { return check_lateral_dist_; }

  virtual bool CheckSafeDistanceLongitudinal(
    const float v_f, const float v_r, const float dist,
    const double a_r,  const double a_f, const double delta) const;

  virtual bool CheckSafeDistanceLateral(
    const float v_f_lat, const float v_r_lat, const float dist_lat,
    const double a_r_lat,  const double a_f_lat, const double delta1,
     const double delta2) const;

 protected:
  inline double CalcVelFrontStar(double v_f, double a_f, double delta) const {
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

  inline double CalcSafeDistance0(double v_r, double a_r, double delta) const { 
    return v_r * delta - pow(v_r, 2) / (2.0 * a_r);
  }

  inline double CalcSafeDistance1(double v_r, double v_f, double a_r,
                                  double a_f, double delta) const {
    return v_r * delta - pow(v_r, 2) / (2.0 * a_r) + pow(v_f, 2) / (2.0 * a_f);
  }

  inline double CalcSafeDistance2(double v_r, double v_f, double a_r,
                                  double a_f, double delta) const {
  double sqrt_numerator = v_f + a_f * delta - v_r;
    return pow(sqrt_numerator, 2) / (2.0 * (a_f - a_r)) - v_f * delta - 
            0.5 * a_f * pow(delta, 2) + v_r * delta;
  }

  inline double CalcSafeDistance3(double v_r, double v_f, double a_r,
                                  double a_f, double delta) const {
    return v_r * delta - pow(v_r, 2) / (2.0 * a_r) - v_f * delta -
            a_f * pow(delta, 2) / 2.0;
  }

 private:  
  bool CheckSafeDistanceLongitudinal(FrontRearAgents& fr_agents, const AgentPtr& ego_agent) const;
  bool CheckSafeDistanceLateral(FrontRearAgents& fr_agents, const AgentPtr& ego_agent) const;

  bool to_rear_;
  double delta_ego_;  //! Reaction times
  double delta_others_;  //! Reaction times
  double a_e_;    //! Max. deceleration of ego
  double a_o_;    //! Max. deceleration of front/rear agent
  bool use_frac_param_from_world_; // Flag to use passed frac param
  double lateral_difference_threshold_;  //! Distance term for lateral offset
  double angle_difference_threshold_;  //! Max frenet angle to consider as front
  bool consider_crossing_corridors_;
  unsigned int max_agents_for_crossing_;
  bool check_lateral_dist_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_LTL_LABELS_SAFE_DISTANCE_LABEL_FUNCTION_HPP_
