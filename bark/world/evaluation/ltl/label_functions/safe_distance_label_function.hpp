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

namespace bark {
namespace world {
namespace evaluation {

using bark::commons::transformation::FrenetPosition;
using bark::world::objects::AgentPtr;

class SafeDistanceLabelFunction : public BaseLabelFunction {
 public:
  SafeDistanceLabelFunction(const std::string& label_str, bool to_rear,
                            double delta_ego, double delta_others, double a_e, double a_o,
                            bool consider_crossing_corridors,
                            unsigned int max_agents_for_crossing,
                            bool use_frac_param_from_world,
                            double lateral_difference_threshold,
                            bool check_lateral_dist);
  LabelMap Evaluate(const world::ObservedWorld& observed_world) const override;

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
  bool GetConsiderCrossingCorridors() const { return consider_crossing_corridors_; }
  unsigned int GetMaxAgentsForCrossing() const { return max_agents_for_crossing_; }
  bool GetCheckLateralDist() const { return check_lateral_dist_; }

 private:
  bool CheckSafeDistanceLongitudinal(
    const float v_f, const float v_r, const float dist,
    const double a_r,  const double a_f, const double delta) const;
  inline double CalcVelFrontStar(double v_f, double a_f, double delta) const;
  inline double CalcSafeDistance0(double v_r, double a_r, double delta) const;
  inline double CalcSafeDistance1(double v_r, double v_f, double a_r,
                                  double a_f, double delta) const;
  inline double CalcSafeDistance2(double v_r, double v_f, double a_r,
                                  double a_f, double delta) const;
  inline double CalcSafeDistance3(double v_r, double v_f, double a_r,
                                  double a_f, double delta) const;

  bool CheckSafeDistanceLateral(
    const float v_f_lat, const float v_r_lat, const float dist_lat,
    const double a_r_lat,  const double a_f_lat, const double delta1,
     const double delta2) const;

  bool to_rear_;
  double delta_ego_;  //! Reaction times
  double delta_others_;  //! Reaction times
  double a_e_;    //! Max. deceleration of ego
  double a_o_;    //! Max. deceleration of front/rear agent
  bool use_frac_param_from_world_; // Flag to use passed frac param
  double lateral_difference_threshold_;  //! Fraction term for lateral offset
  bool consider_crossing_corridors_;
  unsigned int max_agents_for_crossing_;
  bool check_lateral_dist_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_LTL_LABELS_SAFE_DISTANCE_LABEL_FUNCTION_HPP_
