// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_LABELS_SAFE_DISTANCE_LABEL_FUNCTION_HPP_
#define MODULES_WORLD_EVALUATION_LABELS_SAFE_DISTANCE_LABEL_FUNCTION_HPP_

#include <string>
#include <vector>

#include "modules/world/evaluation/labels/base_label_function.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/objects/object.hpp"

namespace modules {
namespace world {
namespace evaluation {

using modules::commons::transformation::FrenetPosition;
using modules::world::objects::AgentPtr;

class SafeDistanceLabelFunction : public BaseLabelFunction {
 public:
  SafeDistanceLabelFunction(const std::string& label_str, bool to_rear,
                            double delta, double a_e, double a_o);
  std::vector<LabelMap::value_type> Evaluate(
      const world::ObservedWorld& observed_world) const override;

 private:
  bool CheckSafeDistance(const AgentPtr& rear_agent,
                         const AgentPtr& front_agent,
                         const FrenetPosition& frenet_dist, double a_r,
                         double a_f) const;
  inline double CalcVelFrontStar(double v_f, double a_f) const;
  inline double CalcSafeDistance0(double v_r, double a_r) const;
  inline double CalcSafeDistance1(double v_r, double v_f, double a_r,
                                  double a_f) const;
  inline double CalcSafeDistance2(double v_r, double v_f, double a_r,
                                  double a_f) const;
  inline double CalcSafeDistance3(double v_r, double v_f, double a_r,
                                  double a_f) const;

  bool to_rear_;
  double delta_;  //! Reaction time
  double a_e_;    //! Max. deceleration of ego
  double a_o_;    //! Max. deceleration of front/rear agent (why rear agent?)
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_LABELS_SAFE_DISTANCE_LABEL_FUNCTION_HPP_
