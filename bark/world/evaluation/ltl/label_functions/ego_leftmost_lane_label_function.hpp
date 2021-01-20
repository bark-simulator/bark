// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_LTL_LABELS_EGO_LEFTMOST_LANE_LABEL_FUNCTION_HPP_
#define BARK_WORLD_EVALUATION_LTL_LABELS_EGO_LEFTMOST_LANE_LABEL_FUNCTION_HPP_

#include <string>
#include <vector>

#include "bark/world/evaluation/ltl/label_functions/base_label_function.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/objects/object.hpp"

namespace bark {
namespace world {
namespace evaluation {

using bark::commons::transformation::FrenetPosition;
using bark::world::objects::AgentPtr;

class EgoLeftmostLaneLabelFunction : public BaseLabelFunction {
 public:
  EgoLeftmostLaneLabelFunction(const std::string& label_str,
                            const double distance_thres);
  LabelMap Evaluate(const world::ObservedWorld& observed_world) const override;
  double GetDistanceThres() const { return distance_thres_; }

 private:
  double distance_thres_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_LTL_LABELS_EGO_LEFTMOST_LANE_LABEL_FUNCTION_HPP_
