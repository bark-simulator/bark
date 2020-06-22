// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_LTL_LABELS_EGO_BEYOND_POINT_LABEL_FUNCTION_HPP_
#define MODULES_WORLD_EVALUATION_LTL_LABELS_EGO_BEYOND_POINT_LABEL_FUNCTION_HPP_

#include <string>
#include <vector>

#include "bark/world/evaluation/ltl/labels/base_label_function.hpp"
#include "bark/world/objects/object.hpp"
#include "bark/world/opendrive/commons.hpp"

namespace modules {
namespace world {
namespace evaluation {
using geometry::Point2d;
using opendrive::XodrLaneId;

class EgoBeyondPointLabelFunction : public BaseLabelFunction {
 public:
  EgoBeyondPointLabelFunction(const std::string& label_str,
                              const Point2d& beyond_point);
  LabelMap Evaluate(const world::ObservedWorld& observed_world) const override;
  const Point2d& GetBeyondPoint() const;

 private:
  Point2d beyond_point_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_LTL_LABELS_EGO_BEYOND_POINT_LABEL_FUNCTION_HPP_
