// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_OPENDRIVE_PLAN_VIEW_HPP_
#define MODULES_WORLD_OPENDRIVE_PLAN_VIEW_HPP_

#include <Eigen/Core>
#include "modules/geometry/geometry.hpp"
#include "modules/world/opendrive/lane.hpp"

namespace modules {
namespace world {
namespace opendrive {

using modules::geometry::Point2d;
using modules::geometry::Line;


class PlanView {
 public:
  PlanView() : length_(0.0) {}
  ~PlanView() {}

  //! setter functions
  bool AddLine(Point2d start_point, float heading, float length);

  bool AddSpiral(
    Point2d start_point,
    float heading,
    float length,
    float curvStart,
    float curvEnd,
    float s_inc = 2.0f);

  bool AddArc(
    Point2d start_point,
    float heading,
    float length,
    float curvature,
    float s_inc = 2.0f);

  void CalcArcPosition(
    const float s, float initial_heading, float curvature, float &dx, float &dy);

  //! getter functions
  Line GetReferenceLine() const { return reference_line_; }

  bool ApplyOffsetTransform(float x, float y, float hdg);

  float GetLength() const { return length_; }
  float GetDistance( const Point2d &p) const {
    return boost::geometry::distance(reference_line_.obj_, p);
  }

 private:
  Line reference_line_;  // sequential build up
  float length_;
};

using PlanViewPtr = std::shared_ptr<PlanView>;

}  // namespace opendrive
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OPENDRIVE_PLAN_VIEW_HPP_
