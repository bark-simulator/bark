// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_OPENDRIVE_PLAN_VIEW_HPP_
#define BARK_WORLD_OPENDRIVE_PLAN_VIEW_HPP_

#include <Eigen/Core>
#include "bark/geometry/geometry.hpp"
#include "bark/world/opendrive/lane.hpp"

namespace bark {
namespace world {
namespace opendrive {

using bark::geometry::Line;
using bark::geometry::Point2d;

class PlanView {
 public:
  PlanView() : length_(0.0) {}
  ~PlanView() {}

  //! setter functions
  bool AddLine(Point2d start_point, float heading, float length);

  bool AddSpiral(Point2d start_point, float heading, float length,
                 float curvStart, float curvEnd, float s_inc = 2.0f);

  bool AddArc(Point2d start_point, float heading, float length, float curvature,
              float s_inc = 2.0f);

  void CalcArcPosition(const float s, float initial_heading, float curvature,
                       float& dx, float& dy);

  //! getter functions
  Line GetReferenceLine() const { return reference_line_; }

  bool ApplyOffsetTransform(float x, float y, float hdg);

  float GetLength() const { return length_; }
  float GetDistance(const Point2d& p) const {
    return boost::geometry::distance(reference_line_.obj_, p);
  }

 private:
  Line reference_line_;  // sequential build up
  float length_;
};

using PlanViewPtr = std::shared_ptr<PlanView>;

}  // namespace opendrive
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_OPENDRIVE_PLAN_VIEW_HPP_
