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
  bool AddLine(Point2d start_point, double heading, double length,
               double s_inc);

  bool AddSpiral(Point2d start_point, double heading, double length,
                 double curvStart, double curvEnd, double s_inc = 2.0);

  bool AddArc(Point2d start_point, double heading, double length, double curvature,
              double s_inc = 2.0);

  void CalcArcPosition(const double s, double initial_heading, double curvature,
                       double& dx, double& dy);

  //! getter functions
  Line GetReferenceLine() const { return reference_line_; }

  bool ApplyOffsetTransform(double x, double y, double hdg);

  double GetLength() const { return length_; }
  double GetDistance(const Point2d& p) const {
    return boost::geometry::distance(reference_line_.obj_, p);
  }

 private:
  Line reference_line_;  // sequential build up
  double length_;
};

using PlanViewPtr = std::shared_ptr<PlanView>;

}  // namespace opendrive
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_OPENDRIVE_PLAN_VIEW_HPP_
