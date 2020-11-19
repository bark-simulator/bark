// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/opendrive/plan_view.hpp"
#include <math.h>
#include <limits>
#include "bark/world/opendrive/lane.hpp"
#include "bark/world/opendrive/odrSpiral.hpp"

namespace bark {
namespace world {
namespace opendrive {

/**
 * @brief adds a line segment to the plan view. if only two points shall be
 * generated, set s_inc to length.
 *
 * @param start_point starting point
 * @param heading heading
 * @param length length of line
 * @param s_inc increment at which points are sampled
 */
bool PlanView::AddLine(Point2d start_point, double heading, double length,
                       double s_inc) {
  namespace bg = boost::geometry;
  using bark::geometry::Line;
  using bark::geometry::Point2d;

  int num_points = length / s_inc;
  //! straight line
  for (size_t i = 0; i <= num_points; ++i) {
    Point2d p(bg::get<0>(start_point) + i * s_inc * cos(heading),
              bg::get<1>(start_point) + i * s_inc * sin(heading));
    reference_line_.AddPoint(p);
  }
  if (length > num_points * s_inc) {
    Point2d end_p(bg::get<0>(start_point) + length * cos(heading),
                  bg::get<1>(start_point) + length * sin(heading));
    reference_line_.AddPoint(end_p);
  }

  //! calculate overall length
  length_ = bg::length(reference_line_.obj_);

  if (boost::geometry::intersects(reference_line_.obj_)) {
    LOG(ERROR) << "planview has self-intersection after adding line";
  }

  return true;
}

bool PlanView::AddSpiral(Point2d start_point, double heading, double length,
                         double curvature_start, double curvature_end,
                         double s_inc) {
  namespace bg = boost::geometry;
  double x = bg::get<0>(start_point);
  double y = bg::get<1>(start_point);
  double t = heading, cDot = (curvature_end - curvature_start) / length;
  double x_old = bg::get<0>(start_point), y_old = bg::get<1>(start_point);

  double s = 0.0;
  for (; s <= length;) {
    odrSpiral(s, x_old, y_old, cDot, curvature_start, heading, &x, &y, &t);
    reference_line_.AddPoint(Point2d(x, y));
    if ((length - s < s_inc) && (length - s > 0.)) s_inc = length - s;
    s += s_inc;
  }

  length_ = bg::length(reference_line_.obj_);

  if (boost::geometry::intersects(reference_line_.obj_)) {
    LOG(ERROR) << "planview has self-intersection after adding spiral";
  }

  return true;
}

void PlanView::CalcArcPosition(const double s, double initial_heading,
                               double curvature, double& dx, double& dy) {
  initial_heading = fmod(initial_heading, 2 * M_PI);
  double hdg = initial_heading - M_PI / 2;
  double a = 2 / curvature * sin(s * curvature / 2);
  double alpha = (M_PI - s * curvature) / 2 - hdg;
  dx = -1 * a * cos(alpha);
  dy = a * sin(alpha);
  // tangent = initial_heading + s * initial_curvature;
}

bool PlanView::AddArc(Point2d start_point, double heading, double length,
                      double curvature, double s_inc) {
  // AddSpiral(start_point, heading, length, curvature, curvature, s_inc);
  namespace bg = boost::geometry;
  double dx, dy;
  double x_old = bg::get<0>(start_point), y_old = bg::get<1>(start_point);
  double s = 0.0;
  for (; s <= length;) {
    CalcArcPosition(s, heading, curvature, dx, dy);
    reference_line_.AddPoint(Point2d(x_old + dx, y_old + dy));
    if (length - s < s_inc && length - s > 0.) s_inc = length - s;
    s += s_inc;
  }

  if (boost::geometry::intersects(reference_line_.obj_)) {
    LOG(ERROR) << "planview has self-intersection after adding arc";
  }

  return true;
}

bool PlanView::ApplyOffsetTransform(double x, double y, double hdg) {
  Line rotated_line = Rotate(reference_line_, hdg);
  Line transformed_line = Translate(rotated_line, x, y);
  reference_line_ = transformed_line;

  return true;
}

}  // namespace opendrive
}  // namespace world
}  // namespace bark
