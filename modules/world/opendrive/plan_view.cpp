// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <math.h>
#include <limits>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include "modules/world/opendrive/plan_view.hpp"
#include "modules/world/opendrive/lane.hpp"
#include "modules/world/opendrive/odrSpiral.hpp"

namespace modules {
namespace world {
namespace opendrive {

namespace bg = boost::geometry;

bool PlanView::add_line(geometry::Point2d start_point,
                        float heading,
                        float length) {
  //! straight line
  reference_line_.add_point(start_point);
  geometry::Point2d end_point(
    bg::get<0>(start_point) + length * cos(heading),
    bg::get<1>(start_point) + length * sin(heading));
  reference_line_.add_point(end_point);
  //! calculate overall length
  length_ = bg::length(reference_line_.obj_);
  return true;
}

bool PlanView::add_spiral(
  geometry::Point2d start_point,
  float heading,
  float length,
  float curvature_start,
  float curvature_end,
  float s_inc) {
  double x = bg::get<0>(start_point);
  double y = bg::get<1>(start_point);
  double t = heading, cDot = (curvature_end - curvature_start) / length;
  double x_old = bg::get<0>(start_point), y_old = bg::get<1>(start_point);

  double s = 0.0;
  for (; s <= length;) {
    odrSpiral(s, x_old, y_old, cDot, curvature_start, heading, &x, &y, &t);
    reference_line_.add_point(geometry::Point2d(x, y));
    if ((length - s < s_inc) && (length - s > 0.))
      s_inc = length - s;
    s += s_inc;
  }
  length_ = bg::length(reference_line_.obj_);
  return true;
}

void PlanView::calc_arc_position(
  const float s,
  float initial_heading,
  float curvature,
  float &dx,
  float &dy) {
  initial_heading = fmod(initial_heading, 2 * M_PI);
  float hdg = initial_heading - M_PI / 2;
  float a = 2 / curvature * sin(s * curvature / 2);
  float alpha = (M_PI - s * curvature) / 2 - hdg;
  dx = -1 * a * cos(alpha);
  dy = a * sin(alpha);
  // tangent = initial_heading + s * initial_curvature;
}

bool PlanView::add_arc(
  geometry::Point2d start_point,
  float heading,
  float length,
  float curvature,
  float s_inc) {
  // add_spiral(start_point, heading, length, curvature, curvature, s_inc);

  float dx, dy;
  double x_old = bg::get<0>(start_point), y_old = bg::get<1>(start_point);
  double s = 0.0;
  for (; s <= length;) {
    calc_arc_position(s, heading, curvature, dx, dy);
    reference_line_.add_point(geometry::Point2d(x_old + dx, y_old + dy));
    if (length - s < s_inc && length - s > 0.)
      s_inc = length - s;
    s += s_inc;
  }
  return true;
}

}  // namespace opendrive
}  // namespace world
}  // namespace modules
