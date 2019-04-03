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

bool PlanView::add_line(geometry::Point2d start_point, float heading, float length) {
  //! straight line
  reference_line_.add_point(start_point);
  geometry::Point2d end_point(bg::get<0>(start_point) + length * cos(heading), bg::get<1>(start_point) + length * sin(heading));
  reference_line_.add_point(end_point);

  //! calculate overall length
  length_ = bg::length(reference_line_.obj_);
  return true;
}

bool PlanView::add_spiral(geometry::Point2d start_point, float heading, float length, float curvature_start, float curvature_end, float s_inc) {
  double x = bg::get<0>(start_point), y = bg::get<1>(start_point), t = heading, cDot = (curvature_end - curvature_start) / length;
  double x_old = bg::get<0>(start_point), y_old = bg::get<1>(start_point);

  double s = 0.0;
  for (; s < length; s += s_inc) {
    odrSpiral(s, x_old, y_old, cDot, curvature_start, heading, &x, &y, &t);
    reference_line_.add_point(geometry::Point2d(x, y));
  }

  // fill last point if increment does not match
  double delta_s = fabs(length - s);
  if (delta_s > 0.0) {
    odrSpiral(length, x_old, y_old, cDot, curvature_start, heading, &x, &y, &t);
    reference_line_.add_point(geometry::Point2d(x, y));
  }
  
  length_ = bg::length(reference_line_.obj_);
  return true;
}

void PlanView::calc_arc_position(const float s, float initial_heading, float curvature, float &dx, float &dy) {
  initial_heading = fmod(initial_heading, 2 * M_PI);
  float hdg = initial_heading - M_PI / 2;

  float a = 2 / curvature * sin(s * curvature / 2);
  float alpha = (M_PI - s * curvature) / 2 - hdg;

  dx = -1 * a * cos(alpha);
  dy = a * sin(alpha);

  // tangent = initial_heading + s * initial_curvature;
}

bool PlanView::add_arc(geometry::Point2d start_point, float heading, float length, float curvature, float s_inc) {
  // add_spiral(start_point, heading, length, curvature, curvature, s_inc);

  float dx, dy;
  double x_old = bg::get<0>(start_point), y_old = bg::get<1>(start_point);
  double s = 0.0;
  for (; s < length; s += s_inc) {
    calc_arc_position(s, heading, curvature, dx, dy);
    reference_line_.add_point(geometry::Point2d(x_old + dx, y_old + dy));
  }
  
  // fill last point if increment does not match
  double delta_s = fabs(length - s);
  if (delta_s >= 0.0){
    calc_arc_position(length, heading, curvature, dx, dy);
    reference_line_.add_point(geometry::Point2d(x_old + dx, y_old + dy));
  }
  
  return true;
}

geometry::Line PlanView::create_line(int id, LaneWidth lane_width, float s_inc) {
  float s_start = lane_width.s_start;
  float s_end = lane_width.s_end;
  LaneOffset off = lane_width.off;

  float s = s_start;
  float scale = 0.0f;
  geometry::Line tmp_line;
  geometry::Point2d normal(0.0f, 0.0f);
  int sign = id > 0 ? -1 : 1;

  // TODO(fortiss): check if sampling does work with relative s, probably not
  if (off.b != 0.0f || off.c != 0.0f || off.d != 0.0f || (lane_width.s_end - lane_width.s_start) != 1.0) {
    for (; s < s_end; s += s_inc) {
      geometry::Point2d point = get_point_at_s(reference_line_, s);
      normal = get_normal_at_s(reference_line_, s);
      scale = -sign * polynom(s, off.a, off.b, off.c, off.d);
      tmp_line.add_point(geometry::Point2d(bg::get<0>(point) + scale * bg::get<0>(normal),
                                  bg::get<1>(point) + scale * bg::get<1>(normal)));
    }

    // fill last point if increment does not match
    double delta_s = fabs(s_end-s);
    if(delta_s>0.0){
      geometry::Point2d point = get_point_at_s(reference_line_, s_end);
      normal = get_normal_at_s(reference_line_, s_end);
      scale = -sign * polynom(s_end, off.a, off.b, off.c, off.d);
      tmp_line.add_point(geometry::Point2d(bg::get<0>(point) + scale * bg::get<0>(normal),
                                  bg::get<1>(point) + scale * bg::get<1>(normal)));
    }
  } else {
      for (uint32_t i = 0; i < reference_line_.obj_.size() - 1; i++) {
        normal = get_normal_at_s(reference_line_, reference_line_.s_[i]);
        scale = -sign * polynom(s, off.a, off.b, off.c, off.d);
        tmp_line.add_point(geometry::Point2d(bg::get<0>(reference_line_.obj_[i]) + scale * bg::get<0>(normal),
                                  bg::get<1>(reference_line_.obj_[i]) + scale * bg::get<1>(normal)));
        s += geometry::distance(reference_line_.obj_[i + 1], reference_line_.obj_[i]);
      }
      // add last point
      normal = get_normal_at_s(reference_line_, reference_line_.s_[reference_line_.obj_.size() - 1]);
      int size = reference_line_.obj_.size() - 1;
      scale = -sign * polynom(length_, off.a, off.b, off.c, off.d);
      tmp_line.add_point(geometry::Point2d(bg::get<0>(reference_line_.obj_[size]) + scale * bg::get<0>(normal),
                                bg::get<1>(reference_line_.obj_[size]) + scale * bg::get<1>(normal)));
  }


  return tmp_line;
}

//! TODO: this function needs to resive a vector of Struct {s_start, s_end, off}
LanePtr PlanView::create_lane(LanePosition lane_position, LaneWidths lane_widths, float s_inc) {
  std::shared_ptr<Lane> ret_lane(new Lane(lane_position));
  if (lane_widths.size() > 1) {
    assert("Not supported");
  }
  for (LaneWidth lane_width : lane_widths) {
    geometry::Line tmp_line = create_line(lane_position, lane_width, s_inc);
    ret_lane->set_line(tmp_line);
  }

  // ret_lane->calculate_center_line();
  return ret_lane;
}

}  // namespace opendrive
}  // namespace world
}  // namespace modules
