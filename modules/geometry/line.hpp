// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_GEOMETRY_LINE_HPP_
#define MODULES_GEOMETRY_LINE_HPP_

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

#include <boost/tuple/tuple.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "modules/geometry/commons.hpp"
#include "modules/geometry/angle.hpp"

namespace modules {
namespace geometry {

//! templated line class with a boost polygon as a member function
template <typename T>
class Line_t : public Shape<bg::model::linestring<T>, T> {
 public:
  Line_t() : Shape<bg::model::linestring<T>, T>(Pose(0, 0, 0), std::vector<T>(), 0) {}

  virtual Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> toArray() const;

  virtual std::shared_ptr<Shape<bg::model::linestring<T>, T>> Clone() const;

  //! @todo improvement: do not recompute full s but only add one point (but we would need to store the line length for that!)
  bool add_point(const T &p) {
    return Shape<bg::model::linestring<T>, T>::add_point(p) && recompute_s();
  }

  auto length() const { return bg::length(Shape<bg::model::linestring<T>, T>::obj_);}

  unsigned int size() const { return Shape<bg::model::linestring<T>, T>::obj_.size(); }

  void append_linestring(const Line_t &ls) {
    // TODO(@fortiss): wrong
    bg::append(Shape<bg::model::linestring<T>, T>::obj_, ls.obj_);
    recompute_s();
  }

  std::vector<T> get_points_in_s_interval(float begin, float end) const {
    std::vector<T> points;
    uint begin_idx = std::upper_bound(s_.begin(), s_.end(), begin) - s_.begin();
    uint end_idx = std::lower_bound(s_.begin(), s_.end(), end) - s_.begin();
    std::copy(Shape<bg::model::linestring<T>, T>::obj_.begin() + begin_idx,
              Shape<bg::model::linestring<T>, T>::obj_.begin() + end_idx,
              std::back_inserter(points));
    return points;
  }

  virtual bool Valid() {
    return Shape<bg::model::linestring<T>, T>::Valid() &&
    s_.size() == size();
  }

  void reverse() {
    boost::geometry::reverse(Shape<bg::model::linestring<T>, T>::obj_);
  }

  void ConcatenateLinestring(const Line_t &other_line) {
    // Get first and last points
    auto first_point_this = *begin();
    auto last_point_this = *(end() - 1);
    auto first_point_other = *other_line.begin();
    auto last_point_other = *(other_line.end() - 1);

    float distance_first_first = distance(first_point_this, first_point_other);
    float distance_first_last = distance(first_point_this, last_point_other);
    float distance_last_first = distance(last_point_this, first_point_other);
    float distance_last_last = distance(last_point_this, last_point_other);

    if (distance_first_first <= std::min({distance_first_last, distance_last_first, distance_last_last})) {
      // Reverse this
      reverse();
      append_linestring(other_line);
    } else if (distance_first_last <= std::min({distance_first_first, distance_last_first, distance_last_last})) {
      // Reverse both
      reverse();
      Line_t new_line = other_line;
      new_line.reverse();
      append_linestring(new_line);
    } else if (distance_last_first <= std::min({distance_first_first, distance_first_last, distance_last_last})) {
      // No reversing
      append_linestring(other_line);
    } else {
      // Reverse other
      Line_t new_line = other_line;
      new_line.reverse();
      append_linestring(new_line);
    }
  }

  typedef typename std::vector<T>::iterator point_iterator;
  typedef typename std::vector<T>::const_iterator const_point_iterator;
  point_iterator begin() { return Shape<bg::model::linestring<T>, T>::obj_.begin(); }
  const_point_iterator  begin() const { return Shape<bg::model::linestring<T>, T>::obj_.begin(); }
  point_iterator end() { return Shape<bg::model::linestring<T>, T>::obj_.end(); }
  const_point_iterator end() const { return Shape<bg::model::linestring<T>, T>::obj_.end(); }

  typedef typename std::vector<T>::reverse_iterator reverse_point_iterator;
  typedef typename std::vector<T>::const_reverse_iterator const_reverse_point_iterator;
  reverse_point_iterator rbegin() { return Shape<bg::model::linestring<T>, T>::obj_.rbegin(); }
  const_reverse_point_iterator rbegin() const { return Shape<bg::model::linestring<T>, T>::obj_.rbegin(); }
  reverse_point_iterator rend() { return Shape<bg::model::linestring<T>, T>::obj_.rend(); }
  const_reverse_point_iterator rend() const { return Shape<bg::model::linestring<T>, T>::obj_.rend(); }

  std::vector<float> s_;  //! local coordinates 0..[total distance] along the lines

  //! @todo free function, s_ private?
  bool recompute_s() {
    s_.clear();
    // edge case no points
    if (Shape<bg::model::linestring<T>, T>::obj_.empty()) {
      return true;

    } else if (Shape<bg::model::linestring<T>, T>::obj_.size() == 1) {  // edge case one point
      s_.push_back(0.0);
      return true;
    } else {  // nominal case
      // compute distance from last point to next point and store these on vector s_ (avoid additional tmp vector)
      T last_pt = Shape<bg::model::linestring<T>, T>::obj_.front();
      s_.reserve(Shape<bg::model::linestring<T>, T>::obj_.size());
      double distance_until_now = 0.0;
      for (const T &next_pt : Shape<bg::model::linestring<T>, T>::obj_) {
        distance_until_now += bg::distance(next_pt, last_pt);
        s_.push_back(distance_until_now);
        last_pt = next_pt;
      }
      // // divide by total distance --> s \in 0..1
      // const double &last_s = s_.back();
      // for (int i = 0; i < s_.size(); ++i) {  // normalized 0..1
      //   s_[i] = s_[i] / last_s;
      // }
      return true;
    }
  }
};

//! for better usage simple float defines
using LinePoint = Point2d;
using Line = Line_t<LinePoint>;

template <>
inline Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Line::toArray() const {
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mat(obj_.size(), 2);
  for (uint32_t i = 0; i < obj_.size(); i++) {
    mat.row(i) << bg::get<0>(obj_[i]), bg::get<1>(obj_[i]);
  }
  return mat;
}

template <typename T>
inline std::shared_ptr<Shape<bg::model::linestring<T>, T>> Line_t<T>::Clone() const {
  std::shared_ptr<Line_t<T>> new_line = std::make_shared<Line_t<T>>(*this);
  return new_line;
}

inline float distance(const Line &line, const Point2d &p) {
  return bg::distance(line.obj_, p);
}

inline float distance(const Line &line, const Line &line2) {
  return bg::distance(line.obj_, line2.obj_);
}

template <typename T>
inline T length(const Line &line) {
  return bg::length<T>(line.obj_);
}

inline int get_segment_end_idx(Line l, float s) {
  std::vector<float>::iterator up = std::upper_bound(l.s_.begin(), l.s_.end(), s);
  if(up != l.s_.end()) {
    int retval = up - l.s_.begin();
    return retval;
  } else {
    return  l.s_.size()-1; // last point if s is larger then line length
  }
  
}

inline bool check_s_for_segment_intersection(Line l, float s) {
  int start_it = get_segment_end_idx(l, s);
  std::vector<float>::iterator low = std::lower_bound(l.s_.begin(), l.s_.end(), s);
  int start_it_low = low - l.s_.begin();
  return start_it != start_it_low;
}

inline Point2d get_point_at_s(Line l, float s) {
  const size_t &length = l.obj_.size();
  if (length <= 1) {  // this is an error Line consist of 0 or 1 element
    return Point2d(0, 0);
  } else if (s <= 0.0) {  // edge case begin
    return l.obj_.at(0);
  } else if (s >= l.s_.back()) {  // edge case end
    return l.obj_.at(length - 1);
  } else {  // nominal case
    int segment_end_idx = get_segment_end_idx(l, s);
    int segment_begin_idx = segment_end_idx - 1;

    float s_on_segment = (s - l.s_.at(segment_begin_idx)) / (l.s_.at(segment_end_idx) - l.s_.at(segment_begin_idx));
    float interp_pt_x = bg::get<0>(l.obj_.at(segment_begin_idx))
                        + s_on_segment * (bg::get<0>(l.obj_.at(segment_end_idx))
                        - bg::get<0>(l.obj_.at(segment_begin_idx)));
    float interp_pt_y = bg::get<1>(l.obj_.at(segment_begin_idx))
                        + s_on_segment * (bg::get<1>(l.obj_.at(segment_end_idx))
                        - bg::get<1>(l.obj_.at(segment_begin_idx)));
    return Point2d(interp_pt_x, interp_pt_y);
  }
}

inline float get_tangent_angle_at_s(Line l, float s) {
  if (s >= l.s_.back()) {
    Point2d p1 = l.obj_.at(l.obj_.size()-2);
    Point2d p2 = l.obj_.at(l.obj_.size()-1);
    float angle = atan2(bg::get<1>(p2) - bg::get<1>(p1), bg::get<0>(p2) - bg::get<0>(p1));
    return angle;
  } else if (s <= 0.0) {
    Point2d p1 = l.obj_.at(0);
    Point2d p2 = l.obj_.at(1);
    return atan2(bg::get<1>(p2) - bg::get<1>(p1), bg::get<0>(p2) - bg::get<0>(p1));
  } else {  // not start or end
    int end_segment_it = get_segment_end_idx(l, s);
    if (check_s_for_segment_intersection(l, s)) {  // check if s is at intersection, if true then calculate the intermediate angle
      Point2d p1 = l.obj_.at(end_segment_it-2);
      Point2d p2 = l.obj_.at(end_segment_it-1);
      Point2d p3 = l.obj_.at(end_segment_it);
      float sin_mean = 0.5 * (sin(atan2(bg::get<1>(p2) - bg::get<1>(p1), bg::get<0>(p2) - bg::get<0>(p1)))
                        + sin(atan2(bg::get<1>(p3) - bg::get<1>(p2), bg::get<0>(p3) - bg::get<0>(p2))));
      float cos_mean = 0.5 * (cos(atan2(bg::get<1>(p2) - bg::get<1>(p1), bg::get<0>(p2) - bg::get<0>(p1)))
                        + cos(atan2(bg::get<1>(p3) - bg::get<1>(p2), bg::get<0>(p3) - bg::get<0>(p2))));
      return atan2(sin_mean, cos_mean);
    } else {  // every s not start, end or intersection
      Point2d p1 = l.obj_.at(end_segment_it-1);
      Point2d p2 = l.obj_.at(end_segment_it);
      return atan2(bg::get<1>(p2) - bg::get<1>(p1), bg::get<0>(p2) - bg::get<0>(p1));
    }
  }
}

inline Point2d get_normal_at_s(Line l, float s) {
  // std::cout << "get_normal_at_s() ";
  // for (auto& ll : l) {
  //   std::cout << boost::geometry::get<0>(ll) << ", " << boost::geometry::get<1>(ll) << " " << s << std::endl;
  // }
  float tangent = get_tangent_angle_at_s(l, s);
  Point2d t(cos(tangent+asin(1)), sin(tangent+asin(1)));  // rotate unit vector anti-clockwise with angle = tangent by 1/2 pi
  // std::cout << "tangent" << tangent << std::endl;
  return t;
}


inline Line get_line_from_s_interval(Line line, float begin, float end) {
  Line new_line;
  new_line.add_point(get_point_at_s(line, begin));
  std::vector<Point2d> points = line.get_points_in_s_interval(begin, end);
  for (auto const &point : points) {
    new_line.add_point(point);
  }
  new_line.add_point(get_point_at_s(line, end));
  return new_line;
}


inline std::tuple<Point2d, double, uint> get_nearest_point_and_s(Line l, const Point2d &p) {  // get_nearest_point
  // edge cases: empty or one-point line
  if (l.obj_.empty()) {
    return std::make_tuple(Point2d(0, 0), 0.0, 0);
  } else if (l.obj_.size() == 1) {
    return std::make_tuple(l.obj_.at(0), 0.0, 0);
  }

  // nominal case:
  // check distance to each line segment and find closest segment
  double min_dist = boost::numeric::bounds<float>::highest();
  int min_segment_idx = 0;
  for (uint line_idx = 0; line_idx < l.obj_.size() - 1; ++line_idx) {
    bg::model::linestring<Point2d> current_segment;
    bg::append(current_segment, l.obj_.at(line_idx));
    bg::append(current_segment, l.obj_.at(line_idx + 1));
    double d = bg::comparable_distance(current_segment, p);
    if (d < min_dist) {
      min_dist = d;
      min_segment_idx = line_idx;
    }
  }

  // compute closest point on segment and interpolation factor lambda
  // point: p, line a--b, closest point on line: s, tangent factor: lambda, normal factor: delta
  // equation by matlab:
  // syms a1 a2 b1 b2 s1 s2 p1 p2 lambda delta
  // gl3x = (a1 + lambda*(b1 - a1) == s1);
  // gl3y = (a2 + lambda*(b2 - a2) == s2);
  // gl4x = (s1 + delta*(-b2 + a2) == p1);
  // gl4y = (s2 + delta*(b1 - a1) == p2);
  // sol = solve([gl3x, gl3y, gl4x, gl4y], [s1, s2, lambda, delta], 'ReturnConditions', true);
  const double a1 = bg::get<0>(l.obj_.at(min_segment_idx));
  const double a2 = bg::get<1>(l.obj_.at(min_segment_idx));
  const double b1 = bg::get<0>(l.obj_.at(min_segment_idx + 1));
  const double b2 = bg::get<1>(l.obj_.at(min_segment_idx + 1));
  const double p1 = bg::get<0>(p);
  const double p2 = bg::get<1>(p);

  const double lambda = -(a1 * b1 + a2 * b2 + a1 * p1 + a2 * p2 - b1 * p1 - b2 * p2 - a1 * a1 - a2 * a2)
                        / (a1 * a1 - 2 * a1 * b1 + a2 * a2 - 2 * a2 * b2 + b1 * b1 + b2 * b2);

  // calculate interpolated s value
  double s;
  // double dist;  // unused
  Point2d retval;

  if (lambda < 0) {  // extrapolation front
    s = l.s_.at(min_segment_idx);
    retval = Point2d(a1, a2);
    // debug
    // dist = sqrt(pow(p1 - a1, 2) + pow(p2 - a2, 2));
  } else if (lambda > 1) {  // extrapolation end
    s = l.s_.at(min_segment_idx + 1);
    retval = Point2d(b1, b2);
    // debug
    // dist = sqrt(pow(p1 - b1, 2) + pow(p2 - b2, 2));
  } else {  // real interpolation
    s = (1 - lambda) * l.s_.at(min_segment_idx) + lambda * l.s_.at(min_segment_idx + 1);

    const double s1 = (p1 * a1 * a1 - a1 * a2 * b2 + p2 * a1 * a2
                    - 2 * p1 * a1 * b1 + a1 * b2 * b2 - p2 * a1 * b2
                    + a2 * a2 * b1 - a2 * b1 * b2 - p2 * a2 * b1 + p1 * b1 * b1 + p2 * b1 * b2)
                    / (a1 * a1 - 2 * a1 * b1 + a2 * a2 - 2 * a2 * b2 + b1 * b1 + b2 * b2);
    const double s2 = (a1 * a1 * b2 - a1 * a2 * b1 + p1 * a1 * a2
                    - a1 * b1 * b2 - p1 * a1 * b2 + p2 * a2 * a2
                    + a2 * b1 * b1 - p1 * a2 * b1 - 2 * p2 * a2 * b2 + p1 * b1 * b2 + p2 * b2 * b2)
                    / (a1 * a1 - 2 * a1 * b1 + a2 * a2 - 2 * a2 * b2 + b1 * b1 + b2 * b2);

    // debug
    // dist = sqrt(pow(p1 - s1, 2) + pow(p2 - s2, 2));
    retval = Point2d(s1, s2);
  }

  // debug
  // const double dist_boost = bg::distance(l.obj_, p);

  // return
  return std::make_tuple(retval, s, min_segment_idx);
}
inline Point2d get_nearest_point(Line l, const Point2d &p) {
  return std::get<0>(get_nearest_point_and_s(l, p));
}
inline float get_nearest_s(Line l, const Point2d &p) {
  return std::get<1>(get_nearest_point_and_s(l, p));
}
inline uint FindNearestIdx(Line l, const Point2d &p) {
  return std::get<2>(get_nearest_point_and_s(l, p));
}
//! Point - Line collision checker using boost::intersection
inline bool Collide(const Line &l, const LinePoint &p) {
  std::vector<LinePoint> shape_intersect;
  bg::intersection(l.obj_, p, shape_intersect);
  return !shape_intersect.empty();
}

//! Line - Point collision checker
inline bool Collide(const LinePoint &p, const Line &l) {
  return Collide(l, p);
}

//! Line - Line collision checker using boost::intersection
inline bool Collide(const Line &l1, const Line &l2) {
  std::vector<bg::model::linestring<LinePoint>> shape_intersect;
  bg::intersection(l1.obj_, l2.obj_, shape_intersect);
  return !shape_intersect.empty();
}

// An oriented point can have a linestring (the nearest point on it) on the left or right side, left side < 0, right side > 0
inline double signed_distance(const Line &line, const Point2d &p, const float& orientation) {
  auto closest_point = get_nearest_point(line, p);
  auto direction_vector = closest_point - p;

  double diff = signed_angle_diff(orientation , atan2(bg::get<1>(direction_vector), bg::get<0>(direction_vector)));
  double sign = (diff > 0) ? 1 : ((diff < 0) ? -1 : 0);

  return bg::distance(line.obj_, p)*sign;
}

inline Line ComputeCenterLine(const Line& outer_line_,
                              const Line& inner_line_) {
  Line center_line_;
  Line line_more_points = outer_line_;
  Line line_less_points = inner_line_;
  if ( inner_line_.obj_.size() > outer_line_.obj_.size() ) {
    line_more_points = inner_line_;
    line_less_points = outer_line_;
  }
  for ( Point2d& point_loop : line_more_points.obj_ ) {
    Point2d nearest_point_other = geometry::get_nearest_point(line_less_points,
                                                              point_loop);
    geometry::Point2d middle_point = (point_loop + nearest_point_other) / 2;
    center_line_.add_point(middle_point);
  }
  return center_line_;
}


template <typename T>
std::pair<T, T> merge_bounding_boxes(std::pair<T, T> bb1, std::pair<T, T> bb2) {
  Line_t<T> line; // just use a line and add all points
  line.add_point(bb1.first);
  line.add_point(bb1.second);
  line.add_point(bb2.first);
  line.add_point(bb2.second);
  return line.bounding_box();
}

}  // namespace geometry
}  // namespace modules
#endif  // MODULES_GEOMETRY_LINE_HPP_
