// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_GEOMETRY_LINE_HPP_
#define BARK_GEOMETRY_LINE_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/tuple/tuple.hpp>

#include "bark/geometry/angle.hpp"
#include "bark/geometry/commons.hpp"

#include "src/spline.h"

namespace bark {
namespace geometry {

//! templated line class with a boost polygon as a member function
template <typename T>
class Line_t : public Shape<bg::model::linestring<T>, T> {
 public:
  Line_t()
      : Shape<bg::model::linestring<T>, T>(Pose(0, 0, 0), std::vector<T>(), 0) {
  }

  explicit Line_t(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& points);

  virtual Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ToArray() const;

  virtual std::shared_ptr<Shape<bg::model::linestring<T>, T>> Clone() const;

  //! TODO(@all): do not recompute full s but only add one point
  bool AddPoint(const T& p) {
    return Shape<bg::model::linestring<T>, T>::AddPoint(p) && RecomputeS();
  }

  bool AddPoints(const std::vector<T>& pts) {
    for (const auto& p : pts) {
      Shape<bg::model::linestring<T>, T>::AddPoint(p);
    }
    return RecomputeS();
  }

  auto Length() const {
    return bg::length(Shape<bg::model::linestring<T>, T>::obj_);
  }

  unsigned int size() const {
    return Shape<bg::model::linestring<T>, T>::obj_.size();
  }

  void AppendLinestring(const Line_t& ls) {
    bg::append(Shape<bg::model::linestring<T>, T>::obj_, ls.obj_);
    RecomputeS();
  }

  std::vector<T> GetPointsInSInterval(double begin, double end) const {
    std::vector<T> points;
    uint begin_idx = std::upper_bound(s_.begin(), s_.end(), begin) - s_.begin();
    uint end_idx = std::lower_bound(s_.begin(), s_.end(), end) - s_.begin();
    std::copy(Shape<bg::model::linestring<T>, T>::obj_.begin() + begin_idx,
              Shape<bg::model::linestring<T>, T>::obj_.begin() + end_idx,
              std::back_inserter(points));
    return points;
  }

  virtual bool Valid() const {
    return Shape<bg::model::linestring<T>, T>::Valid() && s_.size() == size();
  }

  void Reverse() {
    boost::geometry::reverse(Shape<bg::model::linestring<T>, T>::obj_);
  }

  typedef typename std::vector<T>::iterator point_iterator;
  typedef typename std::vector<T>::const_iterator const_point_iterator;
  point_iterator begin() {
    return Shape<bg::model::linestring<T>, T>::obj_.begin();
  }
  const_point_iterator begin() const {
    return Shape<bg::model::linestring<T>, T>::obj_.begin();
  }
  point_iterator end() {
    return Shape<bg::model::linestring<T>, T>::obj_.end();
  }
  const_point_iterator end() const {
    return Shape<bg::model::linestring<T>, T>::obj_.end();
  }

  typedef typename std::vector<T>::reverse_iterator reverse_point_iterator;
  typedef typename std::vector<T>::const_reverse_iterator
      const_reverse_point_iterator;  // NOLINT

  reverse_point_iterator rbegin() {
    return Shape<bg::model::linestring<T>, T>::obj_.rbegin();
  }
  const_reverse_point_iterator rbegin() const {
    return Shape<bg::model::linestring<T>, T>::obj_.rbegin();
  }
  reverse_point_iterator rend() {
    return Shape<bg::model::linestring<T>, T>::obj_.rend();
  }
  const_reverse_point_iterator rend() const {
    return Shape<bg::model::linestring<T>, T>::obj_.rend();
  }

  //! local coordinates 0..[total distance] along the lines
  std::vector<double> s_;

  //! @todo free function, s_ private?
  bool RecomputeS() {
    s_.clear();
    // edge case no points
    if (Shape<bg::model::linestring<T>, T>::obj_.empty()) {
      return true;
    } else if (Shape<bg::model::linestring<T>, T>::obj_.size() == 1) {
      // edge case one point
      s_.push_back(0.0);
      return true;
    } else {  // nominal case
      // compute distance from last point to next point and
      // store these on vector s_ (avoid additional tmp vector)
      T last_pt = Shape<bg::model::linestring<T>, T>::obj_.front();
      s_.reserve(Shape<bg::model::linestring<T>, T>::obj_.size());
      double distance_until_now = 0.0;
      for (const T& next_pt : Shape<bg::model::linestring<T>, T>::obj_) {
        distance_until_now += bg::distance(next_pt, last_pt);
        s_.push_back(distance_until_now);
        last_pt = next_pt;
      }
      return true;
    }
  }

  void RemoveDuplicates() {
    bg::unique(Shape<bg::model::linestring<T>, T>::obj_);
    RecomputeS();
  }
  bool operator==(const Line_t& rhs) const {
    return bg::equals(this->obj_, rhs.obj_);
  }
  bool operator!=(const Line_t& rhs) const { return !(rhs == *this); }
};

//! for better usage simple double defines
using LinePoint = Point2d;
using Line = Line_t<LinePoint>;

template <typename T>
inline Line_t<T>::Line_t(
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& points)
    : Shape<bg::model::linestring<T>, T>(points, 0) {
  RecomputeS();
}

template <>
inline Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Line::ToArray()
    const {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mat(obj_.size(), 2);
  for (uint32_t i = 0; i < obj_.size(); i++) {
    mat.row(i) << bg::get<0>(obj_[i]), bg::get<1>(obj_[i]);
  }
  return mat;
}

template <typename T>
inline std::shared_ptr<Shape<bg::model::linestring<T>, T>> Line_t<T>::Clone()
    const {
  std::shared_ptr<Line_t<T>> new_line = std::make_shared<Line_t<T>>(*this);
  return new_line;
}

template <typename T>
inline Line Reverse(const T& l) {
  T lr = l;
  lr.Reverse();
  return lr;
}

inline double Distance(const Line& line, const Point2d& p) {
  return bg::distance(line.obj_, p);
}

inline double Distance(const Line& line, const Line& line2) {
  return bg::distance(line.obj_, line2.obj_);
}

template <typename T>
inline T Length(const Line& line) {
  return bg::length<T>(line.obj_);
}

inline Line Rotate(const Line& line, double hdg) {
  using boost::geometry::strategy::transform::rotate_transformer;
  rotate_transformer<boost::geometry::radian, double, 2, 2> rotate(hdg);
  Line line_rotated;
  boost::geometry::transform(line.obj_, line_rotated.obj_, rotate);
  line_rotated.RecomputeS();
  return line_rotated;
}

inline Line Translate(const Line& line, double x, double y) {
  using boost::geometry::strategy::transform::translate_transformer;
  translate_transformer<double, 2, 2> translate(x, y);
  Line line_translated;
  boost::geometry::transform(line.obj_, line_translated.obj_, translate);
  return line_translated;
}

inline Line Simplify(const Line& line, double max_distance) {
  Line temp_line;
  boost::geometry::simplify(line.obj_, temp_line.obj_, max_distance);
  temp_line.RecomputeS();
  return temp_line;
}

inline int GetSegmentEndIdx(Line l, double s) {
  std::vector<double>::iterator up =
      std::upper_bound(l.s_.begin(), l.s_.end(), s);
  if (up != l.s_.end()) {
    int retval = up - l.s_.begin();
    return retval;
  } else {
    return l.s_.size() - 1;  // last point if s is larger then line length
  }
}

inline bool CheckSForSegmentIntersection(Line l, double s) {
  int start_it = GetSegmentEndIdx(l, s);
  std::vector<double>::iterator low =
      std::lower_bound(l.s_.begin(), l.s_.end(), s);
  int start_it_low = low - l.s_.begin();
  return start_it != start_it_low;
}

inline Point2d GetPointAtIdx(const Line& l, const uint idx) {
  if (idx > l.obj_.size() - 1) {
    LOG(WARNING) << "idx is outside line";
    return l.obj_.back();
  } else {
    return l.obj_.at(idx);
  }
}

inline Eigen::VectorXd Gradient(Eigen::VectorXd vec) {
  // calculating central difference
  Eigen::VectorXd g(vec.size());
  for (int i = 1; i < vec.size() - 1; i++) {
    g(i) = (vec(i + 1) - vec(i - 1)) / 2;
  }
  // TODO: find better solution for first and last point
  g(0) = g(1);
  g(vec.size() - 1) = g(vec.size() - 2);
  return g;
}

inline Eigen::VectorXd GetCurvature(const Line& l) {
  Eigen::MatrixXd larray = l.ToArray();
  Eigen::VectorXd dx = Gradient(larray.col(0));
  Eigen::VectorXd ddx = Gradient(dx);
  Eigen::VectorXd dy = Gradient(larray.col(1));
  Eigen::VectorXd ddy = Gradient(dy);

  // elementwise, as pow(vector, scalar) does not work
  Eigen::VectorXd curvature(larray.rows());
  for (int i = 0; i < curvature.size(); i++) {
    double n = dx(i) * ddy(i) - ddx(i) * dy(i);
    double r = pow(dx(i), 2) + pow(dy(i), 2);
    double d = pow(r, 1.5);
    if (d == 0) {
      curvature(i) = 0;
    } else {
      curvature(i) = n / d;
    }
  }
  return curvature;
}

inline Point2d GetPointAtS(Line l, double s) {
  const size_t& length = l.obj_.size();
  if (length <= 1) {  // this is an error Line consist of 0 or 1 element
    return Point2d(0, 0);
  } else if (s <= 0.0) {  // edge case begin
    return l.obj_.at(0);
  } else if (s >= l.s_.back()) {  // edge case end
    return l.obj_.at(length - 1);
  } else {  // nominal case
    int segment_end_idx = GetSegmentEndIdx(l, s);
    int segment_begin_idx = segment_end_idx - 1;

    double s_on_segment =
        (s - l.s_.at(segment_begin_idx)) /
        (l.s_.at(segment_end_idx) - l.s_.at(segment_begin_idx));
    double interp_pt_x =
        bg::get<0>(l.obj_.at(segment_begin_idx)) +
        s_on_segment * (bg::get<0>(l.obj_.at(segment_end_idx)) -
                        bg::get<0>(l.obj_.at(segment_begin_idx)));
    double interp_pt_y =
        bg::get<1>(l.obj_.at(segment_begin_idx)) +
        s_on_segment * (bg::get<1>(l.obj_.at(segment_end_idx)) -
                        bg::get<1>(l.obj_.at(segment_begin_idx)));
    return Point2d(interp_pt_x, interp_pt_y);
  }
}

inline double GetTangentAngleAtS(Line l, double s) {
  if (s >= l.s_.back()) {
    Point2d p1 = l.obj_.at(l.obj_.size() - 2);
    Point2d p2 = l.obj_.at(l.obj_.size() - 1);
    double angle =
        atan2(bg::get<1>(p2) - bg::get<1>(p1), bg::get<0>(p2) - bg::get<0>(p1));
    return angle;
  } else if (s <= 0.0) {
    Point2d p1 = l.obj_.at(0);
    Point2d p2 = l.obj_.at(1);
    return atan2(bg::get<1>(p2) - bg::get<1>(p1),
                 bg::get<0>(p2) - bg::get<0>(p1));
  } else {  // not start or end
    int end_segment_it = GetSegmentEndIdx(l, s);
    // check if s is at intersection, if true then calculate the angle
    if (CheckSForSegmentIntersection(l, s)) {
      Point2d p1 = l.obj_.at(end_segment_it - 2);
      Point2d p2 = l.obj_.at(end_segment_it - 1);
      Point2d p3 = l.obj_.at(end_segment_it);
      double sin_mean = 0.5 * (sin(atan2(bg::get<1>(p2) - bg::get<1>(p1),
                                         bg::get<0>(p2) - bg::get<0>(p1))) +
                               sin(atan2(bg::get<1>(p3) - bg::get<1>(p2),
                                         bg::get<0>(p3) - bg::get<0>(p2))));
      double cos_mean = 0.5 * (cos(atan2(bg::get<1>(p2) - bg::get<1>(p1),
                                         bg::get<0>(p2) - bg::get<0>(p1))) +
                               cos(atan2(bg::get<1>(p3) - bg::get<1>(p2),
                                         bg::get<0>(p3) - bg::get<0>(p2))));
      return atan2(sin_mean, cos_mean);
    } else {  // every s not start, end or intersection
      Point2d p1 = l.obj_.at(end_segment_it - 1);
      Point2d p2 = l.obj_.at(end_segment_it);
      return atan2(bg::get<1>(p2) - bg::get<1>(p1),
                   bg::get<0>(p2) - bg::get<0>(p1));
    }
  }
}

inline Point2d GetNormalAtS(const Line& l, double s) {
  double tangent = GetTangentAngleAtS(l, s);
  // rotate unit vector anti-clockwise with angle = tangent by 1/2 pi
  Point2d t(cos(tangent + asin(1)), sin(tangent + asin(1)));
  return t;
}

inline Line GetLineFromSInterval(const Line& line, double begin, double end) {
  Line new_line;
  new_line.AddPoint(GetPointAtS(line, begin));
  std::vector<Point2d> points = line.GetPointsInSInterval(begin, end);
  for (auto const& point : points) {
    new_line.AddPoint(point);
  }
  new_line.AddPoint(GetPointAtS(line, end));
  return new_line;
}

inline Line GetLineShiftedLaterally(const Line& line, double lateral_shift) {
  Line new_line;
  for (const auto& s : line.s_) {
    const Point2d normal = GetNormalAtS(line, s);
    const Point2d point_at_s = GetPointAtS(line, s);
    const Point2d shifted = point_at_s + (normal * lateral_shift);
    new_line.AddPoint(shifted);
  }
  return new_line;
}

inline std::tuple<Point2d, double, uint> GetNearestPointAndS(
    const Line& l, const Point2d& p) {  // GetNearestPoint
  // edge cases: empty or one-point line
  if (l.obj_.empty()) {
    return std::make_tuple(Point2d(0, 0), 0.0, 0);
  } else if (l.obj_.size() == 1) {
    return std::make_tuple(l.obj_.at(0), 0.0, 0);
  }

  // nominal case:
  // check distance to each line segment and find closest segment
  double min_dist = boost::numeric::bounds<double>::highest();
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

  const double a1 = bg::get<0>(l.obj_.at(min_segment_idx));
  const double a2 = bg::get<1>(l.obj_.at(min_segment_idx));
  const double b1 = bg::get<0>(l.obj_.at(min_segment_idx + 1));
  const double b2 = bg::get<1>(l.obj_.at(min_segment_idx + 1));
  const double p1 = bg::get<0>(p);
  const double p2 = bg::get<1>(p);

  const double lambda = -(a1 * b1 + a2 * b2 + a1 * p1 + a2 * p2 - b1 * p1 -
                          b2 * p2 - a1 * a1 - a2 * a2)  // NOLINT
                        / (a1 * a1 - 2 * a1 * b1 + a2 * a2 - 2 * a2 * b2 +
                           b1 * b1 + b2 * b2);  // NOLINT

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
    s = (1 - lambda) * l.s_.at(min_segment_idx) +
        lambda * l.s_.at(min_segment_idx + 1);  // NOLINT

    const double s1 =
        (p1 * a1 * a1 - a1 * a2 * b2 + p2 * a1 * a2 - 2 * p1 * a1 * b1 +
         a1 * b2 * b2 - p2 * a1 * b2 + a2 * a2 * b1 - a2 * b1 * b2 -
         p2 * a2 * b1 + p1 * b1 * b1 + p2 * b1 * b2)  // NOLINT
        / (a1 * a1 - 2 * a1 * b1 + a2 * a2 - 2 * a2 * b2 + b1 * b1 +
           b2 * b2);  // NOLINT
    const double s2 =
        (a1 * a1 * b2 - a1 * a2 * b1 + p1 * a1 * a2 - a1 * b1 * b2 -
         p1 * a1 * b2 + p2 * a2 * a2 + a2 * b1 * b1 - p1 * a2 * b1 -
         2 * p2 * a2 * b2 + p1 * b1 * b2 + p2 * b2 * b2)  // NOLINT
        / (a1 * a1 - 2 * a1 * b1 + a2 * a2 - 2 * a2 * b2 + b1 * b1 +
           b2 * b2);  // NOLINT

    // debug
    // dist = sqrt(pow(p1 - s1, 2) + pow(p2 - s2, 2));
    retval = Point2d(s1, s2);
  }

  // debug
  // const double dist_boost = bg::distance(l.obj_, p);

  // return
  return std::make_tuple(retval, s, min_segment_idx);
}
inline Point2d GetNearestPoint(const Line& l, const Point2d& p) {
  return std::get<0>(GetNearestPointAndS(l, p));
}
inline double GetNearestS(const Line& l, const Point2d& p) {
  return std::get<1>(GetNearestPointAndS(l, p));
}
inline uint FindNearestIdx(const Line& l, const Point2d& p) {
  return std::get<2>(GetNearestPointAndS(l, p));
}
//! Point - Line collision checker using boost::intersection
inline bool Collide(const Line& l, const LinePoint& p) {
  std::vector<LinePoint> shape_intersect;
  bg::intersection(l.obj_, p, shape_intersect);
  return !shape_intersect.empty();
}

//! Line - Point collision checker
inline bool Collide(const LinePoint& p, const Line& l) { return Collide(l, p); }

//! Line - Line collision checker using boost::intersection
inline bool Collide(const Line& l1, const Line& l2) {
  std::vector<bg::model::linestring<LinePoint>> shape_intersect;
  bg::intersection(l1.obj_, l2.obj_, shape_intersect);
  return !shape_intersect.empty();
}

// An oriented point can have a linestring (the nearest point on it)
// on the left or right side, left side < 0, right side > 0
inline double SignedDistance(const Line& line, const Point2d& p,
                             const double& orientation) {
  auto closest_point = GetNearestPoint(line, p);
  auto direction_vector = closest_point - p;

  double diff = SignedAngleDiff(
      orientation,
      atan2(bg::get<1>(direction_vector), bg::get<0>(direction_vector)));
  double sign = (diff > 0) ? 1 : ((diff < 0) ? -1 : 0);

  return bg::distance(line.obj_, p) * sign;
}

inline Line AppendLinesNoIntersect(const Line& ls1, const Line& ls2) {
  std::vector<Point2d> intersecting_points;
  bg::intersection(ls1.obj_, ls2.obj_, intersecting_points);
  Line lout;
  if (intersecting_points.size() == 1) {
    // get s value for both lines
    double s_i1 = GetNearestS(ls1, intersecting_points.at(0));
    double s_i2 = GetNearestS(ls2, intersecting_points.at(0));
    double rel_s_i1 = s_i1 / ls1.Length();
    double rel_s_i2 = s_i2 / ls2.Length();

    Line ls1_part, ls2_part;
    if (rel_s_i1 < 0.3) {
      // take latter part of line
      ls1_part = GetLineFromSInterval(ls1, s_i1, ls1.Length());
    } else if (rel_s_i1 > 0.7) {
      // take front part of line
      ls1_part = GetLineFromSInterval(ls1, 0, s_i1);
    } else {
      LOG(WARNING) << "Lines intersecting too much, only appending";
      ls1_part = ls1;
    }

    if (rel_s_i2 < 0.3) {
      // take latter part of line
      ls2_part = GetLineFromSInterval(ls2, s_i2, ls2.Length());
    } else if (rel_s_i2 > 0.7) {
      // take front part of line
      ls2_part = GetLineFromSInterval(ls2, 0, s_i2);
    } else {
      LOG(WARNING) << "Lines intersecting too much, only appending";
      ls2_part = ls2;
    }

    lout = ls1_part;
    lout.AppendLinestring(ls2_part);
  } else if (intersecting_points.size() > 1) {
    // do something
    LOG(ERROR) << "two intersecting points";
    lout = ls1;
    lout.AppendLinestring(ls2);
  } else {
    lout = ls1;
    lout.AppendLinestring(ls2);
  }

  lout.RemoveDuplicates();

  if (boost::geometry::intersects(lout.obj_)) {
    const double tol = 1e-6;
    LOG(WARNING) << "AppendLinesNoIntersect yields self intersecting line, "
                    "will simplify it with "
                 << tol;
    VLOG(5) << "ls1: " << ls1.ToArray();
    VLOG(5) << "ls2: " << ls2.ToArray();
    VLOG(5) << "lout: " << lout.ToArray();

    lout = Simplify(lout, tol);
  }
  return lout;
}

inline Line ConcatenateLinestring(const Line& ls1, const Line& ls2) {
  // Get first and last points
  auto first_point_this = *ls1.begin();
  auto last_point_this = *(ls1.end() - 1);
  auto first_point_other = *ls2.begin();
  auto last_point_other = *(ls2.end() - 1);

  double d_first_first = Distance(first_point_this, first_point_other);
  double d_first_last = Distance(first_point_this, last_point_other);
  double d_last_first = Distance(last_point_this, first_point_other);
  double d_last_last = Distance(last_point_this, last_point_other);

  Line lconcat;
  if (d_first_first <= std::min({d_first_last, d_last_first, d_last_last})) {
    // Reverse this
    lconcat = AppendLinesNoIntersect(Reverse(ls1), ls2);
  } else if (d_first_last <=
             std::min({d_first_first, d_last_first, d_last_last})) {
    // Reverse both
    lconcat = AppendLinesNoIntersect(Reverse(ls1), Reverse(ls2));
  } else if (d_last_first <=
             std::min({d_first_first, d_first_last, d_last_last})) {
    // No reversing
    lconcat = AppendLinesNoIntersect(ls1, ls2);
  } else {
    // Reverse other
    lconcat = AppendLinesNoIntersect(ls1, Reverse(ls2));
  }
  return lconcat;
}

// Subsampling using spline
inline Line SmoothLine(const Line& l, const double ds) {
  if (l.size() < 3) {
    LOG(WARNING) << "cannot subsample line with only 3 points";
    return l;
  } else {
    int num_points = l.Length() / ds;

    tk::spline splineX, splineY;
    std::vector<double> xVec, yVec;
    for (size_t i = 0; i < l.obj_.size(); i++) {
      xVec.push_back(bg::get<0>(l.obj_[i]));
      yVec.push_back(bg::get<1>(l.obj_[i]));
    }
    std::vector<double> sVec(l.s_.begin(), l.s_.end());
    splineX.set_points(sVec, xVec);
    splineY.set_points(sVec, yVec);

    Line lss;
    for (size_t j = 0; j <= num_points; ++j) {
      double x = splineX(j * ds);
      double y = splineY(j * ds);
      lss.AddPoint(geometry::Point2d(x, y));
    }
    if (l.Length() > num_points * ds) {
      lss.AddPoint(l.obj_.at(l.size() - 1));
    }
    return lss;
  }
}

inline Line ComputeCenterLine(const Line& outer_line, const Line& inner_line) {
  if (boost::geometry::intersects(outer_line.obj_)) {
    LOG(WARNING) << "Computing center line, but outer line self-intersects";
  }
  if (boost::geometry::intersects(inner_line.obj_)) {
    LOG(WARNING) << "Computing center line, but inner line self-intersects";
  }

  Line center_line_;
  Line line_more_points = outer_line;
  Line line_less_points = inner_line;
  if (inner_line.obj_.size() > outer_line.obj_.size()) {
    line_more_points = inner_line;
    line_less_points = outer_line;
  }
  for (Point2d& point_loop : line_more_points.obj_) {
    Point2d nearest_point_other =
        geometry::GetNearestPoint(line_less_points, point_loop);
    geometry::Point2d middle_point = (point_loop + nearest_point_other) / 2;
    center_line_.AddPoint(middle_point);
  }
  return center_line_;
}

template <typename T>
std::pair<T, T> MergeBoundingBoxes(std::pair<T, T> bb1, std::pair<T, T> bb2) {
  Line_t<T> line;  // just use a line and add all points
  line.AddPoint(bb1.first);
  line.AddPoint(bb1.second);
  line.AddPoint(bb2.first);
  line.AddPoint(bb2.second);
  return line.BoundingBox();
}

}  // namespace geometry
}  // namespace bark

#endif  // BARK_GEOMETRY_LINE_HPP_
