// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_GEOMETRY_POLYGON_HPP_
#define MODULES_GEOMETRY_POLYGON_HPP_

#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include "commons.hpp"
#include "modules/geometry/line.hpp"

namespace modules {
namespace geometry {

//! templated polygon class with a boost polygon as a member function
template <typename T>
struct Polygon_t : public Shape<bg::model::polygon<T>, T> {
  Polygon_t();
  virtual ~Polygon_t(){};
  Polygon_t(const Pose& center, const std::vector<T> points);
  Polygon_t(const Pose& center,
            const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& points);
  Polygon_t(const Pose& center,
            const Line_t<T>&
                line);  //! create a polygon from a line enclosing the polygon
  virtual Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> toArray() const;

  virtual std::shared_ptr<Shape<bg::model::polygon<T>, T>> Clone() const;

  void UpdateDistancesToCenter();

  float rear_dist_;
  float front_dist_;
  float left_dist_;
  float right_dist_;
};

template <typename T>
inline Polygon_t<T>::Polygon_t()
    : Shape<bg::model::polygon<T>, T>(Pose(0, 0, 0), std::vector<T>(), 0),
      rear_dist_(0.0f),
      front_dist_(0.0f),
      left_dist_(0.0f),
      right_dist_(0.0f) {}

template <typename T>
inline Polygon_t<T>::Polygon_t(const Pose& center, const std::vector<T> points)
    : Shape<bg::model::polygon<T>, T>(center, points, 0),
      rear_dist_(0.0f),
      front_dist_(0.0f),
      left_dist_(0.0f),
      right_dist_(0.0f) {
  boost::geometry::correct(Shape<bg::model::polygon<T>, T>::obj_);
  UpdateDistancesToCenter();
}

template <typename T>
inline Polygon_t<T>::Polygon_t(
    const Pose& center,
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& points)
    : Shape<bg::model::polygon<T>, T>(center, points, 0),
      rear_dist_(0.0f),
      front_dist_(0.0f),
      left_dist_(0.0f),
      right_dist_(0.0f) {
  boost::geometry::correct(Shape<bg::model::polygon<T>, T>::obj_);
  UpdateDistancesToCenter();
}

template <typename T>
inline Polygon_t<T>::Polygon_t(const Pose& center, const Line_t<T>& line)
    : Shape<bg::model::polygon<T>, T>(center, std::vector<T>(), 0),
      rear_dist_(0.0f),
      front_dist_(0.0f),
      left_dist_(0.0f),
      right_dist_(0.0f) {
  for (const T& next_pt : line.obj_) {
    Shape<bg::model::polygon<T>, T>::add_point(next_pt);
  }
  boost::geometry::correct(Shape<bg::model::polygon<T>, T>::obj_);
  UpdateDistancesToCenter();
}

template <typename T>
void Polygon_t<T>::UpdateDistancesToCenter() {
  boost::geometry::model::box<T> box;
  boost::geometry::envelope(Shape<bg::model::polygon<T>, T>::obj_, box);

  boost::geometry::correct(box);
  float center_x = Shape<bg::model::polygon<T>, T>::center_[0];
  float center_y = Shape<bg::model::polygon<T>, T>::center_[1];

  rear_dist_ = std::abs(bg::get<bg::min_corner, 0>(box) - center_x);
  front_dist_ = std::abs(bg::get<bg::max_corner, 0>(box) - center_x);
  left_dist_ = std::abs(bg::get<bg::min_corner, 1>(box) - center_y);
  right_dist_ = std::abs(bg::get<bg::max_corner, 1>(box) - center_y);
}

template <typename T>
inline std::shared_ptr<Shape<bg::model::polygon<T>, T>> Polygon_t<T>::Clone()
    const {
  std::shared_ptr<Polygon_t<T>> new_poly =
      std::make_shared<Polygon_t<T>>(*this);
  return new_poly;
}

//! for better usage simple float defines
using PolygonPoint = Point2d;  // for internal stores of collision checkers
using Polygon = Polygon_t<PolygonPoint>;

template <>
inline Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Polygon::toArray()
    const {
  std::vector<Point2d> points = obj_.outer();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mat(points.size(), 2);
  for (std::vector<Point2d>::size_type i = 0; i < points.size(); ++i) {
    mat.row(i) << bg::get<0>(points[i]), bg::get<1>(points[i]);
  }
  return mat;
}

inline bool equals(const Polygon& poly1, const Polygon& poly2) {
  return bg::equals(poly1.obj_, poly2.obj_);
}

inline float distance(const Polygon& poly, const Point2d& p) {
  return bg::distance(poly.obj_, p);
}

inline float distance(const Polygon& poly, const Line& l) {
  return bg::distance(poly.obj_, l.obj_);
}

inline float distance(const Polygon& poly1, const Polygon& poly2) {
  return bg::distance(poly1.obj_, poly2.obj_);
}

//! Polygon - Point collision checker using boost::within
inline bool Collide(const Polygon& poly, const PolygonPoint& p) {
  return bg::within(p, poly.obj_);
}

//! Point - Polygon collision checker using boost::within
inline bool Collide(const PolygonPoint& p, const Polygon& poly) {
  return Collide(poly, p);
}

//! Polygon within polygon check, true if g1 is completely contained within g2,
//! else false
inline bool Within(const Polygon& g1, const Polygon& g2) {
  return bg::within(g1.obj_, g2.obj_);
}

//! Point2d within polygon check, true if g1 is completely contained within g2,
//! else false
inline bool Within(const Point2d& g1, const Polygon& g2) {
  return bg::within(g1, g2.obj_);
}

//! Line2d within polygon check, true if g1 is completely contained within g2,
//! else false
inline bool Within(const Line& l, const Polygon& poly) {
  return bg::within(l.obj_, poly.obj_);
}

//! Polygon - Line collision checker using boost::intersection
//! @note we only check the shape intersection(s) and no line RHS/LHS line
//! crossing!
inline bool Collide(const Polygon& poly, const Line& l) {
  std::vector<bg::model::linestring<LinePoint>> shape_intersect;
  bg::intersection(poly.obj_, l.obj_, shape_intersect);
  const bool inner_intersection = !shape_intersect.empty();
  if (inner_intersection) {
    return inner_intersection;
  } else {
    // boost interection does not treat edge intersections as intersection1,
    // but this shall be a collision! -> cast poly edge to line and re-check
    // collision.
    Line outer_polyline;
    //! @todo geht das eleganter?
    for (auto it = boost::begin(boost::geometry::exterior_ring(poly.obj_));
         it != boost::end(boost::geometry::exterior_ring(poly.obj_)); ++it) {
      outer_polyline.add_point(*it);
    }
    return Collide(outer_polyline, l);
  }
}

//! Line - Polygon collision checker using boost::intersection
inline bool Collide(const Line& line, const Polygon& poly) {
  return Collide(poly, line);
}

//! Polygon - Polygon collision checker using boost::intersection
//! @todo might not be very efficient without Strategy...
inline bool Collide(const Polygon& poly1, const Polygon& poly2) {
  std::vector<bg::model::polygon<PolygonPoint>> shape_intersect;
  bg::intersection(poly1.obj_, poly2.obj_, shape_intersect);
  const bool inner_intersection = !shape_intersect.empty();
  if (inner_intersection) {
    return inner_intersection;
  } else {
    // boost interection does not treat edge intersections as intersection,
    // but this shall be a collision! -> cast poly edge to line and re-check
    // collision.
    Line outer_polyline1;
    Line outer_polyline2;
    //! @todo geht das eleganter?
    for (auto it = boost::begin(boost::geometry::exterior_ring(poly1.obj_));
         it != boost::end(boost::geometry::exterior_ring(poly1.obj_)); ++it) {
      outer_polyline1.add_point(*it);
    }
    for (auto it = boost::begin(boost::geometry::exterior_ring(poly2.obj_));
         it != boost::end(boost::geometry::exterior_ring(poly2.obj_)); ++it) {
      outer_polyline2.add_point(*it);
    }
    return Collide(outer_polyline1, outer_polyline2);
  }
}

inline bool ShrinkPolygon(const Polygon& polygon, const double distance,
                          Polygon& shrunk_polygon) {
  namespace bg = boost::geometry;
  namespace bbuf = bg::strategy::buffer;

  bbuf::distance_symmetric<float> distance_strategy(distance);
  bbuf::side_straight side_strategy;
  bbuf::join_miter join_strategy;
  bbuf::end_flat end_strategy;
  bbuf::point_circle point_strategy;

  bg::model::multi_polygon<bg::model::polygon<geometry::Point2d>>
      shrunk_polygons;
  bg::buffer(polygon.obj_, shrunk_polygons, distance_strategy, side_strategy,
             join_strategy, end_strategy, point_strategy);

  if (shrunk_polygons.size() != 1) {
    // Shrinking the polygon turns it into two disjointed polygons
    return false;
  }

  for (auto const& point :
       boost::make_iterator_range(bg::exterior_ring(shrunk_polygons.front()))) {
    shrunk_polygon.add_point(point);
  }
  assert(shrunk_polygon.Valid());
  return true;
}

}  // namespace geometry
}  // namespace modules

#endif  // MODULES_GEOMETRY_POLYGON_HPP_
