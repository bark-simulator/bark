// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
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
  Polygon_t(const Pose &center, const std::vector<T> points);
  Polygon_t(const Pose &center, const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &points);

  virtual Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> toArray() const;

  virtual Shape<bg::model::polygon<T>, T> *Clone() const;

  void UpdateDistancesToCenter();

  float rear_dist_;
  float front_dist_;
  float left_dist_;
  float right_dist_;
};

template<typename T>
inline Polygon_t<T>::Polygon_t() : Shape<bg::model::polygon<T>, T>(Pose(0, 0, 0), std::vector<T>(), 0),
                rear_dist_(0.0f),
                front_dist_(0.0f),
                left_dist_(0.0f),
                right_dist_(0.0f) {}

template<typename T>
inline Polygon_t<T>::Polygon_t(const Pose &center, const std::vector<T> points) :
           Shape<bg::model::polygon<T>, T>(center, points, 0),
           rear_dist_(0.0f),
           front_dist_(0.0f),
           left_dist_(0.0f),
           right_dist_(0.0f) {
    UpdateDistancesToCenter();
}

template<typename T>
inline Polygon_t<T>::Polygon_t(const Pose &center,
      const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &points) :
           Shape<bg::model::polygon<T>, T>(center, points, 0),
           rear_dist_(0.0f),
           front_dist_(0.0f),
           left_dist_(0.0f),
           right_dist_(0.0f) {
    UpdateDistancesToCenter();
}

template<typename T>
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
inline Shape<bg::model::polygon<T>, T> *Polygon_t<T>::Clone() const {
  return new Polygon_t<T>(*this);
}

//! for better usage simple float defines
using PolygonPoint = Point2d;  // for internal stores of collision checkers
using Polygon = Polygon_t<PolygonPoint>;

template <>
inline Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Polygon::toArray() const {
  std::vector<Point2d> points = obj_.outer();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mat(points.size(), 2);
  for (std::vector<Point2d>::size_type i = 0; i < points.size(); ++i) {
    mat.row(i) << bg::get<0>(points[i]), bg::get<1>(points[i]);
  }
  return mat;
}

inline bool equals(const Polygon &poly1, const Polygon &poly2) {
  return bg::equals(poly1.obj_, poly2.obj_);
}

inline float distance(const Polygon &poly, const Point2d &p) {
  return bg::distance(poly.obj_, p);
}

inline float distance(const Polygon &poly, const Line &l) {
  return bg::distance(poly.obj_, l.obj_);
}

inline float distance(const Polygon &poly1, const Polygon &poly2) {
  return bg::distance(poly1.obj_, poly2.obj_);
}

//! Polygon - Point collision checker using boost::within
inline bool Collide(const Polygon &poly, const PolygonPoint &p) {
  return bg::within(p, poly.obj_);
}

//! Point - Polygon collision checker using boost::within
inline bool Collide(const PolygonPoint &p, const Polygon &poly) {
  return Collide(poly, p);
}

//! Polygon - Line collision checker using boost::intersection
//! @note we only check the shape intersection(s) and no line RHS/LHS line crossing!
inline bool Collide(const Polygon &poly, const Line &l) {
  std::vector<bg::model::linestring<LinePoint>> shape_intersect;
  bg::intersection(poly.obj_, l.obj_, shape_intersect);
  const bool inner_intersection = !shape_intersect.empty();
  if (inner_intersection) {
    return inner_intersection;
  } else {
    // boost interection does not treat edge intersections as intersection1,
    // but this shall be a collision! -> cast poly edge to line and re-check collision.
    Line outer_polyline;
    //! @todo geht das eleganter?
    for (auto it = boost::begin(boost::geometry::exterior_ring(poly.obj_)); it != boost::end(boost::geometry::exterior_ring(poly.obj_)); ++it) {
      outer_polyline.add_point(*it);
    }
    return Collide(outer_polyline, l);
  }
}

//! Line - Polygon collision checker using boost::intersection
inline bool Collide(const Line &line, const Polygon &poly) {
  return Collide(poly, line);
}

//! Polygon - Polygon collision checker using boost::intersection
//! @todo might not be very efficient without Strategy...
inline bool Collide(const Polygon &poly1, const Polygon &poly2) {
  std::vector<bg::model::polygon<PolygonPoint>> shape_intersect;
  bg::intersection(poly1.obj_, poly2.obj_, shape_intersect);
  const bool inner_intersection = !shape_intersect.empty();
  if (inner_intersection) {
    return inner_intersection;
  } else {
    // boost interection does not treat edge intersections as intersection,
    // but this shall be a collision! -> cast poly edge to line and re-check collision.
    Line outer_polyline1;
    Line outer_polyline2;
    //! @todo geht das eleganter?
    for (auto it = boost::begin(boost::geometry::exterior_ring(poly1.obj_)); it != boost::end(boost::geometry::exterior_ring(poly1.obj_)); ++it) {
      outer_polyline1.add_point(*it);
    }
    for (auto it = boost::begin(boost::geometry::exterior_ring(poly2.obj_)); it != boost::end(boost::geometry::exterior_ring(poly2.obj_)); ++it) {
      outer_polyline2.add_point(*it);
    }
    return Collide(outer_polyline1, outer_polyline2);
  }
}

inline void intersection(const Polygon &polygon1, const Polygon &polygon2, std::vector<Polygon> &polygons_out) {
  if (!polygon1.Valid() || !polygon2.Valid()) {
    throw std::runtime_error("Trying to intersect invalid polygons");
  }
  std::deque<bg::model::polygon<PolygonPoint>> bg_polygons_out;
  bg::intersection(polygon1.obj_, polygon2.obj_, bg_polygons_out);
  for (auto const &bg_polygon_out : bg_polygons_out) {
    PolygonPoint center;
    bg::centroid(bg_polygon_out, center);
    Polygon polygon_out(Pose(bg::get<0>(center), bg::get<1>(center), 0), bg_polygon_out.outer());
    polygons_out.push_back(polygon_out);
  }
}

inline void union_(const Polygon &polygon1, const Polygon &polygon2, std::vector<Polygon> &polygons_out) {
  if (!polygon1.Valid() || !polygon2.Valid()) {
    throw std::runtime_error("Trying to union invalid polygons");
  }
  std::deque<bg::model::polygon<PolygonPoint>> bg_polygons_out;
  bg::union_(polygon1.obj_, polygon2.obj_, bg_polygons_out);
  for (auto const &bg_polygon_out : bg_polygons_out) {
    PolygonPoint center;
    bg::centroid(bg_polygon_out, center);
    Polygon polygon_out(Pose(bg::get<0>(center), bg::get<1>(center), 0), bg_polygon_out.outer());
    polygons_out.push_back(polygon_out);
  }
}

}  // namespace geometry
}  // namespace modules

#endif  // MODULES_GEOMETRY_POLYGON_HPP_
