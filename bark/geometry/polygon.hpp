// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_GEOMETRY_POLYGON_HPP_
#define BARK_GEOMETRY_POLYGON_HPP_

#include <memory>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include "bark/geometry/line.hpp"
#include "commons.hpp"

namespace bark {
namespace geometry {

//! templated polygon class with a boost polygon as a member function
template <typename T>
struct Polygon_t : public Shape<bg::model::polygon<T>, T> {
  Polygon_t();
  virtual ~Polygon_t() {}
  Polygon_t(const Pose& center, const std::vector<T> points);
  Polygon_t(const Pose& center,
            const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& points);
  Polygon_t(const Pose& center,
            const Line_t<T>&
                line);  //! create a polygon from a line enclosing the polygon
  virtual Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> ToArray() const;
  virtual float CalculateArea() const;

  virtual std::shared_ptr<Shape<bg::model::polygon<T>, T>> Clone() const;

  void UpdateDistancesToCenter();

  void ConcatenatePolygons(Polygon_t<T> poly) {
    std::vector<boost::geometry::model::polygon<Point2d>> merged_polygon;
    boost::geometry::correct(this->obj_);
    boost::geometry::correct(poly.obj_);
    boost::geometry::union_(this->obj_, poly.obj_, merged_polygon);
    if (merged_polygon.size() > 0) {
      Shape<bg::model::polygon<T>, T>::obj_ = merged_polygon[0];
    }
  }

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
    Shape<bg::model::polygon<T>, T>::AddPoint(next_pt);
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
inline Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Polygon::ToArray()
    const {
  std::vector<Point2d> points = obj_.outer();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mat(points.size(), 2);
  for (std::vector<Point2d>::size_type i = 0; i < points.size(); ++i) {
    mat.row(i) << bg::get<0>(points[i]), bg::get<1>(points[i]);
  }
  return mat;
}

template <>
inline float Polygon::CalculateArea() const {
  return bg::area(obj_);
}

inline bool Equals(const Polygon& poly1, const Polygon& poly2) {
  return bg::equals(poly1.obj_, poly2.obj_);
}

inline float Distance(const Polygon& poly, const Point2d& p) {
  return bg::distance(poly.obj_, p);
}

inline float Distance(const Polygon& poly, const Line& l) {
  return bg::distance(poly.obj_, l.obj_);
}

inline float Distance(const Polygon& poly1, const Polygon& poly2) {
  return bg::distance(poly1.obj_, poly2.obj_);
}

//! Polygon - Point collision checker using boost::covered_by
//! As opposed to boost::whithin, also considers points on the boundary lines
inline bool Collide(const Polygon& poly, const PolygonPoint& p) {
  return bg::covered_by(p, poly.obj_);
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
  return bg::intersects(poly.obj_, l.obj_);
}

//! Line - Polygon collision checker using boost::intersection
inline bool Collide(const Line& line, const Polygon& poly) {
  return Collide(poly, line);
}

//! Polygon - Polygon collision checker using boost::intersects
inline bool Collide(const Polygon& poly1, const Polygon& poly2) {
  return bg::intersects(poly1.obj_, poly2.obj_);
}

inline bool BufferPolygon(const Polygon& polygon, const double distance,
                          Polygon* buffered_polygon) {
  namespace bg = boost::geometry;
  namespace bbuf = bg::strategy::buffer;

  bbuf::distance_symmetric<float> distance_strategy(distance);
  bbuf::side_straight side_strategy;
  bbuf::join_miter join_strategy;
  bbuf::end_flat end_strategy;
  bbuf::point_circle point_strategy;
  bg::model::multi_polygon<bg::model::polygon<geometry::Point2d>>
      buffered_polygons;
  Polygon copied_polygon = polygon;
  bg::correct(copied_polygon.obj_);
  bg::buffer(copied_polygon.obj_, buffered_polygons, distance_strategy,
             side_strategy, join_strategy, end_strategy, point_strategy);
  bg::correct(buffered_polygons);
  if (buffered_polygons.size() != 1) {
    // Shrinking the polygon turns it into two disjointed polygons
    return false;
  }
  for (auto const& point : boost::make_iterator_range(
           bg::exterior_ring(buffered_polygons.front()))) {
    buffered_polygon->AddPoint(point);
  }
  if (!buffered_polygon->Valid()) {
    LOG(INFO) << "Buffered polygon is not valid.";
  }
  return true;
}

}  // namespace geometry
}  // namespace bark

#endif  // BARK_GEOMETRY_POLYGON_HPP_
