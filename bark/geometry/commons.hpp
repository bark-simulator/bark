// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_GEOMETRY_COMMONS_HPP_
#define BARK_GEOMETRY_COMMONS_HPP_

#include <Eigen/Core>
#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include "bark/commons/util/util.hpp"

namespace bark {
namespace geometry {

//! using boost geometry
namespace bg = boost::geometry;

//! points
template <typename T>
using Point2d_t = bg::model::point<T, 2, bg::cs::cartesian>;
using Point2d = Point2d_t<float>;

//! Point operators
inline bool operator==(const Point2d& lhs, const Point2d& rhs) {
  return (bg::get<0>(lhs) == bg::get<0>(rhs) &&
          bg::get<1>(lhs) == bg::get<1>(rhs));
}
inline bool operator!=(const Point2d& lhs, const Point2d& rhs) {
  return !(lhs == rhs);
}
inline Point2d operator+(const Point2d& lhs, const Point2d& rhs) {
  return Point2d(bg::get<0>(lhs) + bg::get<0>(rhs),
                 bg::get<1>(lhs) + bg::get<1>(rhs));
}
inline Point2d operator+(const Point2d& lhs, const float& rhs) {
  return Point2d(bg::get<0>(lhs) + rhs, bg::get<1>(lhs) + rhs);
}
inline Point2d operator-(const Point2d& lhs, const Point2d& rhs) {
  return Point2d(bg::get<0>(lhs) - bg::get<0>(rhs),
                 bg::get<1>(lhs) - bg::get<1>(rhs));
}
inline Point2d operator-(const Point2d& lhs, const float& rhs) {
  return Point2d(bg::get<0>(lhs) - rhs, bg::get<1>(lhs) - rhs);
}

inline Point2d operator*(const Point2d& point, const float& factor) {
  return Point2d(bg::get<0>(point) * factor, bg::get<1>(point) * factor);
}
inline Point2d operator/(const Point2d& point, const float& divisor) {
  return Point2d(bg::get<0>(point) / divisor, bg::get<1>(point) / divisor);
}

using Pose = Eigen::Vector3d;

inline std::string print(const Point2d& p) {
  std::stringstream ss;
  ss << "Point2d: x: " << bg::get<0>(p) << ", y: " << bg::get<1>(p)
     << std::endl;
  return ss.str();
}

inline float Distance(const Point2d& p1, const Point2d& p2) {
  float dx = bg::get<0>(p1) - bg::get<0>(p2);
  float dy = bg::get<1>(p1) - bg::get<1>(p2);
  return sqrt(dx * dx + dy * dy);
}

template <typename G, typename T>
struct Shape {
  Shape(const Pose& center, std::vector<T> points, int32_t id)
      : obj_(), id_(id), center_(center) {
    for (auto it = points.begin(); it != points.end(); ++it) AddPoint(*it);
  }

  Shape(const Pose& center,
        const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& points,
        int32_t id)
      : obj_(), id_(id), center_(center) {
    auto row_num = points.rows();
    for (auto rowIter = 0; rowIter < row_num; ++rowIter) {
      AddPoint(T(points.coeff(rowIter, 0), points.coeff(rowIter, 1)));
    }
  }

  virtual ~Shape() {}
  virtual std::shared_ptr<Shape> Clone() const = 0;
  virtual std::string ShapeToString() const;

  // rotates object
  std::shared_ptr<Shape<G, T>> Rotate(const float& a) const;

  // translates object
  std::shared_ptr<Shape<G, T>> Translate(const Point2d& point) const;

  // return object transform
  std::shared_ptr<Shape<G, T>> Transform(const Pose& pose) const;

  virtual bool Valid() const;

  virtual Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> ToArray()
      const = 0;

  bool AddPoint(const T& p) {
    bg::append(obj_, p);
    return true;
  }

  std::pair<T, T> BoundingBox() const {
    boost::geometry::model::box<T> box;
    boost::geometry::envelope(obj_, box);
    boost::geometry::correct(box);
    return std::make_pair(
        T(bg::get<bg::min_corner, 0>(box), bg::get<bg::min_corner, 1>(box)),
        T(bg::get<bg::max_corner, 0>(box), bg::get<bg::max_corner, 1>(box)));
  }

  G obj_;
  int32_t id_;
  Pose center_;  // fixed center pose of shape
};

template <typename G, typename T>
inline bool Shape<G, T>::Valid() const {
  std::string message;
  bool valid = boost::geometry::is_valid(obj_, message);
  if (!valid) {
    LOG(ERROR) << "Polygon not valid. Why not valid? " << message;
  }
  return valid;
}

template <typename G, typename T>
inline std::shared_ptr<Shape<G, T>> Shape<G, T>::Rotate(const float& a) const {
  namespace trans = boost::geometry::strategy::transform;
  // move shape relative to coordinate center
  trans::translate_transformer<double, 2, 2> translate_rel_to_center(
      -center_[0], -center_[1]);
  G obj_rel_translated;
  boost::geometry::transform(obj_, obj_rel_translated, translate_rel_to_center);

  // rotate (counterclockwise)
  trans::rotate_transformer<boost::geometry::radian, double, 2, 2> rotate(-a);
  G obj_rotated;
  boost::geometry::transform(obj_rel_translated, obj_rotated, rotate);

  // move object backwards plus translation component
  trans::translate_transformer<double, 2, 2> translate_backwards(center_[0],
                                                                 center_[1]);
  G obj_transformed;
  boost::geometry::transform(obj_rotated, obj_transformed, translate_backwards);

  std::shared_ptr<Shape<G, T>> shape_transformed = this->Clone();
  shape_transformed->obj_ = obj_transformed;
  shape_transformed->center_[2] += a;
  return shape_transformed;
}

template <typename G, typename T>
inline std::shared_ptr<Shape<G, T>> Shape<G, T>::Translate(
    const Point2d& point) const {
  namespace trans = boost::geometry::strategy::transform;
  trans::translate_transformer<double, 2, 2> translate_backwards(
      bg::get<0>(point), bg::get<1>(point));
  G obj_transformed;
  boost::geometry::transform(obj_, obj_transformed, translate_backwards);

  std::shared_ptr<Shape<G, T>> shape_transformed = this->Clone();
  shape_transformed->obj_ = obj_transformed;
  shape_transformed->center_[0] += bg::get<0>(point);
  shape_transformed->center_[1] += bg::get<1>(point);
  return shape_transformed;
}

template <typename G, typename T>
inline std::shared_ptr<Shape<G, T>> Shape<G, T>::Transform(
    const Pose& pose) const {
  namespace trans = boost::geometry::strategy::transform;
  // move shape relative to coordinate center
  trans::translate_transformer<double, 2, 2> translate_rel_to_center(
      -center_[0], -center_[1]);
  G obj_rel_translated;
  boost::geometry::transform(obj_, obj_rel_translated, translate_rel_to_center);

  // rotate (counterclockwise)
  trans::rotate_transformer<boost::geometry::radian, double, 2, 2> rotate(
      -pose[2]);
  G obj_rotated;
  boost::geometry::transform(obj_rel_translated, obj_rotated, rotate);

  // move object backwards plus translation component
  trans::translate_transformer<double, 2, 2> translate_backwards(
      center_[0] + pose[0], center_[1] + pose[1]);
  G obj_transformed;
  boost::geometry::transform(obj_rotated, obj_transformed, translate_backwards);

  std::shared_ptr<Shape<G, T>> shape_transformed = this->Clone();
  shape_transformed->obj_ = obj_transformed;
  shape_transformed->center_[0] += pose[0];
  shape_transformed->center_[1] += pose[1];
  shape_transformed->center_[2] += pose[2];
  return shape_transformed;
}

template <typename G, typename T>
inline std::string Shape<G, T>::ShapeToString() const {
  std::stringstream ss;
  Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[",
                            "]");
  ss << ToArray().format(OctaveFmt);
  return ss.str();
}

// template<typename G, typename T>
// inline bool Shape<G,T>::Collide(const G& shape1, const G& shape2)
//{
//  return Collide(shape1, shape2);
//}

}  // namespace geometry
}  // namespace bark

#endif  // BARK_GEOMETRY_COMMONS_HPP_
