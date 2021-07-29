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
using Point2d = Point2d_t<double>;

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
inline Point2d operator+(const Point2d& lhs, const double& rhs) {
  return Point2d(bg::get<0>(lhs) + rhs, bg::get<1>(lhs) + rhs);
}
inline Point2d operator-(const Point2d& lhs, const Point2d& rhs) {
  return Point2d(bg::get<0>(lhs) - bg::get<0>(rhs),
                 bg::get<1>(lhs) - bg::get<1>(rhs));
}
inline Point2d operator-(const Point2d& lhs, const double& rhs) {
  return Point2d(bg::get<0>(lhs) - rhs, bg::get<1>(lhs) - rhs);
}

inline Point2d operator*(const Point2d& point, const double& factor) {
  return Point2d(bg::get<0>(point) * factor, bg::get<1>(point) * factor);
}
inline Point2d operator/(const Point2d& point, const double& divisor) {
  return Point2d(bg::get<0>(point) / divisor, bg::get<1>(point) / divisor);
}

using Pose = Eigen::Vector3d;

inline std::string print(const Point2d& p) {
  std::stringstream ss;
  ss << "Point2d: x: " << bg::get<0>(p) << ", y: " << bg::get<1>(p)
     << std::endl;
  return ss.str();
}

inline double Distance(const Point2d& p1, const Point2d& p2) {
  double dx = bg::get<0>(p1) - bg::get<0>(p2);
  double dy = bg::get<1>(p1) - bg::get<1>(p2);
  return sqrt(dx * dx + dy * dy);
}

template <typename G, typename T>
struct Shape {
  Shape(const Pose& center, std::vector<T> points, int32_t id)
      : obj_(), id_(id), center_(center) {
    for (auto it = points.begin(); it != points.end(); ++it) AddPoint(*it);
  }

  Shape(const Pose& center,
        const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& points,
        int32_t id)
      : obj_(), id_(id), center_(center) {
    auto row_num = points.rows();
    for (auto rowIter = 0; rowIter < row_num; ++rowIter) {
      AddPoint(T(points.coeff(rowIter, 0), points.coeff(rowIter, 1)));
    }
  }

  Shape(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& points,
        int32_t id)
      : obj_(), id_(id) {
    auto row_num = points.rows();
    for (auto rowIter = 0; rowIter < row_num; ++rowIter) {
      AddPoint(T(points.coeff(rowIter, 0), points.coeff(rowIter, 1)));
    }
  }

  virtual ~Shape() {}
  virtual std::shared_ptr<Shape> Clone() const = 0;
  virtual std::string ShapeToString() const;

  // translates, scales, and rotates object
  std::shared_ptr<Shape<G, T>> ScalingTransform(const double& scaling_factor,
                                                const Pose& pose) const;

  // rotates object
  std::shared_ptr<Shape<G, T>> Rotate(const double& a) const;

  // translates object
  std::shared_ptr<Shape<G, T>> Translate(const Point2d& point) const;

  // return object transform
  std::shared_ptr<Shape<G, T>> Transform(const Pose& pose) const;

  // Scales in x- and y-direction separately
  virtual std::shared_ptr<Shape<G, T>> Scale(const double& x_dir, const double& y_) const;

  // Inflates in x- and y-direction separately
  virtual std::shared_ptr<Shape<G, T>> Inflate(const double& x_dir, const double& y_dir) const;

  virtual bool Valid() const;

  virtual Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ToArray()
      const = 0;

  bool AddPoint(const T& p) {
    bg::append(obj_, p);
    return true;
  }

  std::vector<T> GetPoints() const {
      std::vector<T> points;
      std::copy(Shape<G, T>::obj_.begin(),
          Shape<G, T>::obj_.end(),
          std::back_inserter(points));
        return points;
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
inline std::shared_ptr<Shape<G, T>> Shape<G, T>::Rotate(const double& a) const {
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
inline std::shared_ptr<Shape<G, T>> Shape<G, T>::ScalingTransform(
    const double& scaling_factor, const Pose& pose) const {
  namespace trans = boost::geometry::strategy::transform;
  // move shape relative to coordinate center
  trans::translate_transformer<double, 2, 2> translate_rel_to_center(
      -center_[0], -center_[1]);
  G obj_rel_translated;
  boost::geometry::transform(obj_, obj_rel_translated, translate_rel_to_center);

  trans::scale_transformer<double, 2, 2> scale(scaling_factor);
  G obj_scaled;
  boost::geometry::transform(obj_rel_translated, obj_scaled, scale);

  // rotate (counterclockwise)
  trans::rotate_transformer<boost::geometry::radian, double, 2, 2> rotate(
      -pose[2]);
  G obj_rotated;
  boost::geometry::transform(obj_scaled, obj_rotated, rotate);

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
inline std::shared_ptr<Shape<G, T>> Shape<G, T>::Scale(const double& x_dir, const double& y_dir) const {
  namespace trans = boost::geometry::strategy::transform;
  // move shape relative to coordinate center
  trans::translate_transformer<double, 2, 2> translate_rel_to_center(
      -center_[0], -center_[1]);
  G obj_rel_translated;
  boost::geometry::transform(obj_, obj_rel_translated, translate_rel_to_center);

  // rotate so that orientation is aligned with x-axis
  trans::rotate_transformer<boost::geometry::radian, double, 2, 2> rotate(
      center_[2]);
  G obj_rotated;
  boost::geometry::transform(obj_rel_translated, obj_rotated, rotate);

  // Do longitudinal and lateral scaling
  const auto x_m_scale = x_dir == 0.0 ? 1.0 : x_dir;
  const auto y_m_scale = y_dir == 0.0 ? 1.0 : y_dir;
  trans::matrix_transformer<double, 2, 2> scale_transform(x_m_scale, 0.0, 0.0, 0.0, y_m_scale, 0.0, 0.0, 0.0, 0.0);
  G obj_scaled;
  boost::geometry::transform(obj_rotated, obj_scaled, scale_transform);

  // rotate back according to center specification
  trans::rotate_transformer<boost::geometry::radian, double, 2, 2> back_rotate(
      -center_[2]);
  G obj_back_rotated;
  boost::geometry::transform(obj_scaled, obj_back_rotated, back_rotate);

  // move object according to center specification
  trans::translate_transformer<double, 2, 2> translate_backwards(
      center_[0], center_[1]);
  G obj_transformed;
  boost::geometry::transform(obj_back_rotated, obj_transformed, translate_backwards);

  std::shared_ptr<Shape<G, T>> shape_transformed = this->Clone();
  shape_transformed->obj_ = obj_transformed;
  return shape_transformed;
}

template <typename G, typename T>
inline std::shared_ptr<Shape<G, T>> Shape<G, T>::Inflate(const double& x_dir, const double& y_dir) const {
  namespace trans = boost::geometry::strategy::transform;
  // move shape relative to coordinate center
  trans::translate_transformer<double, 2, 2> translate_rel_to_center(
      -center_[0], -center_[1]);
  G obj_rel_translated;
  boost::geometry::transform(obj_, obj_rel_translated, translate_rel_to_center);

  // rotate so that orientation is aligned with x-axis
  trans::rotate_transformer<boost::geometry::radian, double, 2, 2> rotate(
      center_[2]);
  G obj_rotated;
  boost::geometry::transform(obj_rel_translated, obj_rotated, rotate);

  // Do x and y inflating
  const auto get_inflated = [&](const T& p) {
    T p_inflated;
    if (bg::get<0>(p) >= 0.0 && bg::get<1>(p) >= 0.0)
      p_inflated = T(bg::get<0>(p) + x_dir, bg::get<1>(p) + y_dir);
    else if (bg::get<0>(p) >= 0.0 && bg::get<1>(p) < 0.0)
      p_inflated = T(bg::get<0>(p) + x_dir, bg::get<1>(p) - y_dir);
    else if (bg::get<0>(p) < 0.0 && bg::get<1>(p) < 0.0)
      p_inflated = T(bg::get<0>(p) - x_dir, bg::get<1>(p) -y_dir);
    else 
      p_inflated = T(bg::get<0>(p) - x_dir, bg::get<1>(p) + y_dir);
    return p_inflated;
  };
  G inflated_object;
  if constexpr (std::is_same<G, bg::model::polygon<T>>::value) {
    const auto& points = obj_rotated.outer();
    for (const auto& p : points) {
      bg::append(inflated_object, get_inflated(p));
    }
  }
  else if constexpr (std::is_same<G, bg::model::linestring<T>>::value) {
    for (const auto& p : obj_rotated) {
      bg::append(inflated_object, get_inflated(p));
    }
  }

  // rotate back according to center specification
  trans::rotate_transformer<boost::geometry::radian, double, 2, 2> back_rotate(
      -center_[2]);
  G obj_back_rotated;
  boost::geometry::transform(inflated_object, obj_back_rotated, back_rotate);

  // move object according to center specification
  trans::translate_transformer<double, 2, 2> translate_backwards(
      center_[0], center_[1]);
  G obj_transformed;
  boost::geometry::transform(obj_back_rotated, obj_transformed, translate_backwards);

  std::shared_ptr<Shape<G, T>> shape_transformed = this->Clone();
  shape_transformed->obj_ = obj_transformed;
  return shape_transformed;
}

template <typename G, typename T>
inline std::string Shape<G, T>::ShapeToString() const {
  std::stringstream ss;
  Eigen::IOFormat OctaveFmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[",
                            "]");
  ss << ToArray().format(OctaveFmt);
  return ss.str();
}

template <typename G, typename T>
std::vector<Point2d> Intersection(const G& g1, const T& g2) {
  std::vector<Point2d> intersecting_points;
  bg::intersection(g1.obj_, g2.obj_, intersecting_points);
  return intersecting_points;
}

}  // namespace geometry
}  // namespace bark

#endif  // BARK_GEOMETRY_COMMONS_HPP_
