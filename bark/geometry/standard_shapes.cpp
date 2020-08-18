// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/geometry/standard_shapes.hpp"
#include <vector>

namespace bark {
namespace geometry {

Polygon bark::geometry::standard_shapes::CarLimousine() {
  return Polygon(
      Pose(0, 0, 0),
      std::vector<Point2d>{
          Point2d(3.85, 0), Point2d(3.772, -0.495), Point2d(3.426, -0.887),
          Point2d(2.914, -0.956), Point2d(1.457, -0.956), Point2d(0, -0.956),
          Point2d(-0.512, -0.886), Point2d(-1.02, -0.589), Point2d(-1.119, 0),
          Point2d(-1.02, 0.589), Point2d(-0.512, 0.886), Point2d(0, 0.956),
          Point2d(1.457, 0.956), Point2d(2.914, 0.956), Point2d(3.426, 0.887),
          Point2d(3.772, 0.495), Point2d(3.85, 0)});
}

Polygon bark::geometry::standard_shapes::CarRectangle() {
  return Polygon(
      Pose(1.25, 0, 0),
      std::vector<Point2d>{Point2d(-1, -1), Point2d(-1, 1), Point2d(3, 1),
                           Point2d(3, -1), Point2d(-1, -1)});
}

Polygon bark::geometry::standard_shapes::GenerateGoalRectangle(float length, float width) {
  return Polygon(
      Pose(width/2, length/2, 0),
      std::vector<Point2d>{Point2d(0, 0), Point2d(0, length), Point2d(width, length),
                           Point2d(width, 0), Point2d(0, 0)});
}

}  // namespace geometry
}  // namespace bark
