// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_GEOMETRY_STANDARD_SHAPES_HPP_
#define BARK_GEOMETRY_STANDARD_SHAPES_HPP_

#include "bark/geometry/polygon.hpp"

namespace bark {
namespace geometry {
namespace standard_shapes {

Polygon CarLimousine();

// Creates Car Rectangle from wheelbase l and collision radius r
Polygon GenerateCarLimousine(double wheelbase, double collision_radius);

Polygon CarRectangle();

// Creates Car Rectangle from wheelbase l and collision radius r
Polygon GenerateCarRectangle(double wheelbase, double collision_radius);

Polygon GenerateGoalRectangle(double length, double width);

// TODO(@all): add more variants and objects here

}  // namespace standard_shapes
}  // namespace geometry
}  // namespace bark

#endif  // BARK_GEOMETRY_STANDARD_SHAPES_HPP_
