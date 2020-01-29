// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_GEOMETRY_STANDARD_SHAPES_HPP_
#define MODULES_GEOMETRY_STANDARD_SHAPES_HPP_

#include "modules/geometry/polygon.hpp"

namespace modules {
namespace geometry {
namespace standard_shapes {

Polygon CarLimousine();

Polygon CarRectangle();

// TODO(fortiss): add more variants and objects here

}  // namespace standard_shapes
}  // namespace geometry
}  // namespace modules

#endif  // MODULES_GEOMETRY_STANDARD_SHAPES_HPP_
