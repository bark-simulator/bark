// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_COMMONS_TRANSFORMATION_FRENET_HPP_
#define MODULES_COMMONS_TRANSFORMATION_FRENET_HPP_

#include "modules/geometry/line.hpp"


namespace modules {
namespace commons {
namespace transformation {

struct FrenetPosition {
  FrenetPosition() : lon(0.0f), lat(0.0f) {}
  FrenetPosition(const double& longitudinal, const double& lateral) :
    lon(longitudinal), lat(lateral) {}
  FrenetPosition(
    const modules::geometry::Point2d& position,
    const modules::geometry::Line& path);

  double lon;
  double lat;
};

}  // namespace transformation
}  // namespace commons
}  // namespace modules

#endif  // MODULES_COMMONS_TRANSFORMATION_FRENET_HPP_