// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_MAP_FRENET_HPP_
#define MODULES_WORLD_MAP_FRENET_HPP_

#include "modules/geometry/line.hpp"


namespace modules {
namespace world {
namespace map {

struct Frenet {

Frenet() : lon(0.0f), lat(0.0f) {}
Frenet(const double& longitudinal, const double& lateral) : lon(longitudinal), lat(lateral) {}
Frenet(const modules::geometry::Point2d& position, const modules::geometry::Line& path);

double lon;
double lat;
};

}  // namespace map
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_MAP_FRENET_HPP_