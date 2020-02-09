// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_TESTS_MAKE_TEST_XODR_MAP_HPP_
#define MODULES_WORLD_TESTS_MAKE_TEST_XODR_MAP_HPP_

#include "modules/world/opendrive/opendrive.hpp"

namespace modules {
namespace world {
namespace tests {

using modules::world::opendrive::OpenDriveMapPtr;

OpenDriveMapPtr MakeXodrMapOneRoadTwoLanes();

OpenDriveMapPtr MakeXodrMapTwoRoadsOneLane();

}  // namespace tests
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_TESTS_MAKE_TEST_XODR_MAP_HPP_