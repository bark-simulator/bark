// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_TESTS_MAKE_TEST_XODR_MAP_HPP_
#define BARK_WORLD_TESTS_MAKE_TEST_XODR_MAP_HPP_

#include "bark/world/opendrive/opendrive.hpp"

namespace bark {
namespace world {
namespace tests {

using bark::world::opendrive::OpenDriveMapPtr;

OpenDriveMapPtr MakeXodrMapOneRoadTwoLanes();

OpenDriveMapPtr MakeXodrMapTwoRoadsOneLane();

OpenDriveMapPtr MakeXodrMapEndingLaneInParallel();

}  // namespace tests
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_TESTS_MAKE_TEST_XODR_MAP_HPP_