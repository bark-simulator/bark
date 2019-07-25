// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "modules/world/map/local_map.hpp"


TEST(driving_corridor_frenet, local_map) {
  using namespace modules::world::map;
  using namespace modules::geometry;

  // some line with three points from x=1 to x=10, y=0
  Line line;
  line.add_point(Point2d(1,0));
  line.add_point(Point2d(2,0));
  line.add_point(Point2d(10,0));

  DrivingCorridor corridor;
  corridor.set_center(line);

  
  Frenet frenet = corridor.FrenetFromCenterLine(Point2d(5,2.5));

  EXPECT_EQ(frenet.lon, 4);
  EXPECT_EQ(frenet.lat, 2.5);

  Frenet frenet2 = corridor.FrenetFromCenterLine(Point2d(1,2.5));

  EXPECT_EQ(frenet2.lon, 0);
  EXPECT_EQ(frenet2.lat, 2.5);

  Frenet frenet3 = corridor.FrenetFromCenterLine(Point2d(10, 0));

  EXPECT_EQ(frenet3.lon, 9);
  EXPECT_EQ(frenet3.lat, 0);


  Frenet frenet4 = corridor.FrenetFromCenterLine(Point2d(3.5,-4));

  EXPECT_EQ(frenet4.lon, 2.5);
  EXPECT_EQ(frenet4.lat, -4);

}