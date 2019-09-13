// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "modules/world/map/local_map.hpp"
#include "modules/geometry/commons.hpp"


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


TEST(corridor_polygon, local_map) {
  using namespace modules::world::map;
  using namespace modules::geometry;

  // some line with three points from x=1 to x=10, y=0
  Line center;
  center.add_point(Point2d(1,1));
  center.add_point(Point2d(2,1));
  center.add_point(Point2d(10,1));

  Line outer;
  outer.add_point(Point2d(1,2));
  outer.add_point(Point2d(2,2));
  outer.add_point(Point2d(10,2));

  Line inner;
  inner.add_point(Point2d(1,0));
  inner.add_point(Point2d(2,0));
  inner.add_point(Point2d(10,0));

  DrivingCorridor corridor(outer, inner, center);

  auto corridor_polygon = corridor.CorridorPolygon();

  auto bb = corridor_polygon.bounding_box();
  EXPECT_TRUE(bb.first== Point2d(1,0));
  EXPECT_TRUE(bb.second == Point2d(10,2));

  Polygon ptest(Pose(0, 0, 0), std::vector<Point2d>{Point2d(0, 0), Point2d(0, 2), Point2d(4, 2), Point2d(4, 0), Point2d(0, 0)});

  EXPECT_TRUE(modules::geometry::Collide(ptest, corridor_polygon));
}

TEST(calculate_horizon, local_map) {
  using namespace modules::world::map;
  using namespace modules::geometry;
  using namespace modules::world::goal_definition;

  // some line with three points from x=1 to x=10, y=0
  Line center;
  center.add_point(Point2d(1,1));
  center.add_point(Point2d(2,1));
  center.add_point(Point2d(10,1));

  Line outer;
  outer.add_point(Point2d(1,2));
  outer.add_point(Point2d(2,2));
  outer.add_point(Point2d(10,2));

  Line inner;
  inner.add_point(Point2d(1,0));
  inner.add_point(Point2d(2,0));
  inner.add_point(Point2d(10,0));

  DrivingCorridor corridor(outer, inner, center);

  LocalMap local_map(0, GoalDefinitionPtr(), corridor);

  local_map.ComputeHorizonCorridor(Point2d(10,1), 4);
}