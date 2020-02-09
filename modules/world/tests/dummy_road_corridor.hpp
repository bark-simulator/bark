// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_TESTS_DUMMY_ROAD_CORRIDOR_HPP_
#define MODULES_WORLD_TESTS_DUMMY_ROAD_CORRIDOR_HPP_

#include "modules/geometry/commons.hpp"
#include "modules/world/map/map_interface.hpp"

namespace modules {
namespace world {
namespace tests {

using namespace modules::geometry;
using modules::geometry::Line;
using modules::geometry::Polygon;
using modules::world::map::LaneCorridor;
using modules::world::map::LaneCorridorPtr;

class DummyRoadCorridor : public modules::world::map::RoadCorridor {
 public:
  DummyRoadCorridor(const Line& driving_corridor_center,
                    const Line& driving_corridor_left_boundary,
                    const Line& driving_corridor_right_boundary)
      : corridor_ptr_() {
    Line polygon_line = driving_corridor_left_boundary;
    Line temp = driving_corridor_right_boundary;
    temp.Reverse();
    polygon_line.AppendLinestring(temp);
    const auto corridor_polygon = Polygon(Pose(), polygon_line);

    LaneCorridor lane_corridor;
    lane_corridor.SetLeftBoundary(driving_corridor_left_boundary);
    lane_corridor.SetRightBoundary(driving_corridor_right_boundary);
    lane_corridor.SetCenterLine(driving_corridor_center);
    lane_corridor.SetMergedPolygon(corridor_polygon);
    corridor_ptr_ = std::make_shared<LaneCorridor>(lane_corridor);
  }
  virtual ~DummyRoadCorridor() {}

  virtual LaneCorridorPtr GetCurrentLaneCorridor(const Point2d& pt) const {
    return corridor_ptr_;
  }

 private:
  LaneCorridorPtr corridor_ptr_;
};

}  // namespace tests
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_TESTS_DUMMY_ROAD_CORRIDOR_HPP_