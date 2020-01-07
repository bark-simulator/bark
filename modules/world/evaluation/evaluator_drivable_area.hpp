// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_EVALUATOR_DRIVABLE_AREA_
#define MODULES_WORLD_EVALUATION_EVALUATOR_DRIVABLE_AREA_

#include <vector>
#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/opendrive/commons.hpp"
#include "modules/world/map/roadgraph.hpp"
#include "modules/geometry/geometry.hpp"

namespace modules {
namespace world {
class World;
namespace evaluation {

using modules::world::map::RoadgraphPtr;
using modules::world::opendrive::LaneId;
using modules::world::map::PolygonPtr;
using modules::geometry::Polygon;
using modules::geometry::Point2d;


class EvaluatorDrivableArea : public BaseEvaluator {
 public:
  EvaluatorDrivableArea() {}
  virtual ~EvaluatorDrivableArea() {}

  virtual EvaluationReturn Evaluate(const world::World& world) {
    if (merged_polygons_.size() == 0) {
      RoadgraphPtr roadgraph = world.get_map()->get_roadgraph();
      std::vector<LaneId> lane_ids = roadgraph->get_all_laneids();
      std::vector<PolygonPtr> polygons;

      for (auto& lane_id : lane_ids) {
        PolygonPtr polygon = roadgraph->get_lane_polygon_by_id(lane_id);
        polygons.push_back(polygon);
      }

      for (unsigned i = 1; i < polygons.size(); i++) {
        boost::geometry::correct(polygons.at(i-1)->obj_);
        boost::geometry::correct(polygons.at(i)->obj_);
        boost::geometry::union_(
          polygons.at(i-1)->obj_,
          polygons.at(i)->obj_,
          merged_polygons_);
      }
    }

    for (auto agent : world.get_agents()) {
      Polygon poly_agent = agent.second->GetPolygonFromState(
        agent.second->get_current_state());
      // TODO(@hart): check if agent is in any of the merged polygons
      if (!boost::geometry::within(poly_agent.obj_, merged_polygons_.at(0)))
        return true;
    }
    return false;
  }

 private:
  std::vector<boost::geometry::model::polygon<Point2d>> merged_polygons_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_EVALUATOR_DRIVABLE_AREA_

