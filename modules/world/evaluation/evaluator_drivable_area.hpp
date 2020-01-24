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
using modules::world::opendrive::XodrLaneId;
using modules::world::map::PolygonPtr;
using modules::geometry::Polygon;
using modules::geometry::Point2d;


class EvaluatorDrivableArea : public BaseEvaluator {
 public:
  EvaluatorDrivableArea() {}
  virtual ~EvaluatorDrivableArea() {}

  virtual EvaluationReturn Evaluate(const world::World& world) {
    for (const auto& agent : world.GetAgents()) {
      Polygon poly_agent = agent.second->GetPolygonFromState(
        agent.second->GetCurrentState());
      if (!boost::geometry::within(poly_agent.obj_,
          agent.second->GetRoadCorridor()->GetPolygon().obj_))
        return true;
    }
    return false;
  }
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_EVALUATOR_DRIVABLE_AREA_

