// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_EVALUATOR_DRIVABLE_AREA_
#define MODULES_WORLD_EVALUATION_EVALUATOR_DRIVABLE_AREA_

#include <vector>
#include "modules/geometry/geometry.hpp"
#include "modules/world/evaluation/base_evaluator.hpp"

namespace modules {
namespace world {
class World;
namespace evaluation {

class EvaluatorDrivableArea : public BaseEvaluator {
 public:
  EvaluatorDrivableArea() {}
  virtual ~EvaluatorDrivableArea() {}

  virtual EvaluationReturn Evaluate(const world::World& world) {
    using modules::geometry::Polygon;
    namespace bg = boost::geometry;

    for (const auto& agent : world.GetAgents()) {
      Polygon poly_agent =
          agent.second->GetPolygonFromState(agent.second->GetCurrentState());
      auto poly_road = agent.second->GetRoadCorridor()->GetPolygon();
      if (!bg::within(poly_agent.obj_, poly_road.obj_)) {
        return true;
      }
    }
    return false;
  }
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_EVALUATOR_DRIVABLE_AREA_
