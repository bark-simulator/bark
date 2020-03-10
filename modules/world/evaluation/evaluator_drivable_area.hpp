// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_EVALUATOR_DRIVABLE_AREA_HPP_
#define MODULES_WORLD_EVALUATION_EVALUATOR_DRIVABLE_AREA_HPP_

#include <limits>
#include <vector>
#include "modules/geometry/geometry.hpp"
#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {
class World;
class ObservedWorld;
namespace evaluation {

class EvaluatorDrivableArea : public BaseEvaluator {
 public:
  EvaluatorDrivableArea() :
    agent_id_(std::numeric_limits<AgentId>::max()) {}
  explicit EvaluatorDrivableArea(const AgentId& agent_id) :
    agent_id_(agent_id) {}
  virtual ~EvaluatorDrivableArea() {}

  virtual EvaluationReturn Evaluate(const world::World& world) {
    using modules::geometry::Polygon;
    namespace bg = boost::geometry;

    if (agent_id_ != std::numeric_limits<AgentId>::max()) {
      const auto& agent = world.GetAgent(agent_id_);
      if(!agent) {
        return true;
      }
      Polygon poly_agent =
        agent->GetPolygonFromState(agent->GetCurrentState());
      const auto& poly_road = agent->GetRoadCorridor()->GetPolygon();
      if (!bg::within(poly_agent.obj_, poly_road.obj_)) {
        return true;
      }
      return false;
    }

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

  virtual EvaluationReturn Evaluate(
    const world::ObservedWorld& observed_world) {
      using modules::geometry::Polygon;
      namespace bg = boost::geometry;

      const auto& agent = observed_world.GetEgoAgent();
      Polygon poly_agent =
        agent->GetPolygonFromState(agent->GetCurrentState());
      const auto& poly_road = agent->GetRoadCorridor()->GetPolygon();
      if (!bg::within(poly_agent.obj_, poly_road.obj_)) {
        return true;
      }
      return false;
  }

 private:
  AgentId agent_id_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_EVALUATOR_DRIVABLE_AREA_HPP_
