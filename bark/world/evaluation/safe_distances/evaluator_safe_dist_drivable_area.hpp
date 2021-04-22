// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_EVALUATOR_SAFE_DIST_DRIVABLE_AREA_HPP_
#define BARK_WORLD_EVALUATION_EVALUATOR_SAFE_DIST_DRIVABLE_AREA_HPP_

#include <limits>
#include <vector>
#include "bark/geometry/geometry.hpp"
#include "bark/world/evaluation/evaluator_drivable_area.hpp"
#include "bark/world/evaluation/safe_distances/evaluator_safe_dist_base.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {
class World;
class ObservedWorld;
namespace evaluation {

using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::geometry::Pose;
using bark::models::dynamic::State;
using bark::models::dynamic::StateDefinition;

class EvaluatorSafeDistDrivableArea : public EvaluatorDrivableArea, public EvaluatorSafeDistBase {
 public:
  explicit EvaluatorSafeDistDrivableArea(const bark::commons::ParamsPtr& params, const AgentId& agent_id);
  virtual ~EvaluatorSafeDistDrivableArea() {}

  EvaluationReturn Evaluate(const world::World& world) override;
  EvaluationReturn Evaluate(const world::ObservedWorld& observed_world) override;

  bool CheckSafeDistance(const world::ObservedWorld& observed_world) override;

 private:
  float lateral_safety_dist_;
  float longitudinal_safety_dist_;

 private:
  Polygon GetCollisionShape(const AgentPtr& checked_agent) const override;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_EVALUATOR_SAFE_DIST_DRIVABLE_AREA_HPP_
