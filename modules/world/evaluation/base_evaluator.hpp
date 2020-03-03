// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_BASE_EVALUATOR_HPP_
#define MODULES_WORLD_EVALUATION_BASE_EVALUATOR_HPP_

#include <memory>
#include <boost/variant.hpp>
#include "modules/commons/base_type.hpp"

namespace modules {
namespace world {
class World;
class ObservedWorld;
namespace evaluation {

typedef boost::variant<float, bool, std::string, int> EvaluationReturn;

class BaseEvaluator {
 public:
  BaseEvaluator() { }
  virtual ~BaseEvaluator() { }
  virtual EvaluationReturn Evaluate(const world::World& world) { return false; }
  virtual EvaluationReturn Evaluate(
    const world::ObservedWorld& observed_world) { return false; }
};
typedef std::shared_ptr<BaseEvaluator> EvaluatorPtr;

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif // MODULES_WORLD_EVALUATION_BASE_EVALUATOR_HPP_
