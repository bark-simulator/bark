// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_BASE_EVALUATOR_HPP_
#define BARK_WORLD_EVALUATION_BASE_EVALUATOR_HPP_

#include <boost/variant.hpp>
#include <memory>
#include "bark/commons/commons.hpp"

namespace bark {
namespace world {
class World;
class ObservedWorld;
namespace evaluation {

typedef boost::variant<float, bool, std::string, int> EvaluationReturn;

class BaseEvaluator {
 public:
  BaseEvaluator() {}
  virtual ~BaseEvaluator() {}
  virtual EvaluationReturn Evaluate(const world::World& world) { return false; }
  virtual EvaluationReturn Evaluate(
      const world::ObservedWorld& observed_world) {
    return false;
  }
};
typedef std::shared_ptr<BaseEvaluator> EvaluatorPtr;

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_BASE_EVALUATOR_HPP_
