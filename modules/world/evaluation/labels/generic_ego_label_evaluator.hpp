// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_GENERIC_EGO_LABEL_EVALUATOR_HPP_
#define MODULES_WORLD_EVALUATION_GENERIC_EGO_LABEL_EVALUATOR_HPP_
#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/evaluation/labels/base_label_evaluator.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace world {
namespace evaluation {
template <class T>
class GenericEgoLabelEvaluator : public BaseLabelEvaluator {
 public:
  GenericEgoLabelEvaluator(const std::string& label_str, T evaluator)
      : BaseLabelEvaluator(label_str), evaluator_(evaluator) {}
  template <typename ... Args>
  GenericEgoLabelEvaluator(const std::string& label_str, Args&&... args)
      : BaseLabelEvaluator(label_str), evaluator_(std::forward<Args>(args)...) {}
  std::vector<LabelMap::value_type> Evaluate(const world::ObservedWorld& observed_world) const override {
    T temp_eval(evaluator_);
    auto ego_agent = observed_world.GetEgoAgent();
    bool res = false;
    if(ego_agent) {
      res = boost::get<bool>(
          temp_eval.Evaluate(observed_world));
    }
    return {{this->GetLabel(), res}};
  }
  T GetEvaluator() const { return evaluator_; }

 private:
  T evaluator_;
};
}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_GENERIC_EGO_LABEL_EVALUATOR_HPP_
