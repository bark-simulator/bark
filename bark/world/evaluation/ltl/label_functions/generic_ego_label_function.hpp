// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_LTL_LABELS_GENERIC_EGO_LABEL_FUNCTION_HPP_
#define BARK_WORLD_EVALUATION_LTL_LABELS_GENERIC_EGO_LABEL_FUNCTION_HPP_

#include <string>
#include <utility>
#include <vector>

#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/evaluation/ltl/label_functions/base_label_function.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {
namespace evaluation {
template <class T>
class GenericEgoLabelFunction : public BaseLabelFunction {
 public:
  GenericEgoLabelFunction(const std::string& label_str, T evaluator)
      : BaseLabelFunction(label_str), evaluator_(evaluator) {}
  template <typename... Args>
  GenericEgoLabelFunction(const std::string& label_str, Args&&... args)
      : BaseLabelFunction(label_str), evaluator_(std::forward<Args>(args)...) {}
  LabelMap Evaluate(const world::ObservedWorld& observed_world) const override {
    T temp_eval(evaluator_);
    auto ego_agent = observed_world.GetEgoAgent();
    bool res = false;
    if (ego_agent) {
      res = boost::get<bool>(temp_eval.Evaluate(observed_world));
    }
    return {{this->GetLabel(), res}};
  }
  T GetEvaluator() const { return evaluator_; }

 private:
  T evaluator_;
};
}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_LTL_LABELS_GENERIC_EGO_LABEL_FUNCTION_HPP_
