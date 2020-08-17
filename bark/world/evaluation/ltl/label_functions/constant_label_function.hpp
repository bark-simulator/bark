// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_TESTS_CONSTANT_LABEL_EVALUATOR_HPP_
#define BARK_WORLD_TESTS_CONSTANT_LABEL_EVALUATOR_HPP_

#include <string>
#include <vector>

#include "bark/world/evaluation/ltl/label_functions/base_label_function.hpp"

namespace bark {
namespace world {
namespace evaluation {

class ConstantLabelFunction : public BaseLabelFunction {
 public:
  ConstantLabelFunction(const std::string& label_str)
      : BaseLabelFunction(label_str), value_(true) {}
  LabelMap Evaluate(const world::ObservedWorld& observed_world) const override {
    return {{Label(GetLabelStr()), value_}};
  }
  bool GetValue() const { return value_; }
  void SetValue(bool value) { value_ = value; }

 private:
  bool value_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_TESTS_CONSTANT_LABEL_EVALUATOR_HPP_
