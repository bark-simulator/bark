// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_TESTS_CONSTANT_LABEL_EVALUATOR_HPP_
#define MODULES_WORLD_TESTS_CONSTANT_LABEL_EVALUATOR_HPP_

#include <string>
#include <vector>

#include "modules/world/evaluation/labels/base_label_function.hpp"

namespace modules {
namespace world {
namespace evaluation {

class ConstantLabelFunction : public BaseLabelFunction {
 public:
  ConstantLabelFunction(const std::string& label_str)
      : BaseLabelFunction(label_str), value_(true) {}
  std::vector<LabelMap::value_type> Evaluate(
      const world::ObservedWorld& observed_world) const override {
    return {{ltl::Label(GetLabelStr()), value_}};
  }
  bool GetValue() const { return value_; }
  void SetValue(bool value) { value_ = value; }
 private:
  bool value_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_TESTS_CONSTANT_LABEL_EVALUATOR_HPP_
