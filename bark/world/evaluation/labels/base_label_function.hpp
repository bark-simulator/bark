// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_LABELS_BASE_LABEL_FUNCTION_HPP_
#define MODULES_WORLD_EVALUATION_LABELS_BASE_LABEL_FUNCTION_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ltl/label.h"
#include "ltl/rule_monitor.h"
#include "bark/world/objects/object.hpp"

namespace modules {
namespace world {

class ObservedWorld;

namespace evaluation {
using ltl::Label;
using LabelMap = ltl::EvaluationMap;

class BaseLabelFunction {
 public:
  explicit BaseLabelFunction(std::string label_str)
      : label_str_(std::move(label_str)) {}
  virtual std::vector<LabelMap::value_type> Evaluate(
      const world::ObservedWorld &observed_world) const = 0;
  const std::string &GetLabelStr() const { return label_str_; }
  Label GetLabel(const objects::AgentId agent_id) const {
    return Label(label_str_, agent_id);
  }
  Label GetLabel() const { return Label(label_str_); }

 private:
  std::string label_str_;
};
typedef std::shared_ptr<BaseLabelFunction> LabelFunctionPtr;
}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_LABELS_BASE_LABEL_FUNCTION_HPP_
