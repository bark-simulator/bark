// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_LTL_LABELS_BASE_LABEL_FUNCTION_HPP_
#define BARK_WORLD_EVALUATION_LTL_LABELS_BASE_LABEL_FUNCTION_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "bark/world/evaluation/ltl/label/label.h"
#include "bark/world/objects/object.hpp"

namespace bark {
namespace world {

class ObservedWorld;

namespace evaluation {
using LabelMap = EvaluationMap;

class BaseLabelFunction {
 public:
  explicit BaseLabelFunction(std::string label_str)
      : label_str_(std::move(label_str)) {}
  virtual LabelMap Evaluate(
      const world::ObservedWorld& observed_world) const = 0;
  const std::string& GetLabelStr() const { return label_str_; }
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
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_LTL_LABELS_BASE_LABEL_FUNCTION_HPP_
