//
// Created by luis on 07.10.19.
//

#ifndef BARK_MODULES_WORLD_EVALUATION_BASE_LABEL_EVALUATOR_HPP_
#define BARK_MODULES_WORLD_EVALUATION_BASE_LABEL_EVALUATOR_HPP_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ltl/label.h"
#include "ltl/rule_monitor.h"
#include "modules/world/objects/object.hpp"

namespace modules {
namespace world {

class ObservedWorld;

namespace evaluation {
using ltl::Label;
using LabelMap = ltl::EvaluationMap;

class BaseLabelEvaluator {
 public:
  BaseLabelEvaluator(std::string label_str) : label_str_(std::move(label_str)) {}
  virtual std::vector<LabelMap::value_type> Evaluate(const world::ObservedWorld &observed_world) const = 0;
  const std::string &GetLabelStr() const { return label_str_; }
  Label GetLabel(const objects::AgentId agent_id) const {return Label(label_str_, agent_id);}
  Label GetLabel() const { return Label(label_str_); }
 private:
  std::string label_str_;
};
typedef std::shared_ptr<BaseLabelEvaluator> LabelEvaluatorPtr;
}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // BARK_MODULES_WORLD_EVALUATION_BASE_LABEL_EVALUATOR_HPP_
