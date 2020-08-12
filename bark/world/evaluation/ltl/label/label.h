// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_LTL_LABEL_LABEL_HPP_
#define BARK_WORLD_EVALUATION_LTL_LABEL_LABEL_HPP_

#include <ostream>
#include <string>
#include <unordered_map>

namespace bark {
namespace world {
namespace evaluation {

class Label {
 public:
  Label() {}
  Label(const std::string& label_str, int agent_id)
      : label_str_(label_str), agent_id_(agent_id), is_agent_specific_(true) {}

  Label(const std::string& label_str)
      : label_str_(label_str), agent_id_(-1), is_agent_specific_(false) {}

  static Label MakeAlive() { return Label("alive"); }

  const std::string& GetLabelStr() const { return label_str_; }

  int GetAgentId() const { return agent_id_; }
  bool IsAgentSpecific() const { return is_agent_specific_; }

  bool operator==(const Label& rhs) const {
    bool equal = label_str_ == rhs.label_str_;
    equal &= is_agent_specific_ == rhs.is_agent_specific_;
    if (is_agent_specific_) {
      equal &= agent_id_ == rhs.agent_id_;
    }
    return equal;
  }
  bool operator!=(const Label& rhs) const { return !(rhs == *this); }

  friend std::ostream& operator<<(std::ostream& os, const Label& label) {
    os << "label_str_: " << label.label_str_
       << " agent_id_: " << label.agent_id_
       << " is_agent_specific_: " << label.is_agent_specific_;
    return os;
  }

 private:
  std::string label_str_;
  int agent_id_;
  bool is_agent_specific_;
};

class LabelHash {
 public:
  size_t operator()(const Label& label) const {
    return std::hash<std::string>()(label.GetLabelStr());
  }
};

typedef std::unordered_map<Label, bool, LabelHash> EvaluationMap;

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_LTL_LABEL_LABEL_HPP_
