// Copyright (c) 2020 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/commons/params/setter_params.hpp"


namespace modules {
namespace commons {

SetterParams::SetterParams(bool log_if_default, const CondensedParamList& param_list) {
  for(const auto& param_pair : param_list) {
    const auto& param_name = param_pair.first;
    const auto& param_variant = param_pair.second;
    // TODO(@all): how to disable that message from python to not spam the console
    // LOG(INFO) << "Deserializing param " << param_name;
    boost::apply_visitor(ParamVisitor(this, param_name), param_variant);
  }
}



} // commons
} // modules