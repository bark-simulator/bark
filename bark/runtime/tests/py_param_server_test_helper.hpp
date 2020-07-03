// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_RUNTIME_TESTS_PARAM_SERVER_TEST_HELPER_HPP_
#define BARK_RUNTIME_TESTS_PARAM_SERVER_TEST_HELPER_HPP_

#include "bark/commons/commons.hpp"

class CppParamServerTestObject {
 public:
  CppParamServerTestObject(std::shared_ptr<bark::commons::Params> params)
      : params_(params) {
    value_float_ =
        params_->GetReal("Child1::Child2::ValueFloat", "Desc", 23434.0);
    value_bool_false_ =
        params_->GetBool("Child1::Child2::ValueBoolFalse", "Desc", true);
    value_bool_true_ =
        params_->GetBool("Child3::Child2::ValueBoolTrue", "Desc", false);
    value_int_ = params_->GetInt("Child1::Child4::ValueInt", "Desc", 234);
    value_list_list_float_ =
        params_->GetListListFloat("Child1::Child4::ValueListListFloat", "Desc",
                                  {{1.0, 3.4545234}, {1.1266135, 2.0, 3434.4}});
    value_list_float_ =
        params_->GetListFloat("Child1::Child5::ValueListFloat", "Desc",
                              {1.0, 3.4545234, 1.1266135, 2.0, 3434.4});
  }

  float GetRealValue() const { return value_float_; }
  bool GetBoolValueTrue() const { return value_bool_true_; }
  bool GetBoolValueFalse() const { return value_bool_false_; }
  float GetIntValue() const { return value_int_; }
  bark::commons::ListListFloat GetListListFloatValue() const {
    return value_list_list_float_;
  }
  bark::commons::ListFloat GetListFloatValue() const {
    return value_list_float_;
  }

  std::shared_ptr<bark::commons::Params> GetParams() const { return params_; }

 private:
  float value_float_;
  bool value_bool_false_;
  bool value_bool_true_;
  int value_int_;
  bark::commons::ListListFloat value_list_list_float_;
  bark::commons::ListFloat value_list_float_;

  std::shared_ptr<bark::commons::Params> params_;
};

#endif  // BARK_RUNTIME_TESTS_PARAM_SERVER_TEST_HELPER_HPP_