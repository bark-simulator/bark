// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_PYTHON_BINDINGS_COMMON_HPP_
#define PYTHON_PYTHON_BINDINGS_COMMON_HPP_

#include <chrono>
#include <sstream>

#include "boost/variant.hpp"
#include "pybind11/complex.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "pybind11/stl_bind.h"

PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

// `boost::variant` as an example -- can be any `std::variant`-like container
namespace pybind11 {
namespace detail {

template <typename... Ts>
struct type_caster<boost::variant<Ts...>>
    : variant_caster<boost::variant<Ts...>> {};

// Specifies the function used to visit the variant
// -- `apply_visitor` instead of `visit`
template <>
struct visit_helper<boost::variant> {
  template <typename... Args>
  static auto call(Args&&... args) -> decltype(boost::apply_visitor(args...)) {
    return boost::apply_visitor(args...);
  }
};

}  // namespace detail
}  // namespace pybind11

#endif  // PYTHON_PYTHON_BINDINGS_COMMON_HPP_
