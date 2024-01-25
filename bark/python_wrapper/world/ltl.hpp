// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_PYTHON_WRAPPER_WORLD_LTL_HPP_
#define BARK_PYTHON_WRAPPER_WORLD_LTL_HPP_

#include "bark/python_wrapper/common.hpp"
#include "bark/world/evaluation/ltl/label_functions/base_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/safe_distance_label_function.hpp"
#include "bark/world/evaluation/ltl/evaluator_ltl.hpp"

namespace py = pybind11;
using namespace bark::world::evaluation;
using namespace bark::world;

class PyBaseLabelFunction : public BaseLabelFunction {
 public:
  using BaseLabelFunction::BaseLabelFunction;

  LabelMap Evaluate(const ObservedWorld& observed_world) const override {
    PYBIND11_OVERLOAD_PURE(LabelMap, BaseLabelFunction, Evaluate,
                           observed_world);
  }
};

class PySafeDistanceLabelFunction : public SafeDistanceLabelFunction { 
 public:
    /* Inherit the constructors */
    using SafeDistanceLabelFunction::SafeDistanceLabelFunction;

    using SafeDistanceLabelFunction::CalcVelFrontStar; 
    using SafeDistanceLabelFunction::CalcSafeDistance0; 
    using SafeDistanceLabelFunction::CalcSafeDistance1; 
    using SafeDistanceLabelFunction::CalcSafeDistance2; 
    using SafeDistanceLabelFunction::CalcSafeDistance3; 

    bool CheckSafeDistanceLateral(const float v_1_lat, const float v_2_lat, const float dist_lat,
                                  const double a_1_lat,  const double a_2_lat, const double delta1,
                                  const double delta2) const override {
        PYBIND11_OVERLOAD(
            bool,
            SafeDistanceLabelFunction,     
            CheckSafeDistanceLateral,       
            v_1_lat, v_2_lat, dist_lat, a_1_lat,  a_2_lat, delta1, delta2
        );
    }

    bool CheckSafeDistanceLongitudinal(const float v_f, const float v_r, const float dist,
                                       const double a_r,  const double a_f, const double delta) const override {
        PYBIND11_OVERLOAD(
            bool,
            SafeDistanceLabelFunction,     
            CheckSafeDistanceLongitudinal,       
            v_f, v_r, dist, a_r, a_f, delta
        );
    }

    LabelMap Evaluate(const ObservedWorld& observed_world) const override {
        PYBIND11_OVERLOAD(
            LabelMap,
            SafeDistanceLabelFunction,     
            Evaluate,       
            observed_world
        );
    }

};

class PyEvaluatorLTL : public EvaluatorLTL { 
 public:
    using EvaluatorLTL::Evaluate; 
    /* Inherit the constructors */
    using EvaluatorLTL::EvaluatorLTL;

    EvaluationReturn Evaluate(const ObservedWorld& observed_world) override {
        PYBIND11_OVERLOAD(
            EvaluationReturn,
            EvaluatorLTL,     
            Evaluate,       
            observed_world
        );
    }
};

void python_ltl(py::module m);

#endif  // BARK_PYTHON_WRAPPER_WORLD_LTL_HPP_
