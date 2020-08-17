// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/python_wrapper/models/plan/plan.hpp"
#ifdef PLANNER_UCT
#include "bark_mcts/python_wrapper/python_planner_uct.hpp"
#endif

void python_behavior_plan(py::module m) {
#ifdef PLANNER_UCT
  python_planner_uct(m);
#endif
}