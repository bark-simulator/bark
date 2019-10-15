// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "python/models/plan/plan.hpp"
#ifdef PLANNER_UCT
  #include "python/python_planner_uct.hpp"
#endif

void python_behavior_plan(py::module m) {
#ifdef PLANNER_UCT
  python_planner_uct(m);
#endif

}