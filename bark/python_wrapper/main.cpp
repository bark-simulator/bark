// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "commons/commons.hpp"
#include "geometry/geometry.hpp"
#include "models/models.hpp"
#include "runtime.hpp"
#include "viewer/viewer.hpp"
#include "world/world.hpp"
#ifdef LTL_RULES
#include "python/bindings/define_rule_monitor.hpp"
#endif

namespace py = pybind11;

PYBIND11_MODULE(core, m) {
  m.doc() = "Wrapper for BARK library.";
  python_geometry(m.def_submodule("geometry", "submodule handling geometries"));
  python_commons(m.def_submodule(
      "commons", "submodule containing all wrapped parameters objects"));
  python_models(m.def_submodule("models",
                                "submodule containing models such as the "
                                "behavior, dynamic and execution"));
  python_world(m.def_submodule(
      "world", "submodule containing all wrapped parameters objects"));
  python_viewer(m.def_submodule("viewer", "submodule containing the viewer"));
  python_runtime(
      m.def_submodule("runtime", "submodule containing the runtime"));
#ifdef LTL_RULES
  define_rule_monitor(
      m.def_submodule("ltl", "submodule containing the rule monitor"));
#endif
}
