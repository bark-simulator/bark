// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "commons/commons.hpp"
#include "geometry/geometry.hpp"
#include "models/models.hpp"
#include "world/world.hpp"
#include "viewer/viewer.hpp" 
#include "runtime.hpp" 

namespace py = pybind11;

PYBIND11_MODULE(bark, m) {
  m.doc() = "Wrapper for fortiss-behave library";
  python_geometry(m.def_submodule("geometry", "submodule handling geometries"));
  python_commons(m.def_submodule("commons", "submodule containing all wrapped parameters objects"));
  python_models(m.def_submodule("models", "submodule containing models such as the behavior, dynamic and execution"));
  python_world(m.def_submodule("world", "submodule containing all wrapped parameters objects"));
  python_viewer(m.def_submodule("viewer", "submodule containing the viewer"));
  python_runtime(m.def_submodule("runtime", "submodule containing the runtime"));
}