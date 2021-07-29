// Copyright (c) 2021 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include <string>
#include "bark/python_wrapper/world/renderer.hpp"
#include "bark/geometry/geometry.hpp"
#include "bark/world/renderer/renderer.hpp"
#include "bark/world/renderer/primitive.hpp"

namespace py = pybind11;
using bark::geometry::Point2d;
using bark::geometry::Line;
using bark::geometry::Polygon;
using bark::world::renderer::Renderer;
using bark::world::renderer::RenderPrimitive;
using bark::world::renderer::RenderPrimitivePtr;
using bark::world::renderer::HolderType;


void python_renderer(py::module m) {
  py::class_<Renderer, std::shared_ptr<Renderer>>(
    m, "Renderer")
    .def(py::init<>())
    .def("Add", &Renderer::Add)
    .def("Clear", &Renderer::Clear)
    .def("AddRendererChild", &Renderer::AddRendererChild)
    .def("GetRendererChild", &Renderer::GetRendererChild)
    .def("GetRendererChildren", &Renderer::GetRendererChildren)
    .def("ClearRendererChildren", &Renderer::ClearRendererChildren)
    .def_property_readonly(
      "primitives", &Renderer::GetRenderPrimitives);
  
  py::class_<RenderPrimitive, std::shared_ptr<RenderPrimitive>>(
    m, "RenderPrimitive")
    .def(py::init<
      const HolderType&>())
    .def_readwrite("conf", &RenderPrimitive::conf)
    .def("Add", &RenderPrimitive::SetAttr)
    .def_property_readonly(
      "object", &RenderPrimitive::GetObject);
}
