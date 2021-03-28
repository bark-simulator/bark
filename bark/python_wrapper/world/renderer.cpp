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
#include "bark/world/renderer/object_primitive.hpp"

namespace py = pybind11;
using bark::geometry::Point2d;
using bark::geometry::Line;
using bark::geometry::Polygon;
using bark::world::renderer::Renderer;
using bark::world::renderer::RenderPrimitive;
using bark::world::renderer::RenderPrimitivePtr;
using bark::world::renderer::ObjectRenderPrimitive;

void python_renderer(py::module m) {
  py::class_<Renderer, std::shared_ptr<Renderer>>(
    m, "Renderer")
    .def(py::init<>())
    .def("Add", &Renderer::Add)
    .def("Clear", &Renderer::Clear)
    .def_property_readonly(
      "render_primitives", &Renderer::GetRenderPrimitives);
  
  py::class_<RenderPrimitive, std::shared_ptr<RenderPrimitive>>(
    m, "RenderPrimitive")
    .def(py::init<std::string, std::string, std::string, std::string>())
    .def_property(
      "attr", &RenderPrimitive::GetAttr,  &RenderPrimitive::SetAttr);

  // py::class_<ObjectRenderPrimitive<Line>, RenderPrimitive, RenderPrimitivePtr>(
  //     m, "LineRenderPrimitive")
  //     .def(
  //       py::init<
  //         const Line&, std::string, std::string, std::string, std::string>());

  // py::class_<ObjectRenderPrimitive<Polygon>, RenderPrimitive, RenderPrimitivePtr>(
  //     m, "PolygonRenderPrimitive")
  //     .def(
  //       py::init<
  //         const Polygon&, std::string, std::string, std::string, std::string>());

  // py::class_<ObjectRenderPrimitive<Point2d>, RenderPrimitive, RenderPrimitivePtr>(
  //     m, "Point2dRenderPrimitive")
  //     .def(
  //       py::init<
  //         const Polygon&, std::string, std::string, std::string, std::string>());
}
