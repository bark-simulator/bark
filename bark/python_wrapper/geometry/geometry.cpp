// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "geometry.hpp"
#include <vector>
#include "bark/geometry/geometry.hpp"

namespace py = pybind11;

using bark::geometry::Collide;
using bark::geometry::Distance;
using bark::geometry::Line;
using bark::geometry::Norm0To2PI;
using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::geometry::Pose;
using bark::geometry::SignedDistance;
using bark::geometry::Within;
using Eigen::Dynamic;
using Eigen::Matrix;

void python_standard_shapes(py::module m) {
  m.def("CarLimousine", &bark::geometry::standard_shapes::CarLimousine);
  m.def("CarRectangle", &bark::geometry::standard_shapes::CarRectangle);
}

void python_geometry(py::module m) {
  py::class_<Point2d>(m, "Point2d")
      .def(py::init<float, float>())
      .def("__repr__",
           [](const Point2d& p) { return bark::geometry::print(p); })
      .def("x", [](Point2d& p) { return p.get<0>(); })
      .def("y", [](Point2d& p) { return p.get<1>(); })
      .def(py::pickle(
          [](const Point2d& p) {
            return py::make_tuple(p.get<0>(), p.get<1>());
          },
          [](py::tuple t) {
            if (t.size() != 2) throw std::runtime_error("Invalid point state!");
            return Point2d(t[0].cast<float>(), t[1].cast<float>());
          }));

  m.def("Distance",
        py::overload_cast<const Point2d&, const Point2d&>(&Distance),
        "Returns euclidean distance between two Point2d.");

  m.def("Distance", py::overload_cast<const Line&, const Point2d&>(&Distance),
        "Returns euclidean distance between Line2d and Point2d.");

  m.def("SignedDistance",
        py::overload_cast<const Line&, const Point2d&, const float&>(
            &SignedDistance),
        "Returns signed euclidean distance between Line2d and Point2d.");

  m.def("Distance", py::overload_cast<const Line&, const Line&>(&Distance),
        "Returns euclidean distance between Line2d and Point2d.");

  m.def("Distance",
        py::overload_cast<const Polygon&, const Polygon&>(&Distance),
        "Returns euclidean distance between polygon and polygon.");

  m.def("Distance", py::overload_cast<const Polygon&, const Line&>(&Distance),
        "Returns euclidean distance between polygon and line2d.");

  m.def("Distance",
        py::overload_cast<const Polygon&, const Point2d&>(&Distance),
        "Returns euclidean distance between polygon and point2d.");

  m.def("GetNearestPoint", &bark::geometry::GetNearestPoint,
        "get the nearest point from point to a line.");

  m.def("GetNearestS", &bark::geometry::GetNearestS,
        "get the nearest s value from point to a line.");

  m.def("GetPointAtS", &bark::geometry::GetPointAtS,
        "get the Point2d at position s of the line");

  m.def("GetTangentAngleAtS", &bark::geometry::GetTangentAngleAtS,
        "get the angle at position s of the line");

  m.def(
      "GetNearestPointAndS", &bark::geometry::GetNearestPointAndS,
      "get the point nearest to another point and its position on the line s ");

  m.def("GetLineFromSInterval", &bark::geometry::GetLineFromSInterval,
        "get line between specified interval.");

  m.def("MergeBoundingBoxes", &bark::geometry::MergeBoundingBoxes<Point2d>,
        "merge two bounding boxes consisting of pairs of min and max corners");

  m.def("ComputeCenterLine", &bark::geometry::ComputeCenterLine,
        "computes the center line.");

  m.def("Collide", py::overload_cast<const Polygon&, const Point2d&>(&Collide),
        "Returns true if polygon and point2d collide.");

  m.def("Collide", py::overload_cast<const Polygon&, const Line&>(&Collide),
        "Returns true if polygon and line collide.");

  m.def("Collide", py::overload_cast<const Polygon&, const Polygon&>(&Collide),
        "Returns true if polygon and polygon collide.");

  m.def("Within", py::overload_cast<const Point2d&, const Polygon&>(&Within),
        "Returns true if point within polygon");

  m.def("Within", py::overload_cast<const Line&, const Polygon&>(&Within),
        "Returns true if line within polygon");

  m.def("Within", py::overload_cast<const Polygon&, const Polygon&>(&Within),
        "Returns true if polygon within polygon.");

  m.def("Norm0To2PI", &bark::geometry::Norm0To2PI, "limit input to 0..2pi");

  py::class_<Line, std::shared_ptr<Line>>(m, "Line2d")
      .def(py::init<>(), "Create empty line")
      .def("AddPoint", &Line::AddPoint, "add a point")
      .def("AddPoint",
           [](Line& line, py::list list) {
             if (list.size() != 2) {
               printf("Error: List size of two required.");
               return;
             }
             line.AddPoint(
                 Point2d(list[0].cast<float>(), list[1].cast<float>()));
           })
      .def("__repr__",
           [](const Line& l) {
             std::stringstream ss;
             ss << "<bark.Line2d> Points: ";
             ss << l.ShapeToString();
             return ss.str();
           })
      .def("ToArray", &Line::ToArray, "returns numpy array.")
      .def("Valid", &Line::Valid, "checks if line is valid.")
      .def("Rotate", &Line::Rotate, "rotates object around center point.")
      .def("Translate", &Line::Translate, "translates object.")
      .def("Transform", &Line::Transform, "translates and rotates object.")
      .def("Length", &Line::Length, "calculates length of line.")
      .def("Reverse", &Line::Reverse, "reverse linestring in place")
      .def("AppendLinestring", &Line::AppendLinestring,
           "append linestrings in place")
      .def("concatenate_linestring", &Line::ConcatenateLinestring,
           "concatenate linestrings in place")
      .def_property_readonly("bounding_box", &Line::BoundingBox)
      .def_readwrite("center", &Line::center_, "center point.")
      .def(py::pickle(
          [](const Line& l) -> py::tuple {
            return py::make_tuple(l.ToArray());
          },
          [](const py::tuple& t) {
            if (t.size() != 1) throw std::runtime_error("Invalid line state!");
            Line l;
            auto points = t[0].cast<Matrix<float, Dynamic, Dynamic>>();
            for (int i = 0; i < points.rows(); ++i) {
              l.AddPoint(Point2d(points(i, 0), points(i, 1)));
            }
            return l;
          }));

  py::class_<Polygon, std::shared_ptr<Polygon>>(m, "Polygon2d")
      .def(py::init<>(), "Create empty polygon")
      .def(py::init<Pose, std::vector<Point2d>>(),
           "Create polygon with center point and point list")
      .def(py::init<Pose, const Matrix<float, Dynamic, Dynamic>&>(),
           "Create polygon with center point and point list")
      .def(py::init<Pose, const Line&>(),
           "Create polygon with center point and line enclosing polygon")
      .def("AddPoint", &Polygon::AddPoint, "add a point")
      .def("AddPoint",
           [](Polygon& polygon, py::list list) {
             if (list.size() != 2) {
               printf("Error: List size of two required.");
               return;
             }
             polygon.AddPoint(
                 Point2d(list[0].cast<float>(), list[1].cast<float>()));
           })
      .def("__repr__",
           [](const Polygon& p) {
             std::stringstream ss;
             ss << "<bark.Polygon2d> Points: ";
             ss << p.ShapeToString();
             return ss.str();
           })
      .def("ConcatenatePolygons", &Polygon::ConcatenatePolygons,
           "concatenates with another polygon")
      .def("BufferPolygon",
           [](const Polygon& p, const double distance) {
             Polygon poly_buffered;
             BufferPolygon(p, distance, &poly_buffered);
             return poly_buffered;
           })
      .def("CalculateArea", &Polygon::CalculateArea, "calculates are covered")
      .def("ToArray", &Polygon::ToArray, "returns numpy array")
      .def("Valid", &Polygon::Valid, "checks if polygong is valid.")
      .def("Rotate", &Polygon::Rotate, "rotates object around center point.")
      .def("Translate", &Polygon::Translate, "translates center point.")
      .def("Transform", &Polygon::Transform, "translates and rotates object.")
      .def_readonly("center", &Polygon::center_, "center point.")
      .def_readonly("right_dist", &Polygon::right_dist_, "right distance.")
      .def_readonly("left_dist", &Polygon::left_dist_, "left distance.")
      .def_readonly("front_dist", &Polygon::front_dist_, "front distance.")
      .def_readonly("rear_dist", &Polygon::rear_dist_, "rear distance.")
      .def_property_readonly("bounding_box", &Polygon::BoundingBox)
      .def(py::pickle(
          [](const Polygon& p) -> py::tuple {
            return py::make_tuple(p.ToArray(), p.center_);
          },
          [](py::tuple& t) {
            if (t.size() != 2) throw std::runtime_error("Invalid point state!");
            Polygon p(t[1].cast<Pose>(),
                      t[0].cast<Matrix<float, Dynamic, Dynamic>>());
            return p;
          }));

  python_standard_shapes(
      m.def_submodule("standard_shapes",
                      "Define several standard car, pedestrians,... shapes"));

  py::class_<bark::geometry::Model3D>(m, "Model3d")
      .def(py::init<>(), "Create none 3d model")
      .def(py::init<bark::geometry::Model3D::Type>(),
           "Create 3D model with specific type")
      .def_property_readonly("type", &bark::geometry::Model3D::GetType);

  py::enum_<bark::geometry::Model3D::Type>(m, "bark::geometry::Model3DType",
                                           py::arithmetic())
      .value("NONE", bark::geometry::Model3D::Type::NONE)
      .value("ROAD", bark::geometry::Model3D::Type::ROAD)
      .value("LIMOUSINE", bark::geometry::Model3D::Type::LIMOUSINE)
      .value("PEDESTRIAN", bark::geometry::Model3D::Type::PEDESTRIAN)
      .export_values();
}
