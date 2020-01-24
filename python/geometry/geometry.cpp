// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <vector>
#include "geometry.hpp"
#include "modules/geometry/geometry.hpp"

namespace py = pybind11;

// using namespace modules;
// using namespace modules::commons;
// using namespace modules::geometry;

void python_standard_shapes(py::module m) {
  m.def("CarLimousine", &modules::geometry::standard_shapes::CarLimousine);
}

void python_geometry(py::module m) {
  py::class_<modules::geometry::Point2d>(m, "Point2d")
      .def(py::init<float, float>())
      .def("__repr__", [](const modules::geometry::Point2d &p) {
        return modules::geometry::print(p);
      })
      .def("x", [](modules::geometry::Point2d &p) {
        return p.get<0>();
      })
      .def("y", [](modules::geometry::Point2d &p) {
        return p.get<1>();
      })
      .def(py::pickle(
        [](const modules::geometry::Point2d& p) { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(p.get<0>(), p.get<1>());
        },
        [](py::tuple t)  { // __setstate__
            if (t.size() != 2)
                throw std::runtime_error("Invalid point state!");

            return modules::geometry::Point2d(t[0].cast<float>(), t[1].cast<float>());
        }));

  m.def("Distance", py::overload_cast<const modules::geometry::Point2d &, const modules::geometry::Point2d &>(&modules::geometry::Distance),
        "Returns euclidean distance between two modules::geometry::Point2d.");

  m.def("Distance", py::overload_cast<const modules::geometry::Line &, const modules::geometry::Point2d &>(&modules::geometry::Distance),
        "Returns euclidean distance between modules::geometry::Line2d and modules::geometry::Point2d.");

  m.def("Distance", py::overload_cast<const modules::geometry::Line &, const modules::geometry::Line &>(&modules::geometry::Distance),
        "Returns euclidean distance between modules::geometry::Line2d and modules::geometry::Point2d.");
        
  m.def("Distance", py::overload_cast<const modules::geometry::Polygon &, const modules::geometry::Polygon &>(&modules::geometry::Distance),
        "Returns euclidean distance between polygon and polygon.");

  m.def("Distance", py::overload_cast<const modules::geometry::Polygon &, const modules::geometry::Line &>(&modules::geometry::Distance),
        "Returns euclidean distance between polygon and line2d.");

  m.def("Distance", py::overload_cast<const modules::geometry::Polygon &, const modules::geometry::Point2d &>(&modules::geometry::Distance),
        "Returns euclidean distance between polygon and point2d.");

  m.def("GetNearestPoint", &modules::geometry::GetNearestPoint, "get the nearest point from point to a line.");

  m.def("GetNearestS", &modules::geometry::GetNearestS, "get the nearest s value from point to a line.");
  
  m.def("GetPointAtS", &modules::geometry::GetPointAtS,
                                 "get the Point2d at position s of the line");

  m.def("GetTangentAngleAtS", &modules::geometry::GetTangentAngleAtS,
                                   "get the angle at position s of the line");
  
  m.def("GetNearestPointAndS", &modules::geometry::GetNearestPointAndS, 
                        "get the point nearest to another point and its position on the line s ");

  m.def("GetLineFromSInterval", &modules::geometry::GetLineFromSInterval, 
                        "get line between specified interval.");

  m.def("MergeBoundingBoxes", &modules::geometry::MergeBoundingBoxes<modules::geometry::Point2d>, 
                        "merge two bounding boxes consisting of pairs of min and max corners");

  m.def("compute_center_line", &modules::geometry::ComputeCenterLine, 
                        "computes the center line.");

  py::class_<modules::geometry::Line,
             std::shared_ptr<modules::geometry::Line>>(m, "Line2d")
      .def(py::init<>(), "Create empty line")
      .def("addPoint", &modules::geometry::Line::AddPoint, "add a point")
      .def("addPoint", [](modules::geometry::Line &line, py::list list) {
        if (list.size() != 2) {
          printf("Error: List size of two required.");
          return;
        }
        line.AddPoint(modules::geometry::Point2d(list[0].cast<float>(), list[1].cast<float>()));
      })
      .def("__repr__", [](const modules::geometry::Line &l) {
        std::stringstream ss;
        ss << "<bark.Line2d> Points: ";
        ss << l.ShapeToString();
        return ss.str();
      })
      .def("ToArray", &modules::geometry::Line::ToArray, "returns numpy array.")
      .def("valid", &modules::geometry::Line::Valid, "checks if line is valid.")
      .def("Rotate", &modules::geometry::Line::Rotate, "rotates object around center point.")
      .def("translate", &modules::geometry::Line::translate, "translates object.")
      .def("transform", &modules::geometry::Line::transform, "translates and rotates object.")
      .def("Length", &modules::geometry::Line::Length, "calculates length of line.")
      .def("reverse", &modules::geometry::Line::Reverse, "reverse linestring in place")
      .def("AppendLinestring", &modules::geometry::Line::AppendLinestring, "append linestrings in place")
      .def("concatenate_linestring", &modules::geometry::Line::ConcatenateLinestring, "concatenate linestrings in place")
      .def_property_readonly("bounding_box", &modules::geometry::Line::BoundingBox)
      .def_readwrite("center", &modules::geometry::Line::center_, "center point.")
      .def(py::pickle(
        [](const modules::geometry::Line& l) -> py::tuple { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(l.ToArray());
        },
        [](const py::tuple& t)  { // __setstate__
            if (t.size() != 1)
                throw std::runtime_error("Invalid line state!");

            modules::geometry::Line l;
            auto points = t[0].cast<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>();
            for(int i = 0; i < points.rows(); ++i) {
              l.AddPoint(modules::geometry::Point2d(points(i,0), points(i,1)));
            }
            return l;
        }));

  py::class_<modules::geometry::Polygon,
            std::shared_ptr<modules::geometry::Polygon>>(m, "Polygon2d")
      .def(py::init<>(), "Create empty polygon")
      .def(py::init<modules::geometry::Pose, std::vector<modules::geometry::Point2d>>(),
                       "Create polygon with center point and point list")
      .def(py::init<modules::geometry::Pose, const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &>(),
                       "Create polygon with center point and point list")
      .def(py::init<modules::geometry::Pose, const modules::geometry::Line&>(),
                       "Create polygon with center point and line enclosing polygon")
      .def("addPoint", &modules::geometry::Polygon::AddPoint, "add a point")
      .def("addPoint", [](modules::geometry::Polygon &polygon, py::list list) {
        if (list.size() != 2) {
          printf("Error: List size of two required.");
          return;
        }
        polygon.AddPoint(modules::geometry::Point2d(list[0].cast<float>(), list[1].cast<float>()));
      })
      .def("__repr__", [](const modules::geometry::Polygon &p) {
        std::stringstream ss;
        ss << "<bark.Polygon2d> Points: ";
        ss << p.ShapeToString();
        return ss.str();
      })
      .def("ToArray", &modules::geometry::Polygon::ToArray, "returns numpy array")
      .def("valid", &modules::geometry::Polygon::Valid, "checks if polygong is valid.")
      .def("Rotate", &modules::geometry::Polygon::Rotate, "rotates object around center point.")
      .def("translate", &modules::geometry::Polygon::translate, "translates center point.")
      .def("transform", &modules::geometry::Polygon::transform, "translates and rotates object.")
      .def_readonly("center", &modules::geometry::Polygon::center_, "center point.")
      .def_readonly("right_dist", &modules::geometry::Polygon::right_dist_, "center point.")
      .def_readonly("left_dist", &modules::geometry::Polygon::left_dist_, "center point.")
      .def_readonly("front_dist", &modules::geometry::Polygon::front_dist_, "center point.")
      .def_readonly("rear_dist", &modules::geometry::Polygon::rear_dist_, "center point.")
      .def_property_readonly("bounding_box", &modules::geometry::Polygon::BoundingBox)
      .def(py::pickle(
        [](const modules::geometry::Polygon& p) -> py::tuple { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(p.ToArray(), p.center_);
        },
        [](py::tuple  &t)  { // __setstate__
            if (t.size() != 2)
                throw std::runtime_error("Invalid point state!");

            modules::geometry::Polygon p;
            auto points = t[0].cast<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>();
            for(int i = 0; i < points.rows(); ++i) {
              p.AddPoint(modules::geometry::Point2d(points(i,0), points(i,1)));
            }
            p.center_ = t[1].cast<modules::geometry::Pose>();
            return p;
        }));

  python_standard_shapes(m.def_submodule("standard_shapes", "Define several standard car, pedestrians,... shapes"));

  py::class_<modules::geometry::Model3D>(m, "Model3d")
      .def(py::init<>(), "Create none 3d model")
      .def(py::init<modules::geometry::Model3D::Type>(), "Create 3D model with specific type ")
      .def_property_readonly("type",&modules::geometry::Model3D::GetType);

  py::enum_<modules::geometry::Model3D::Type>(m, "modules::geometry::Model3DType", py::arithmetic())
      .value("NONE", modules::geometry::Model3D::Type::NONE)
      .value("ROAD", modules::geometry::Model3D::Type::ROAD)
      .value("LIMOUSINE", modules::geometry::Model3D::Type::LIMOUSINE)
      .value("PEDESTRIAN", modules::geometry::Model3D::Type::PEDESTRIAN)
      .export_values();
}
