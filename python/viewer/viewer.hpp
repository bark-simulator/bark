// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_PYTHON_BINDINGS_VIEWER_VIEWER_HPP_
#define PYTHON_PYTHON_BINDINGS_VIEWER_VIEWER_HPP_

#include "python/common.hpp"
#include "modules/commons/params/params.hpp"
#include "modules/runtime/viewer/viewer.hpp"

namespace py = pybind11;

using namespace modules::commons;
using namespace modules::viewer;

class PyViewer : public Viewer {
 public:
  using Viewer::Viewer;
  void drawPoint2d(const Point2d &point2d, const Color& color, const float& alpha) override {
    PYBIND11_OVERLOAD_PURE(
            void,
            Viewer,
            drawPoint2d,
            point2d,
            color,
            alpha);
  }

  void drawLine2d(const Line &line, const Color& color, const float& alpha) override {
    PYBIND11_OVERLOAD_PURE(
            void,
            Viewer,
            drawLine2d,
            line,
            color,
            alpha);
  }

  void drawPolygon2d(const Polygon& polygon, const Color& color, const float& alpha) override {
    PYBIND11_OVERLOAD_PURE(
            void,
            Viewer,
            drawPolygon2d,
            polygon,
            color,
            alpha);
  }
};

void python_viewer(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_VIEWER_VIEWER_HPP_

