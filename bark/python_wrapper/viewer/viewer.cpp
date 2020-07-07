// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias
// Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "viewer.hpp"
#include "bark/runtime/viewer/viewer.hpp"
#include "bark/runtime/viewer/viewer_functions.hpp"

namespace py = pybind11;
using namespace bark::viewer;

void python_viewer(py::module m) {
  auto view = py::class_<Viewer, PyViewer, std::shared_ptr<Viewer>>(m, "Viewer")
                  .def(py::init<>())
                  .def("drawPoint2d", &Viewer::drawPoint2d)
                  .def("drawLine2d", &Viewer::drawLine2d);

  py::enum_<Viewer::Color>(view, "Color")
      .value("White", Viewer::Color_White)
      .value("LightGray", Viewer::Color_LightGray)
      .value("DarkGray", Viewer::Color_DarkGray)
      .value("Gray", Viewer::Color_Gray)
      .value("Black", Viewer::Color_Black)
      .value("Pink", Viewer::Color_Pink)
      .value("Red", Viewer::Color_Red)
      .value("DarkRed", Viewer::Color_DarkRed)
      .value("Lime", Viewer::Color_Lime)
      .value("LightGreen", Viewer::Color_LightGreen)
      .value("Green", Viewer::Color_Green)
      .value("LightBlue", Viewer::Color_LightBlue)
      .value("Blue", Viewer::Color_Blue)
      .value("DarkBlue", Viewer::Color_DarkBlue)
      .value("Cyan", Viewer::Color_Cyan)
      .value("Aquamarine", Viewer::Color_Aquamarine)
      .value("Teal", Viewer::Color_Teal)
      .value("Violet", Viewer::Color_Violet)
      .value("Magenta", Viewer::Color_Magenta)
      .value("Purple", Viewer::Color_Purple)
      .value("Orange", Viewer::Color_Orange)
      .value("Yellow", Viewer::Color_Yellow)
      .value("Brown", Viewer::Color_Brown)
      .value("Total", Viewer::Color_Total);

  m.def("draw_random_things", &drawRandomThings);
}
