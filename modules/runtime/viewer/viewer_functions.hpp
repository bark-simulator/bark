// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.



#include <stdlib.h>

#include "modules/runtime/viewer/viewer.hpp"
#include "modules/geometry/geometry.hpp"


namespace modules {
namespace viewer {

using geometry::Point2d;
using geometry::Line;

inline void drawRandomThings(Viewer * viewer) {
  srand(time(NULL));
  const uint16_t num_points = 100;
  for (auto i = 0; i < num_points; ++i) {
    Point2d p(rand() % 10 +1, rand() % 10 + 1);
    viewer->drawPoint2d(p, Viewer::Color::Color_Blue, 1.0f);
  }
  const uint16_t num_lines = 100;
  for (auto i = 0; i < num_lines; ++i) {
    Line l;
    for (auto y = 0; y < 10; ++y) {
      Point2d p(rand() % 10 +1, rand() % 10 +1);
      l.add_point(p);
    }
    viewer->drawLine2d(l, Viewer::Color::Color_Cyan, 1.0f);
  }
}

}  // namespace viewer
}  // namespace modules
