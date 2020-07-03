// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_VIEWER_VIEWER_HPP_
#define BARK_VIEWER_VIEWER_HPP_

#include "bark/geometry/geometry.hpp"

namespace bark {
namespace viewer {

using bark::geometry::Line;
using bark::geometry::Point2d;
using bark::geometry::Polygon;

class Viewer {
 public:
  enum Color {
    Color_White,
    Color_LightGray,
    Color_DarkGray,
    Color_Gray,
    Color_Black,
    Color_Pink,
    Color_Red,
    Color_DarkRed,
    Color_Lime,
    Color_LightGreen,
    Color_Green,
    Color_LightBlue,
    Color_Blue,
    Color_DarkBlue,
    Color_Cyan,
    Color_Aquamarine,
    Color_Teal,
    Color_Violet,
    Color_Magenta,
    Color_Purple,
    Color_Orange,
    Color_Yellow,
    Color_Brown,
    Color_Total
  };

  Viewer() {}
  virtual ~Viewer() {}
  virtual void drawPoint2d(const Point2d& point2d, const Color& color,
                           const float& alpha) = 0;
  virtual void drawLine2d(const Line& line, const Color& color,
                          const float& alpha) = 0;
  virtual void drawPolygon2d(const Polygon& polygon, const Color& color,
                             const float& alpha) = 0;
};

}  // namespace viewer
}  // namespace bark

#endif  // BARK_VIEWER_VIEWER_HPP_
