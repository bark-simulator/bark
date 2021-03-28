// Copyright (c) 2021 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#ifndef BARK_WORLD_RENDERER_PRIMITIVE_HPP_
#define BARK_WORLD_RENDERER_PRIMITIVE_HPP_

#include <memory>
#include <variant>
#include <string>
#include <map>
#include <vector>

namespace bark {
namespace world {
namespace renderer {

using bark::geometry::Point2d;
using bark::geometry::Line;
using bark::geometry::Polygon;

using RenderType = std::variant<Line, Point2d, Polygon>;

class RenderPrimitive {
 public:
  RenderPrimitive() {}
  RenderPrimitive(
    const RenderType& obj,
    std::string line_color = "blue",
    std::string face_color = "blue",
    std::string line_width = "blue",
    std::string line_style = "solid") : object_(obj) {
      conf_[line_color] = line_color;
      conf_[face_color] = face_color;
      conf_[line_width] = line_width;
      conf_[line_style] = line_style;
  }

  void SetAttr(std::string k, std::string v) {
    conf_[k] = v;
  }

  std::string GetAttr(std::string k) {
    return conf_[k];
  }

  RenderType GetObject() const {
    return object_;
  }

 private:
  std::map<std::string, std::string> conf_;
  RenderType object_;
};

using RenderPrimitivePtr = std::shared_ptr<RenderPrimitive>;

}  // namespace renderer
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_RENDERER_PRIMITIVE_HPP_
