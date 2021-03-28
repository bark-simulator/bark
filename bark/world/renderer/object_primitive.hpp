// Copyright (c) 2021 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#ifndef BARK_WORLD_RENDERER_OBJECT_PRIMITIVE_HPP_
#define BARK_WORLD_RENDERER_OBJECT_PRIMITIVE_HPP_

#include <memory>
#include <variant>
#include <string>
#include <map>
#include <vector>

#include "bark/world/renderer/primitive.hpp"

namespace bark {
namespace world {
namespace renderer {

// NOTE: e.g., line, polygon, point
template<typename T>
class ObjectRenderPrimitive : public RenderPrimitive {
 public:
  ObjectRenderPrimitive(
    const T& object,
    std::string line_color = "blue",
    std::string face_color = "blue",
    std::string line_width = "blue",
    std::string line_style = "solid")
      : RenderPrimitive(
          line_color, face_color, line_width, line_style),
        object_(object) {}
    
  T GetObject() const {
    return object_;
  }

 private:
  T object_;
};

}  // namespace renderer
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_RENDERER_OBJECT_PRIMITIVE_HPP_
