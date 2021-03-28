// Copyright (c) 2021 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_RENDERER_RENDERER_HPP_
#define BARK_WORLD_RENDERER_RENDERER_HPP_

#include <memory>
#include <variant>
#include <string>
#include <map>
#include <vector>

#include "bark/world/renderer/primitive.hpp"

namespace bark {
namespace world {
namespace renderer {

class RendererBuffer {
 public:
  RendererBuffer() {}
  void Add(const RenderPrimitivePtr& rp) {
    buffer_.push_back(rp);
  }

  void Clear() {
    buffer_.clear();
  }

  std::vector<RenderPrimitivePtr> GetRenderPrimitives() const {
    return buffer_;
  }

 private:
  std::vector<RenderPrimitivePtr> buffer_;
};


}  // namespace renderer
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_RENDERER_RENDERER_HPP_
