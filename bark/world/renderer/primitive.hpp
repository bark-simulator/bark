// Copyright (c) 2021 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#ifndef BARK_WORLD_RENDERER_PRIMITIVE_HPP_
#define BARK_WORLD_RENDERER_PRIMITIVE_HPP_

#include <Eigen/Core>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

namespace bark {
namespace world {
namespace renderer {

using bark::geometry::Line;
using bark::geometry::Point2d;
using bark::geometry::Polygon;
using State = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using Line3d = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

using HolderType =
    std::variant<Line, Point2d, Polygon, double, int, State, Line3d>;
using ParamType = std::variant<double, int, std::string, std::vector<double>,
                               std::vector<int>>;

struct RenderPrimitive {
  explicit RenderPrimitive(const HolderType& obj) : object(obj) {}

  void SetAttr(std::string k, ParamType v) { conf[k] = v; }

  ParamType GetAttr(std::string k) { return conf[k]; }

  HolderType GetObject() const { return object; }

  std::map<std::string, ParamType> conf;
  HolderType object;
};

using RenderPrimitivePtr = std::shared_ptr<RenderPrimitive>;

}  // namespace renderer
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_RENDERER_PRIMITIVE_HPP_
