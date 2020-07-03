// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_OBJECTS_OBJECT_HPP_
#define BARK_WORLD_OBJECTS_OBJECT_HPP_

#include "bark/commons/commons.hpp"
#include "bark/geometry/model_3d.hpp"
#include "bark/geometry/polygon.hpp"

namespace bark {
namespace world {
class World;
namespace objects {
typedef unsigned int AgentId;
class Object : public commons::BaseType {
 public:
  friend class world::World;

  Object(const geometry::Polygon& shape, const commons::ParamsPtr& params,
         const geometry::Model3D& model_3d = geometry::Model3D());

  Object(const Object& object)
      : BaseType(object),
        shape_(object.shape_),
        model_3d_(object.model_3d_),
        agent_id_(object.agent_id_) {}

  virtual ~Object() {}

  geometry::Polygon GetShape() const { return shape_; }
  geometry::Model3D GetModel3d() const { return model_3d_; }

  AgentId GetAgentId() const { return agent_id_; }

  void SetAgentId(const AgentId& agent_id) { agent_id_ = agent_id; }

  virtual std::shared_ptr<Object> Clone() const;

 private:
  geometry::Polygon shape_;
  geometry::Model3D model_3d_;
  AgentId agent_id_;

  static AgentId agent_count;
};

typedef std::shared_ptr<Object> ObjectPtr;

}  // namespace objects
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_OBJECTS_OBJECT_HPP_
