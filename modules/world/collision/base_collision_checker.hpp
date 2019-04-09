// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_COLLISION_BASE_COLLISION_CHECKER_HPP_
#define MODULES_WORLD_COLLISION_BASE_COLLISION_CHECKER_HPP_

#include <memory>

#include "modules/commons/base_type.hpp"

namespace modules
{
namespace world
{

class World;
namespace collision
{

class BaseCollisionChecker : public modules::commons::BaseType
{
  public:
    explicit BaseCollisionChecker(commons::Params *params) : commons::BaseType(params) {};
    virtual ~BaseCollisionChecker() {};

    virtual bool checkCollision(const world::World& observed_world) const = 0;
};

typedef std::shared_ptr<BaseCollisionChecker> CollisionCheckerPtr;

} // namespace collision
} // namespace world
} // namespace modules

#endif // MODULES_WORLD_COLLISION_BASE_COLLISION_CHECKER_HPP_
