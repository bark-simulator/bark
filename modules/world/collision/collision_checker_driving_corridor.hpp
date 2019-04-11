// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_COLLISION_COLLISION_CHECK_DRIVING_CORRIDOR_HPP_
#define MODULES_WORLD_COLLISION_COLLISION_CHECK_DRIVING_CORRIDOR_HPP_

#include "modules/world/collision/base_collision_checker.hpp"

namespace modules
{
namespace world
{
class World;

namespace collision
{

class CollisioncheckerDrivingCorridor : public BaseCollisionChecker
{
   public:
    explicit CollisioncheckerDrivingCorridor(commons::Params *params) : BaseCollisionChecker(params) {}
    virtual ~CollisioncheckerDrivingCorridor() {}

    bool checkCollision(const world::World& observed_world) const;

};

} // namespace collision
} // namespace world
} // namespace modules

#endif // MODULES_WORLD_COLLISION_COLLISION_CHECK_DRIVING_CORRIDOR_HPP_
