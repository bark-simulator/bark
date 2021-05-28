// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_OBSERVER_OBSERVER_MODEL_NONE_HPP_
#define BARK_MODELS_OBSERVER_OBSERVER_MODEL_NONE_HPP_

#include <string>
#include <memory>
#include "bark/world/world.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/commons/base_type.hpp"

namespace bark {
namespace models {
namespace observer {
using bark::world::World;
using bark::world::WorldPtr;
using bark::world::objects::AgentId;
using bark::world::ObservedWorld;
using world::renderer::RendererPtr;

/**
 * @brief  Observer creating an ObservedWorld given a World
 * @note   
 * @retval None
 */
class ObserverModelNone : public ObserverModel {
 public:
  explicit ObserverModelNone(bark::commons::ParamsPtr params)
    : ObserverModel(params) {}

  ObserverModelNone(const ObserverModelNone& observer_model)
    : ObserverModel(observer_model.GetParams()) {}

  virtual ~ObserverModelNone() {}

  /**
   * @brief Clones the world and generated an ObservedWrodl
   * @retval  Returns an ObservedWorld
   */
  virtual ObservedWorld Observe(
    const WorldPtr& world, const AgentId& agent_id) {
    // NOTE: this creates a standard observed world
    ObservedWorld observed_world(world, agent_id);
    // NOTE: generate child renderer for the observed world
    RendererPtr renderer = world->GetRenderer()->AddRendererChild(
      std::to_string(agent_id));
    observed_world.SetRenderer(renderer);
    return observed_world;
  }

};

typedef std::shared_ptr<ObserverModelNone> ObserverModelNonePtr;

}  // namespace observer
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_OBSERVER_OBSERVER_MODEL_NONE_HPP_
