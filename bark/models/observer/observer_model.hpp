// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_EXECUTION_OBSERVER_MODEL_HPP_
#define BARK_MODELS_EXECUTION_OBSERVER_MODEL_HPP_

#include <Eigen/Core>
#include <memory>
#include "bark/world/world.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/commons/base_type.hpp"

namespace bark {
namespace models {
namespace observer {
using bark::world::World;
using bark::world::WorldPtr;
using bark::world::AgentId;
using bark::world::ObservedWorld;

/**
 * @brief  Observer creating an ObservedWorld given a World
 * @note   
 * @retval None
 */
class ObserverModel : public commons::BaseType {
 public:
  explicit ObserverModel(bark::commons::ParamsPtr params)
    : BaseType(params) {}

  ObserverModel(const ObserverModel& observer_model)
    : BaseType(observer_model.GetParams()) {}

  virtual ~ObserverModel() {}

  /**
   * @brief Function for generating an ObservedWorld
   * @param  world: BARK world
   *         agent_id: id of the agent
   * @retval  Returns an ObservedWorld
   */
  virtual ObservedWorld Observe(
    const WorldPtr& world, const AgentId& agent_id) = 0;

  /**
   * @brief  Function specifying how the world shall be cloned
   */
  virtual std::shared_ptr<ObserverModel> Clone() const = 0;

};

typedef std::shared_ptr<ObserverModel> ObserverModelPtr;

}  // namespace observer
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_EXECUTION_OBSERVER_MODEL_HPP_
