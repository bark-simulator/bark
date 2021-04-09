// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_OBSERVER_OBSERVER_MODEL_HPP_
#define BARK_MODELS_OBSERVER_OBSERVER_MODEL_HPP_

#include <memory>
#include "bark/commons/base_type.hpp"

namespace bark {

// forward declarations
namespace world {
namespace objects {
typedef unsigned int AgentId;
}  // namespace objects
class ObservedWorld;
class World;
typedef std::shared_ptr<World> WorldPtr;
}  // namespace world

namespace models {
namespace observer {
using world::objects::AgentId;
using world::World;
using world::ObservedWorld;
using world::WorldPtr;

/**
 * @brief  Observer creating an ObservedWorld given a World
 * @note   
 * @retval None
 */
class ObserverModel : public commons::BaseType {
 public:
  explicit ObserverModel(bark::commons::ParamsPtr params)
    : BaseType(params),
      observe_only_for_agents_() {}

  ObserverModel(const ObserverModel& observer_model)
    : BaseType(observer_model.GetParams()),
      observe_only_for_agents_() {}

  virtual ~ObserverModel() {}

  /**
   * @brief Function for generating an ObservedWorld
   * @param  world: BARK-world
   *         agent_id: id of the agent
   * @retval  Returns an ObservedWorld
   */
  virtual ObservedWorld Observe(
    const WorldPtr& world, const AgentId& agent_id) = 0;

  void SetObserveOnlyForAgents(const std::vector<AgentId>& observe_only_for_agents) {
    observe_only_for_agents_ = observe_only_for_agents;
  }

  std::vector<AgentId> GetObserveOnlyForAgents() const {
    return observe_only_for_agents_;
  }

  private:
    std::vector<AgentId> observe_only_for_agents_;

};

typedef std::shared_ptr<ObserverModel> ObserverModelPtr;

}  // namespace observer
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_OBSERVER_OBSERVER_MODEL_HPP_
