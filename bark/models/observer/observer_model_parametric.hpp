// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_OBSERVER_OBSERVER_MODEL_PARAMETRIC_HPP_
#define BARK_MODELS_OBSERVER_OBSERVER_MODEL_PARAMETRIC_HPP_

#include <memory>
#include "bark/commons/base_type.hpp"
#include "bark/models/observer/observer_model.hpp"
#include "bark/commons/distribution/distribution.hpp"

namespace bark {

namespace world {
namespace objects {
typedef unsigned int AgentId;
class Agent;
typedef std::shared_ptr<Agent> AgentPtr;
}  // namespace objects
class ObservedWorld;
class World;
typedef std::shared_ptr<World> WorldPtr;
typedef std::shared_ptr<ObservedWorld> ObservedWorldPtr;
}  // namespace world
namespace models {
namespace observer {
using bark::world::World;
using bark::world::WorldPtr;
using bark::world::objects::AgentId;
using bark::world::ObservedWorld;
using bark::world::ObservedWorldPtr;
using bark::world::objects::AgentPtr;
using bark::commons::DistributionPtr;

/**
 * @brief  Observer creating an ObservedWorld given a World
 * @note   
 * @retval None
 */
class ObserverModelParametric : public ObserverModel {
 public:
  explicit ObserverModelParametric(bark::commons::ParamsPtr params);

  ObserverModelParametric(const ObserverModelParametric& observer_model);

  virtual ~ObserverModelParametric() {}

  virtual ObservedWorld Observe(
    const WorldPtr& world, const AgentId& agent_id) override;

 private:
  void AddStateDeviationFrenet(const AgentPtr& agent,
                         const DistributionPtr& multi_dim_distribution,
                         const ObservedWorldPtr& previous_observed_world,
                         const double& delta_time) const;
  const DistributionPtr ego_state_deviation_dist_;
  const DistributionPtr others_state_deviation_dist_;
};

}  // namespace observer
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_OBSERVER_OBSERVER_MODEL_PARAMETRIC_HPP_
