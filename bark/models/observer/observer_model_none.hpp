// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_EXECUTION_OBSERVER_MODEL_NONE_HPP_
#define BARK_MODELS_EXECUTION_OBSERVER_MODEL_NONE_HPP_

#include <Eigen/Core>
#include <memory>
#include "bark/world/world.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/models/observer/observed_model.hpp"
#include "bark/commons/base_type.hpp"

namespace bark {
namespace models {
namespace observer {
using bark::world::World;
using bark::world::ObservedWorld;

/**
 * @brief  Observer creating an ObservedWorld given a World
 * @note   
 * @retval None
 */
class ObserverModelNone : public ObserverModel {
 public:
  explicit ObserverModelNone(bark::commons::ParamsPtr params)
    : BaseType(params) {}

  ObserverModelNone(const ObserverModelNone& execution_model)
    : BaseType(execution_model.GetParams()) {}

  virtual ~ObserverModelNone() {}

  /**
   * @brief Clones the world and generated an ObservedWrodl
   * @retval  Returns an ObservedWorld
   */
  virtual ObservedWorld Observe(const World& world) override {

  }

  /**
   * @brief  Function specifying how the world shall be cloned
   */
  virtual std::shared_ptr<ObserverModel> Clone() override;

};

inline std::shared_ptr<ObserverModel> ObserverModelNone::Clone() const {
  std::shared_ptr<ObserverModelNone> model_ptr =
      std::make_shared<ObserverModelNone>(*this);
  return model_ptr;
}

typedef std::shared_ptr<ObserverModelNone> ObserverModelNonePtr;

}  // namespace observer
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_EXECUTION_OBSERVER_MODEL_NONE_HPP_
