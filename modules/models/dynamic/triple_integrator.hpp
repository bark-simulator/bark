// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_DYNAMIC_TRIPLE_INTEGRATOR_HPP_
#define MODULES_MODELS_DYNAMIC_TRIPLE_INTEGRATOR_HPP_

#include "modules/models/dynamic/dynamic_model.hpp"

namespace modules {
namespace models {
namespace dynamic {

/**
 * @brief Triple integrator model
 * 
 */
class TripleIntegratorModel : public DynamicModel {
 public:
  explicit TripleIntegratorModel(modules::commons::Params *params) :
    DynamicModel(params),
    mass_(0.2) {
      mass_ = params->get_real("DynamicModel::mass",
      "Mass of the object.",
      1.);
    }
  virtual ~TripleIntegratorModel() {}

  /**
   * @brief State space model
   * 
   * @param x State: t, x, vx, vy, y, vy, ay, z, vz, az
   * @param u Input: ax, ay, az
   * @return State 
   */
  State StateSpaceModel(const State &x, const Input &u) const {
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A(10, 10);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> B(10, 3);
    A << 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
         0., 0., 1., 0., 0., 0., 0., 0., 0., 0.,
         0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,
         0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
         0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
         0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,
         0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
         0., 0., 0., 0., 0., 0., 0., 0., 1., 0.,
         0., 0., 0., 0., 0., 0., 0., 0., 0., 1.,
         0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
    B << 0., 0., 0.,
         0., 0., 0.,
         0., 0., 0.,
         1., 0., 0.,
         0., 0., 0.,
         0., 0., 0.,
         0., 1., 0.,
         0., 0., 0.,
         0., 0., 0.,
         0., 0., 1.;
    return A*x + B*u;
  }

  DynamicModel *Clone() const {
    return new TripleIntegratorModel(*this);
  }

 private:
  float mass_;
};

}  // namespace dynamic
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_DYNAMIC_TRIPLE_INTEGRATOR_HPP_
