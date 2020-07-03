// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_DYNAMIC_TRIPLE_INTEGRATOR_HPP_
#define BARK_MODELS_DYNAMIC_TRIPLE_INTEGRATOR_HPP_

#include "bark/models/dynamic/dynamic_model.hpp"

namespace bark {
namespace models {
namespace dynamic {

/**
 * @brief Triple integrator model
 *
 */
class TripleIntegratorModel : public DynamicModel {
 public:
  explicit TripleIntegratorModel(const bark::commons::ParamsPtr params)
      : DynamicModel(params), mass_(0.2) {
    mass_ = params->GetReal("DynamicModel::mass", "Mass of the object.", 1.);
  }
  virtual ~TripleIntegratorModel() {}

  /**
   * @brief State space model
   *
   * @param x State: t, x, y, theta, v, min_state, 6:x, vx, ax, 9:y, vy, ay,
   * 12:z, vz, az
   * @param u Input: ax, ay, az
   * @return State
   */
  State StateSpaceModel(const State& x, const Input& u) const {
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A(15, 15);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> B(15, 3);
    A << 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.,
        0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 0., 0., 0.;
    B << 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
        0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.,
        0., 0., 0., 0., 0., 0., 0., 0., 1.;
    return A * x + B * u;
  }

  std::shared_ptr<DynamicModel> Clone() const {
    std::shared_ptr<TripleIntegratorModel> model_ptr =
        std::make_shared<TripleIntegratorModel>(*this);
    return std::dynamic_pointer_cast<DynamicModel>(model_ptr);
  }

 private:
  float mass_;
};

}  // namespace dynamic
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_DYNAMIC_TRIPLE_INTEGRATOR_HPP_
