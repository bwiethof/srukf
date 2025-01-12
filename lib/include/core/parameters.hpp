//
// Created by bwiethof on 2/27/23.
//

#pragma once

#include <cmath>

#include "core/detail/macros.hpp"

namespace ukf {
namespace core {

class UkfParameters {
 public:
  struct ScalingParameters {
    double alpha{};
    double beta{};
    double kappa{};
  };

  struct CompositionParameters {
    double W0_s{};   // Weight for mean value to calculate mean
    double W0_c{};   // Weight for mean value to calculate covariance
    double Wi{};     // Weight for i=1,...,L to calculate means and covariance
    double gamma{};  // Scaling parameter to draw sigma points
  };

  explicit UkfParameters(const ScalingParameters& scalingParams)
      : _scalingParams(scalingParams) {}

  void update(std::size_t L) {
    const auto lambda = calculateLambda(L);
    updateWeights(L, lambda);
    updateGamma(L, lambda);
  }

  NO_DISCARD CompositionParameters params() const { return _compositionParams; }

 private:
  NO_DISCARD inline double calculateLambda(std::size_t L) const {
    return _scalingParams.alpha * _scalingParams.alpha *
               (L + _scalingParams.kappa) -
           L;
  }

  void updateGamma(std::size_t L, double lambda) {
    _compositionParams.gamma = std::sqrt(lambda + static_cast<double>(L));
  }

  void updateWeights(std::size_t L, double lambda) {
    _compositionParams.W0_s = lambda / (static_cast<double>(L) + lambda);
    _compositionParams.W0_c =
        _compositionParams.W0_s +
        (1.0 - _scalingParams.alpha * _scalingParams.alpha +
         _scalingParams.beta);
    _compositionParams.Wi = 1.0 / (2.0 * (static_cast<double>(L) + lambda));
  }

  ScalingParameters _scalingParams{};
  CompositionParameters _compositionParams{};
};
}  // namespace core

}  // namespace ukf
