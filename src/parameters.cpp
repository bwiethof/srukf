//
// Created by bwiethof on 2/27/23.
//

#include "../include/parameters.hpp"

namespace ukf {
    void UKFParameters::updateWeights(std::size_t L, double lambda) {
        _compositionParams.W0 = lambda / (L + lambda);
        _compositionParams.Wc = _compositionParams.W0 + (1 - _scalingParams.alpha * _scalingParams.alpha + _scalingParams.beta);
    }

    void UKFParameters::updateGamma(std::size_t L, double lambda) {
        _compositionParams.gamma = std::sqrt(lambda + L);
    }

    void UKFParameters::update(std::size_t L) {
        const auto lambda = calculateLambda(L);
        updateWeights(L, lambda);
        updateGamma(L, lambda);
    }
} // ukf