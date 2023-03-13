//
// Created by bwiethof on 2/27/23.
//

#pragma once

#include <utility>
#include <math.h>
#include <tuple>
#include "macros.hpp"

namespace ukf {

    class UKFParameters {

    public:

        struct ScalingParameters {
            double alpha{};
            double beta{};
            double kappa{};
        } __attribute__((aligned(32)));

        struct CompositionParameters {
            double W0{};
            double Wc{};
            double gamma{};
        } __attribute__((aligned(32)));


        // TODO mark as delete to be more explicit what is allowed
        UKFParameters() = default;

        explicit UKFParameters(const ScalingParameters &scalingParams) : _scalingParams(scalingParams) {}

        void update(std::size_t L);

        NO_DISCARD CompositionParameters params() const { return _compositionParams; }


    private:

        NO_DISCARD inline double calculateLambda(std::size_t L) const {
            return _scalingParams.alpha * _scalingParams.alpha * (L + _scalingParams.kappa) - L;
        }

        void updateGamma(std::size_t L, double lambda);

        void updateWeights(std::size_t L, double lambda);


        ScalingParameters _scalingParams{};
        CompositionParameters _compositionParams{};
    };

} // ukf
