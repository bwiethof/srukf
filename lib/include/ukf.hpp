//
// Created by bwiethof on 2/27/23.
//

#pragma once


#include <memory>
#include "parameters.hpp"
#include "core/util/EigenInc.h"

namespace ukf {
    class State;

    class SquareRootCovariance;

    class StateTransition;

    class MeasurementPrediction;

    class SensorData;

    using SigmaPoints = Eigen::MatrixXd;
    using GammaPoints = Eigen::MatrixXd;

    class UKF {

    public:
        UKF() = default;

        UKF(std::unique_ptr<State> &&state,
            std::unique_ptr<SquareRootCovariance> &&S,
            std::unique_ptr<StateTransition> &&stateTransition,
            std::unique_ptr<MeasurementPrediction> &&measurementTransition);

        void update(const SensorData &currentSensorData);

        const State *getState() const { return _X.get(); }

        const SquareRootCovariance *getSquareRootCovariance() const { return _S.get(); }

    private:

        using Estimation = std::pair<State, SquareRootCovariance>;

        UKF::Estimation
        measurementUpdate(const UKF::Estimation &currentEstimation);

        std::unique_ptr<State> _X;
        std::unique_ptr<SquareRootCovariance> _S;
        std::unique_ptr<StateTransition> _stateTransition;
        std::unique_ptr<MeasurementPrediction> _measurementTransition;

        UKFParameters _parameters{};

    };
}
