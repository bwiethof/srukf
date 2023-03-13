//
// Created by bwiethof on 2/27/23.
//

#include "../include/ukf.hpp"
#include "../include/operations.h"
#include "../include/sensor_data.hpp"
#include "../include/square_root_covariance.hpp"
#include "../include/state.hpp"
#include "../include/transitions.hpp"


// TODO: Implement https://www.robots.ox.ac.uk/~gk/publications/HolmesKleinMurrayPAMI_SRUKF.pdf

// Consider new Augmented State which derives from State and is able to augment on its own -> less lock in of Eigen

namespace ukf {

    void UKF::update(const SensorData &currentSensorData) {

        // Setup for current update
        SquareRootCovariance S(*_S);
        State X(*_X);

        _measurementTransition->updateSensorData(currentSensorData);
        const auto Q = X.noise();

        _parameters.update(X.size());


        const SigmaPoints sigmaPoints = drawSigmaPoints(X, _parameters, S);
        const SigmaPoints transformSigmaPoints = _stateTransition->transform(sigmaPoints);

        X = calculateMean(transformSigmaPoints, _parameters);
        S = updateCovariance(transformSigmaPoints, X.vector(), Q.matrix(), _parameters);


        if (_measurementTransition->hasNewData()) {
            auto &&estimation = measurementUpdate(std::make_pair(X, S));
            X = std::move(estimation.first);
            S = std::move(estimation.second);
        }

        // update State for publish
        _X->update(std::move(X));
        _S->update(std::move(S));
    }

    UKF::UKF(std::unique_ptr<State> &&state, std::unique_ptr<SquareRootCovariance> &&S,
             std::unique_ptr<StateTransition> &&stateTransition,
             std::unique_ptr<MeasurementPrediction> &&measurementTransition)
            : _X(std::move(state)), _S(std::move(S)), _stateTransition(std::move(stateTransition)),
              _measurementTransition(std::move(measurementTransition)) {}

    UKF::Estimation
    UKF::measurementUpdate(const UKF::Estimation &currentEstimation) {

        const auto R = _measurementTransition->noise();


        State X(currentEstimation.first);
        SquareRootCovariance S = currentEstimation.second;
        const SigmaPoints sigmaPoints = drawSigmaPoints(X, _parameters, S);
        const GammaPoints gammaPoints = _measurementTransition->predict(sigmaPoints);

        const Mean z = calculateMean(gammaPoints, _parameters);
        const Eigen::MatrixXd S_z = updateCovariance(gammaPoints, z, R.matrix(), _parameters);

        // Calculate Kalman Gain to incorporate uncertainty of measurements and prediction
        const auto params = _parameters.params();
        const Eigen::MatrixXd P_xy = params.W0 * (sigmaPoints - X.vector()) * ((gammaPoints - z).transpose());
        const Eigen::MatrixXd K = (P_xy * S_z.transpose().inverse()) * S_z.inverse();

        const Eigen::MatrixXd U = K * S_z;
        X += K * (_measurementTransition->calculateMeasurement() - z);

        S.cholUpdate(U, -1);

        return std::make_pair(X, S);
    }

}