//
// Created by bene on 01.07.23.
//

#pragma once

#include <Eigen/Core>
#include <iostream>
#include <memory>

#include "core/math/transformations.hpp"
#include "core/parameters.hpp"
#include "core/sensor_data.h"

namespace ukf {
namespace core {
namespace detail {

template <typename State, typename P>
std::ostream &operator<<(std::ostream &os, const std::pair<State, P> &s) {
  os << "state:\n"
     << s.first << "\n"
     << "covariance:\n"
     << s.second << "\n";
  return os;
}

class Ukf {
 public:
  virtual ~Ukf() = default;

  explicit Ukf(UkfParameters parameters) : _parameters(parameters) {}

  explicit Ukf(UkfParameters::ScalingParameters params) : _parameters(params) {}

  Ukf() : _parameters({1e-3, 2, 0}) {}

 protected:
  /**
   * Performs a time step
   * @param X current state vector
   * @param P current covarianceMatrix
   * @param sensorData sensorData after given step
   * @param dt time since last step
   */
  template <typename State, typename Covariance, typename SensorData,
            typename... Inputs>
  std::pair<State, Covariance> timeStep(State const &X, Covariance const &P,
                                        double dt, SensorData const &sensorData,
                                        Inputs &&...inputs) {
    std::pair<State, Covariance> newState{};
    State &X_new = newState.first;
    Covariance &P_new = newState.second;
    _parameters.update(X.rows());
    newState = predict(X, P, dt, inputs... /*std::forward<Inputs>(inputs)...*/);
    if (sensorData.size() == 0) {
      return newState;
    }
    return update(X_new, P_new, sensorData);
  }

 private:
  // region non containing ukf
  template <typename State, typename Covariance, typename... Inputs>
  std::pair<State, Covariance> predict(State const &X, Covariance const &P,
                                       double dt, Inputs &&...inputs) {
    std::pair<State, Covariance> result{};
    auto &X_new = result.first;
    auto &P_new = result.second;

    // Redraw sigma points to incorporate noising
    const SigmaPoints sigmaPoints = math::drawSigmaPoints(X, P, _parameters);

    SigmaPoints transformedSigmaPoints =
        SigmaPoints::Zero(sigmaPoints.rows(), sigmaPoints.cols());

    // propagate (f)
    math::propagate(sigmaPoints, transformedSigmaPoints, [=](const auto &s) {
      // constructing state from vector to access the necessary fields
      return State(s).f(dt, inputs... /*std::forward<Inputs>(inputs)...*/);
    });

    X_new = math::calculateMean(transformedSigmaPoints, _parameters);
    P_new = math::calculateSquareRootCovariance(transformedSigmaPoints, X_new,
                                                getSystemNoise(), _parameters);
    return result;
  }

  template <typename State, typename Covariance, typename SensorData>
  std::pair<State, Covariance> update(State const &X, Covariance const &P,
                                      const SensorData &sensorData) {
    std::pair<State, Covariance> result{X, P};
    auto &X_new = result.first;
    auto &P_new = result.second;

    // Redraw sigma points to incorporate noising
    const SigmaPoints sigmaPoints = math::drawSigmaPoints(X, P, _parameters);

    // calculate measurement size: #Sigmapoints with size of sensorData ->
    // sensorDataSize Rows x #Sigmapoints
    SigmaPoints gammaPoints =
        SigmaPoints::Zero(sensorData.vector().size(), sigmaPoints.cols());

    // Propagate for measurement prediction
    math::propagate(sigmaPoints, gammaPoints, [sensorData](const auto &s) {
      return sensorData.h(State(s));
    });

    // Calculate expected measurement with error covariance
    const Mean measurementMean = math::calculateMean(gammaPoints, _parameters);
    const CovarianceMatrixType S_zz = math::calculateSquareRootCovariance(
        gammaPoints, measurementMean, sensorData.noising(), _parameters);

    const Eigen::MatrixXf P_xz = math::calculateCrossVarianceMatrix(
        sigmaPoints, X, gammaPoints, measurementMean, _parameters);

    // Calculate Kalman Gain with K=(P_zz/S_zz^T)/S_zz=(S_zz^T\(S_zz\P_zz^T))^T=
    const Eigen::MatrixXf K =
        S_zz.transpose()
            .fullPivHouseholderQr()
            .solve(S_zz.fullPivHouseholderQr().solve(P_xz.transpose()))
            .transpose();

    const Eigen::MatrixXf U = K * S_zz;
    Eigen::MatrixXf covarianceMatrix = P.matrix();

    // Perform rank Update for each column in U
    // Eigen::LLT<Eigen::MatrixXf> ltt(covarianceMatrix);

    if (!math::performCholeskyUpdate(covarianceMatrix, U, -1)) {
      std::cerr << "Error during rank update\n";
      return result;  // returning non modified version of the state
    }
    // Assign values to ukf state after successfully rank update
    P_new = covarianceMatrix;
    X_new += K * (sensorData.vector() - measurementMean);

    return result;
  }

  virtual Eigen::MatrixXf getSystemNoise() const = 0;
  // endregion non containing ukf

  UkfParameters _parameters;
};

}  // namespace detail

}  // namespace core

}  // namespace ukf
