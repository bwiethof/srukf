

#pragma once

#include <Eigen/Core>
#include <Eigen/QR>
#include <iostream>
#include <numeric>

#include "core/parameters.hpp"
#include "core/typedefs.h"

namespace ukf {
namespace core {

namespace math {

template <typename Derived_X, typename Derived_P>
SigmaPoints drawSigmaPoints(const Eigen::MatrixBase<Derived_X> &X,
                            const Eigen::MatrixBase<Derived_P> &P,
                            const ukf::core::UkfParameters &parameters) {
  assert(X.rows() == P.rows());

  const auto L = X.size();  // missing sensor data for augmented

  const auto params = parameters.params();

  SigmaPoints sigmaPoints = SigmaPoints::Zero(L, 2 * L + 1);

  sigmaPoints.col(0) = X;
  for (long i = 0; i < L; ++i) {
    sigmaPoints.col(i + 1) = X + params.gamma * P.col(i);
    sigmaPoints.col(i + L + 1) = X - params.gamma * P.col(i);
  }

  return sigmaPoints;
}

template <typename Derived>
Mean calculateMean(const Eigen::MatrixBase<Derived> &sigmaPoints,
                   const UkfParameters &parameters) {
  const auto params = parameters.params();
  return std::accumulate(
      ++sigmaPoints.colwise().begin(), sigmaPoints.colwise().end(),
      Mean(params.W0_s * sigmaPoints.col(0)),
      [weight = params.Wi](const auto &currentMean, const auto &sigmaPoint) {
        return currentMean + weight * sigmaPoint;
      });
}

template <typename DerivedSigmaPoints, typename DerivedStateMean,
          typename DerivedGammaPoints, typename DerivedMeasurementMean>
CrossVariance calculateCrossVarianceMatrix(
    const Eigen::MatrixBase<DerivedSigmaPoints> &sigmaPoints,
    const Eigen::MatrixBase<DerivedStateMean> &mean,
    const Eigen::MatrixBase<DerivedGammaPoints> &gammaPoints,
    const Eigen::MatrixBase<DerivedMeasurementMean> &measurementMean,
    const UkfParameters &parameters) {
  const auto params = parameters.params();

  const DifferenceMatrix deltasState = sigmaPoints.colwise() - mean;
  const DifferenceMatrix deltasGamma = gammaPoints.colwise() - measurementMean;

  return params.W0_c * deltasState.col(0) * deltasGamma.col(0).transpose() +
         params.Wi *
             (deltasState.rightCols(sigmaPoints.cols() - 1) *
              deltasGamma.rightCols(sigmaPoints.cols() - 1).transpose());
}

/**
 * @brief Perform the Cholesky update on a given covariance matrix
 *
 * This function performs a Cholesky update on a given covariance matrix. It
 * updates the covariance matrix by adding multiple update matrices to it.
 *
 * @Remarks P needs to be a lower triangular matrix
 *
 * @tparam DerivedCovariance The type of the covariance matrix
 * @tparam DerivedUpdateMatrix The type of the update matrix
 * @param P The covariance matrix to update
 * @param U The update matrix
 * @param value The value to be added to the update matrix
 * @return Returns true if the Cholesky update is successful, otherwise returns
 * false.
 */
template <typename DerivedCovariance, typename DerivedUpdateMatrix>
bool performCholeskyUpdate(Eigen::MatrixBase<DerivedCovariance> &P,
                           const Eigen::MatrixBase<DerivedUpdateMatrix> &U,
                           float value) {
  for (auto const &u : U.colwise()) {
    const auto row =
        Eigen::internal::llt_inplace<float, Eigen::Lower>::rankUpdate(
            P.derived(), u, value);
    if (row >= 0) return false;
  }
  return true;
}

template <typename S, typename M, typename Q>
Covariance calculateSquareRootCovariance(
    const Eigen::MatrixBase<S> &sigmaPoints, const Eigen::MatrixBase<M> &mean,
    const Eigen::MatrixBase<Q> &sqrtNoise, const UkfParameters &parameters) {
  const auto params = parameters.params();
  const auto numRows = sigmaPoints.rows();
  const auto numCols = sigmaPoints.cols();

  DifferenceMatrix A =
      DifferenceMatrix::Zero(numRows, numCols - 1 + sqrtNoise.cols());
  A << (std::sqrt(params.Wi) *
        (sigmaPoints.block(0, 1, numRows, numCols - 1).colwise() - mean)),
      sqrtNoise;

  // Calculate R_prime from householder as upper right triangle
  Matrix<Eigen::Dynamic, Eigen::Dynamic> R_prime =
      A.transpose()
          .householderQr()
          .matrixQR()
          .topLeftCorner(mean.size(), mean.size())
          .template triangularView<Eigen::Upper>();

  // Transpose since Upper triangular Part of R = L^T
  R_prime.transposeInPlace();
  if (!performCholeskyUpdate(R_prime, sigmaPoints.col(0) - mean, params.W0_c)) {
    std::cerr << "Error performing rank update"
              << "\n";
  };

  return R_prime;
}

template <typename DerivedIn, typename DerivedOut, typename Propagator>
void propagate(const Eigen::MatrixBase<DerivedIn> &sigmaPoints,
               Eigen::MatrixBase<DerivedOut> &transformedSigmaPoints,
               Propagator propagator) {
  std::transform(sigmaPoints.colwise().begin(), sigmaPoints.colwise().end(),
                 transformedSigmaPoints.colwise().begin(), propagator);
}
}  // namespace math

}  // namespace core
}  // namespace ukf
