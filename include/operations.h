//
// Created by bwiethof on 2/27/23.
//
#pragma once

#include <Eigen/Dense>

#include "core/util/EigenInc.h"
#include "parameters.hpp"


namespace ukf {
    class State;

    class SquareRootCovariance;

    class UKFParameters;

    using SigmaPoints = Eigen::MatrixXd;
    using Mean = Eigen::VectorXd;

    SigmaPoints drawSigmaPoints(const State &state, const UKFParameters &params, const SquareRootCovariance &S);


    Eigen::MatrixXd augmentCovariance(const SquareRootCovariance &S);

    Eigen::VectorXd augmentState(const State &S);

    template<typename Derived>
    Mean calculateMean(const Eigen::MatrixBase<Derived> &sigmaPoints, const UKFParameters &parameters) {

        const auto params = parameters.params();

        Mean mean = params.W0 * sigmaPoints.col(0);
        for (int i = 1; i < sigmaPoints.cols(); ++i) {
            mean += params.Wc * sigmaPoints.col(i);
        }

        return mean;
    }

    template<typename S, typename M, typename Q>
    Eigen::MatrixXd updateCovariance(const Eigen::MatrixBase<S> &sigmaPoints, const Eigen::MatrixBase<M> &mean,
                                     const Eigen::MatrixBase<Q> &noise, const UKFParameters &parameters) {

        const auto params = parameters.params();

        const auto numRows = sigmaPoints.rows();
        const auto numCols = sigmaPoints.cols();
        const auto &&temp = std::sqrt(params.W0) * (sigmaPoints.block(0, 1, numRows, numCols - 1).colwise() - mean);
        const auto sqrtQ = noise.cwiseSqrt();
        Eigen::MatrixXd toBeQR(numRows, numCols + noise.cols());
        toBeQR << temp, sqrtQ;
        const Eigen::MatrixXd updated = Eigen::HouseholderQR<Eigen::MatrixXd>(
                toBeQR).matrixQR().triangularView<Eigen::Upper>();


        auto S_new = updated.llt();
        S_new.rankUpdate(sigmaPoints.col(0) - mean, params.Wc);


        return S_new.matrixL();
    }

}
