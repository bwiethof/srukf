//
// Created by bwiethof on 2/27/23.
//
#include "../include/operations.h"
#include "../include/state.hpp"
#include "../include/square_root_covariance.hpp"

namespace ukf {

    using MatrixXd = Eigen::MatrixXd;
    namespace {
        template<typename DerivedState, typename DerivedCovariance>
        SigmaPoints
        drawSigmaPoints(const Eigen::MatrixBase<DerivedState> &state,
                        const Eigen::MatrixBase<DerivedCovariance> &covariance,
                        const UKFParameters &parameters) {
            assert(state.size() == covariance.cols());

            const auto L = state.size(); // missing sensor data for augmented

            const auto params = parameters.params();

            SigmaPoints sigmaPoints(L, 2 * L + 1);
            sigmaPoints.setZero();


            sigmaPoints.col(0) = state;
            for (long i = 1; i < L; ++i) {
                sigmaPoints.col(i) += state + params.gamma * covariance.col(i);
                sigmaPoints.col(i + L) -= state + params.gamma * covariance.col(i);
            }

            return sigmaPoints;
        }

        template<typename P_, typename Q_, typename R_>
        MatrixXd augmentCovariance(const Eigen::MatrixBase<P_> &P, const Eigen::MatrixBase<Q_> &Q,
                                   const Eigen::MatrixBase<R_> &R) {

            const long size_P = P.rows();
            const long size_Q = Q.rows();
            const long size_R = R.rows();
            const long size = size_P + size_Q + size_R;

            MatrixXd P_augmented = MatrixXd::Zero(size, size);
            P_augmented.block(0, 0, size_P, size_P) = P;
            P_augmented.block(size_P, size_P, size_Q, size_Q) = Q;
            P_augmented.block(size_P + size_Q, size_P + size_Q, size_R, size_R) = R;

            return P_augmented;
        }
    }


    SigmaPoints drawSigmaPoints(const State &state, const UKFParameters &params, const SquareRootCovariance &S) {
        return drawSigmaPoints(state.vector(), S.matrix(), params);
    }

    Eigen::MatrixXd augmentCovariance(const SquareRootCovariance &S) {

        const Eigen::MatrixXd sqrQ(0, 0);
        const Eigen::MatrixXd sqrR(0, 0);

        return augmentCovariance(S.matrix(), sqrQ, sqrR);

    }


}