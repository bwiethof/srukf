//
// Created by bwiethof on 2/28/23.
//

#pragma once

#include "EigenInc.h"

namespace ukf {

    class SquareRootCovariance {
    public:
        using Size = Eigen::Index;

        SquareRootCovariance() = default;


        template<typename Derived>
        SquareRootCovariance(Eigen::EigenBase<Derived> &&other):_data(std::move(other)) {}

        template<typename Derived>
        SquareRootCovariance &operator=(Eigen::EigenBase<Derived> &&other) {
            _data = std::move(other);
            return *this;
        }

        template<typename Derived>
        void cholUpdate(const Eigen::MatrixBase<Derived> &U, double mu) {
            Eigen::LLT<Eigen::Ref<Eigen::MatrixXd>> llt(_data);
            for (int i = 0; i < U.cols(); ++i) {
                llt.rankUpdate(U.col(i), mu);
            }
        }

        Eigen::MatrixXd matrix() const { return _data; }

        Size rows() const { return _data.rows(); }

        Size cols() const { return _data.cols(); }

        void update(SquareRootCovariance &&other) {
            *this = std::move(other);
        }


    private:
        Eigen::MatrixXd _data;
    };
}