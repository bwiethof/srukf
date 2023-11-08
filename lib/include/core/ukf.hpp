//
// Created by bene on 01.07.23.
//

#pragma once

#include "core/parameters.hpp"
#include "core/sensor_data.h"
#include "covariance.hpp"
#include "core/detail/ukf_impl.hpp"
#include "state.hpp"
#include <Eigen/Core>

namespace ukf {
    namespace core {

        // Declare template with single parameter for template specialization
        template<typename State_Fields>
        class Ukf;

        /**
         * @class Ukf
         * @brief The Unscented Kalman Filter class.
         *
         * This class represents an implementation of the Unscented Kalman Filter (UKF) algorithm.
         * The UKF is used for state estimation in nonlinear systems.
         *
         * @tparam State_Fields The state fields used in the filter.
         */
        template<typename ...State_Fields>
        class Ukf<ukf::core::StateFields<State_Fields...>> : public detail::Ukf {
            static constexpr std::size_t Size = ukf::core::StateFields<State_Fields...>::StateSize;
            using StateType = State<StateFields<State_Fields...>>;
            using CovarianceType = Covariance<StateFields<State_Fields...>>;


        public:
            Ukf() = default;

            Ukf(const StateType &X, const CovarianceType &P)
                    : _state(X),
                      _covariance(P) {}

            Ukf(StateType &&X, CovarianceType &&P)
                    : _state(std::move(X)),
                      _covariance(std::move(P)) {}

            template<typename Derived_State, typename Derived_Covariance>
            Ukf(const Eigen::MatrixBase<Derived_State> &X, const Eigen::MatrixBase<Derived_Covariance> &P)
                    : _state{X},
                      _covariance{P} {
                EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived_Covariance>, Size, Size);
                EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived_State>, Size);
            }

            template<typename Derived_State, typename Derived_Covariance>
            Ukf(Eigen::MatrixBase<Derived_State> &&X, Eigen::MatrixBase<Derived_Covariance> &&P)
                    : _state{std::move(X)},
                      _covariance{std::move(P)} {
                EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived_Covariance>, Size, Size);
                EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived_State>, Size);
            }

            /**
             * Performs a time step
             * @param sensorData sensorData after given step
             * @param dt time since last step
             */
            template<typename SensorData>
            void step(const SensorData &sensorData, double dt) {
                std::tie(_state, _covariance) =
                        this->timeStep<StateType, CovarianceType, SensorData>(_state, _covariance, dt, sensorData);
            }

            StateType getState() const {
                return _state;
            }

            CovarianceType getCovariance() const {
                return _covariance;
            }

        protected:
            Eigen::MatrixXf getSystemNoise() const override {
                return _state.noising();
            }


        private:

            StateType _state{};
            CovarianceType _covariance{};
        };


    }

}
