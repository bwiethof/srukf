//
// Created by bene on 01.07.23.
//

#pragma once

#include "core/parameters.hpp"
#include "core/sensor_data.h"
#include "covariance.hpp"
#include "core/detail/ukf_impl.hpp"
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
            using StateType = State<StateFields<State_Fields...>>;
            using CovarianceType = Covariance<StateFields<State_Fields...>>;

        public:
            Ukf() = default;

            /**
             * Performs a time step
             * @param sensorData sensorData after given step
             * @param dt time since last step
             */
            template<typename SensorData>
            void step(const SensorData &sensorData, double dt) {
                std::tie(this->_state, this->_covariance) = this->timeStep<StateType, CovarianceType, SensorData>(
                        _state, _covariance, dt, sensorData);
            }

        private:
            Eigen::MatrixXf getSystemNoise() const override {
                return _state.noising();
            }


        private:

            ukf::core::State<ukf::core::StateFields<State_Fields...>> _state{};
            ukf::core::Covariance<ukf::core::StateFields<State_Fields...>> _covariance{};

            Eigen::MatrixXf _systemNoise{};

        };


    }

}
