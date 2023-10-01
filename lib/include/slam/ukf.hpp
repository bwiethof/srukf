//
// Created by bene on 09.09.23.
//

#pragma once


#include "core/field.hpp"
#include "include/core/detail/ukf_impl.hpp"
#include "slam/covariance.h"
#include "slam/field.h"
#include "slam/state.h"

namespace ukf {
    namespace slam {

        template<typename State_Fields, typename Map_Fields>
        class Ukf;

        // TODO: Implement interface for Map Management (inheritance or template?)
        template<typename ...State_Fields, typename ...Map_Fields>
        class Ukf<ukf::core::StateFields<State_Fields...>, ukf::slam::MapFields<Map_Fields...>>
                : public ukf::core::detail::Ukf {
            using StateFieldsType = ukf::core::StateFields<State_Fields...>;
            using MapFieldsType = ukf::slam::MapFields<Map_Fields...>;
        public:

            Ukf() = default;

            template<typename SensorData>
            void step(SensorData const &sensorData, double dt) {
                std::tie(_state, _covariance) = timeStep(_state, _covariance, dt, sensorData);
                // TODO: Map Management will take place here
            }


        private:
            Eigen::MatrixXf getSystemNoise() const override {
                return _state.systemNoise();
            }


            ukf::slam::State<StateFieldsType, MapFieldsType> _state{};
            ukf::slam::Covariance<StateFieldsType, MapFieldsType> _covariance{};

        };
    }
}