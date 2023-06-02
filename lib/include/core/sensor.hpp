//
// Created by bene on 13.05.23.
//

#pragma once

#include <Eigen/Core>
#include "core/util/traits.hpp"
#include "core/util/fwd_declarations.h"

namespace ukf {

    namespace core {

        namespace detail {
            template<typename Derived>
            struct PlainSensorBase : public sensor_xpr_base<Derived>::type {
                using Base = typename sensor_xpr_base<Derived>::type;
                using Base::Size;

                using typename Base::DataContainerType;
                using typename Base::NoiseContainerType;
            };


            template<typename Derived>
            struct SensorBase {
                static constexpr int Size = traits<Derived>::Size;
                using Type = typename traits<Derived>::Type;

                using DataContainerType = std::array<float, traits<Derived>::Size>;
                using NoiseContainerType = std::array<std::array<float, traits<Derived>::Size>, traits<Derived>::Size>;

                using EigenNoiseType = Eigen::Matrix<float, traits<Derived>::Size, traits<Derived>::Size>;
                using EigenMeasurementType = Eigen::Vector<float, traits<Derived>::Size>;

                Derived &derived() { return *static_cast<Derived *>(this); }

                const Derived &derived() const { return *static_cast<const Derived *>(this); }
            };

        }


        template<typename Derived>
        struct Sensor : public detail::PlainSensorBase<Derived> {
            using Base = detail::PlainSensorBase<Derived>;
            using Base::Size;

            using typename Base::DataContainerType;
            using typename Base::NoiseContainerType;

            virtual ~Sensor() = default;

            using Base::z;
            using Base::R;

        private:
            virtual DataContainerType zImpl(const typename Base::Type &data) const = 0;

            virtual NoiseContainerType RImpl(const typename Base::Type &data) const = 0;

            // User of the implementation
            friend typename Base::Base;
        };
    }

}
