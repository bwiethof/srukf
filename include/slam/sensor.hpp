//
// Created by bene on 13.05.23.
//

#pragma once


#include <cstddef>
#include <type_traits>
#include <algorithm>
#include "core/util/traits.hpp"
#include "core/sensor.hpp"
#include "core/util/statics.h"

namespace ukf {

    template<size_t S, typename DataType>
    struct core::detail::traits<ukf::slam::Sensor<S, DataType>> {
        using Type = DataType;
        static constexpr int Size = S;
        using XprKind = SlamXpr;
    };

    namespace slam {

        namespace detail {

            template<typename Derived>
            struct SlamSensorBase : public core::detail::SensorBase<Derived> {
                using Base = core::detail::SensorBase<Derived>;
                using Base::derived;

                using Base::Size;
                using typename Base::Type;
                using typename Base::DataContainerType;
                using typename Base::NoiseContainerType;

                typename Base::EigenMeasurementType _z() const {
                    return typename Base::EigenMeasurementType(derived().z(std::move(derived()._data)).data());
                }

                typename Base::EigenNoiseType _R() const {
                    return typename Base::EigenNoiseType(derived().R(std::move(derived()._data)).data());
                }
            };
        } // namespace detail

        template<size_t S, typename DataType>
        struct Sensor : public ukf::core::Sensor<ukf::slam::Sensor<S, DataType>> {
            using Base = ukf::core::Sensor<ukf::slam::Sensor<S, DataType>>;
            using BaseType = Sensor<S, DataType>;
            using Base::Size;
            using Type = typename Base::Type;
            using typename Base::DataContainerType;
            using typename Base::NoiseContainerType;

            explicit Sensor(
                    DataType data,
                    std::enable_if_t<std::is_trivially_copyable<DataType>::value, bool> = 0,
                    std::enable_if_t<!std::is_rvalue_reference<DataType>::value, bool> = 0)
                    : _data(std::move(data)) {}

            DataType _data;
            std::size_t _id{};
        };

        struct NoOpSensor : public ukf::slam::Sensor<0, ukf::NoOp_t> {

            using Base = ukf::slam::Sensor<0, NoOp_t>;
            using Base::Sensor;

            Base::NoiseContainerType R(NoOp_t) const override { return {{}}; }

            Base::DataContainerType z(NoOp_t) const override { return {}; }
        };

    } // namespace slam
} // namespace ukf
