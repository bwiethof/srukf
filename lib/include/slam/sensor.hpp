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
#include "core/util/lookups.h"

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

                typename Base::EigenMeasurementType z() const {
                    return typename Base::EigenMeasurementType(derived().zImpl(std::move(derived()._data)).data());
                }

                typename Base::EigenNoiseType R() const {
                    using ukf::core::detail::operation::flatten;
                    return typename Base::EigenNoiseType(
                            flatten(derived().RImpl(derived()._data)).data()
                    ).transpose();
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

            Sensor(DataType data, std::size_t id)
                    : _data(std::move(data)), _id(id) {}

            DataType _data;
            std::size_t _id{};
        };

        struct NoOpSensor : public ukf::slam::Sensor<0, ukf::NoOp_t> {

            using Base = ukf::slam::Sensor<0, NoOp_t>;
            using Base::Sensor;

        protected:
            Base::NoiseContainerType RImpl(const NoOp_t &) const override { return {{}}; }

            Base::DataContainerType zImpl(const NoOp_t &) const override { return {}; }
        };

    } // namespace slam
} // namespace ukf
