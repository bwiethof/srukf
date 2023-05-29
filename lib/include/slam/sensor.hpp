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

                typename Base::EigenMeasurementType _z() const {
                    return typename Base::EigenMeasurementType(derived().z(std::move(derived()._data)).data());
                }

                typename Base::EigenNoiseType _R() const {
                    using ukf::core::detail::operation::flatten;
                    flatten(derived().R(derived()._data));
                    return typename Base::EigenNoiseType(
                            flatten(derived().R(derived()._data)).data()
                    );
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

            // TODO: chekc if it is possible to enable if rvalue reference
//            Sensor(DataType &&data, std::size_t id)
//                    : _data(std::move(data)), _id(id) {}
//
            Sensor(DataType data, std::size_t id)
                    : _data(std::move(data)), _id(id) {}

            DataType _data;
            std::size_t _id{};
        };

        struct NoOpSensor : public ukf::slam::Sensor<0, ukf::NoOp_t> {

            using Base = ukf::slam::Sensor<0, NoOp_t>;
            using Base::Sensor;

            Base::NoiseContainerType R(const NoOp_t &) const override { return {{}}; }

            Base::DataContainerType z(const NoOp_t &) const override { return {}; }
        };

    } // namespace slam
} // namespace ukf
