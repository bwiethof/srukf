//
// Created by bene on 11.05.23.
//

#pragma once

#include <cstddef>
#include <vector>
#include "core/util/EigenInc.h"
#include "core/util/traits.hpp"
#include "core/sensor.hpp"
#include "core/util/lookups.h"

namespace ukf {

    template<size_t S, typename DataType>
    struct core::detail::traits<ukf::StaticSensor<S, DataType>> {
        using Type = DataType;
        static constexpr int Size = S;
        using XprKind = StaticXpr;
    };

    namespace detail {


        template<typename Derived>
        struct StaticSensorBase : public core::detail::SensorBase<Derived> {
            using Base = core::detail::SensorBase<Derived>;
            using Base::derived;

            using Base::Size;
            using typename Base::Type;

            typename Base::EigenMeasurementType z(typename Base::Type data) const {
                return typename Base::EigenMeasurementType(derived().zImpl(data).data());
            }

            typename Base::EigenNoiseType R(typename Base::Type data) const {
                using ukf::core::detail::operation::flatten;
                return typename Base::EigenNoiseType(flatten(derived().RImpl(data)).data()).transpose();
            }
        };
    }

    template<size_t S, typename DataType, int Major>
    struct StaticSensor : public ukf::core::Sensor<StaticSensor<S, DataType, Major>> {
        using Base = core::Sensor<StaticSensor>;
        using BaseType = StaticSensor<S, DataType>;
        using typename Base::DataContainerType;
        using typename Base::NoiseContainerType;

    };


    struct NoOpSensor : public ukf::StaticSensor<0, NoOp_t> {

        using Base = ukf::StaticSensor<0, NoOp_t>;
        using Base::Sensor;

        Base::NoiseContainerType RImpl(const NoOp_t &) const override { return {{}}; }

        Base::DataContainerType zImpl(const NoOp_t &) const override { return {}; }
    };

}
