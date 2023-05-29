//
// Created by bene on 13.05.23.
//
#pragma once

#include <cstddef>
#include "core/util/fwd_declarations.h"
#include "statics.h"

namespace ukf {
    namespace core {
        namespace detail {
            // used to determine actual SensorType, e.g. Static or SLAM, see specializations below
            template<typename Derived, typename XprKind = typename traits<Derived>::XprKind>
            struct sensor_xpr_base {
            };

            template<typename Derived>
            struct sensor_xpr_base<Derived, SlamXpr> {
                using type = ukf::slam::detail::SlamSensorBase<Derived>;
            };

            template<typename Derived>
            struct sensor_xpr_base<Derived, StaticXpr> {
                using type = ::ukf::detail::StaticSensorBase<Derived>;
            };


            // Abstractions to define if Measurement or sensor type is looked for

            template<typename T>
            struct MeasurementTypeTrait {
                using Type = typename T::Type;
            };
            template<typename T>
            struct SensorTypeTrait {
                using Type = T;
            };

        };
    };
};
