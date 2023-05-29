//
// Created by bene on 13.05.23.
//

#pragma once

#include <cstddef>
#include "EigenInc.h"

namespace ukf {
    namespace core {
        namespace detail {
            template<typename Derived>
            struct traits;

            template<typename T>
            struct traits<const T> : traits<T> {
            };

            template<typename Derived>
            struct SensorBase;
        }
    }

    namespace slam {
        template<size_t S, typename DataType>
        struct Sensor;

        namespace detail {

            template<typename Derived>
            struct SlamSensorBase;

        }
    }
    namespace detail {

        template<typename Derived>
        struct StaticSensorBase;
    }

    template<size_t S, typename DataType, int Major = Eigen::RowMajor>
    struct StaticSensor;


}
