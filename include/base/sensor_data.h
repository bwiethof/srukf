//
// Created by bene on 25.04.23.
//

#ifndef SRUKF_SENSOR_DATA_H
#define SRUKF_SENSOR_DATA_H

#include <cstddef>
#include <Eigen/Core>
#include <array>
#include <limits>
#include <vector>
#include <typeindex>
#include <iostream>

#include "core/util/lookups.h"
#include "core/util/traits.hpp"


namespace ukf {

    namespace detail {

        template<typename T>
        constexpr std::size_t VectorDimension = T::Size;

        template<typename T>
        constexpr T adder(T v) { return v; }

        template<typename T, typename... Args>
        constexpr T adder(T first, Args...args) {
            return first + adder(args...);
        }

        template<typename... Sensors>
        constexpr std::size_t calculate_vector_size() { return adder(VectorDimension<Sensors>...); }

        template<typename T, std::size_t... Indices>
        constexpr std::array<T, sizeof...(Indices)> create_array(T value, std::index_sequence<Indices...>) {
            // Cast Indices to void to remove the unused value warning.
            return {{(static_cast<void>(Indices), value)...}};
        }

    }

    template<typename... Sensors>
    class StaticSensorData {
    public:

        StaticSensorData() = default;

        // Currently SLAM data ist not supported -> how to implement this?
        template<typename T>
        void setMeasurement(T data) {
            using ukf::core::detail::index::getIndex;
            using ukf::core::detail::MeasurementTypeTrait;

            // using reference since we need to assign a new value
            auto &offset = std::get<getIndex<0, MeasurementTypeTrait, Sensors...>()>(_fieldOffsets);
            auto s = std::get<getIndex<0, MeasurementTypeTrait, Sensors...>()>(_sensors);

            if (offset < _measurement.rows()) {
                _measurement.segment(offset, s.Size) = s(std::move(data));
                return;
            }

            const auto previousSize = _measurement.size();
            _measurement.resize(previousSize + s.Size);
            _measurement.segment(previousSize, s.Size) = s(std::move(data));
            offset = previousSize;
        }


    private:


        std::tuple<Sensors...> _sensors;
        std::array<std::size_t, sizeof...(Sensors)> _fieldOffsets = detail::create_array(
                std::numeric_limits<std::size_t>::max(), std::make_index_sequence<sizeof...(Sensors)>());

        Eigen::Matrix<float, Eigen::Dynamic, 1, 0, detail::calculate_vector_size<Sensors...>(), 1> _measurement;
    };


}


#endif //SRUKF_SENSOR_DATA_H
