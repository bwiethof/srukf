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

    template<typename... Sensors>
    class StaticSensorData {
    public:

        StaticSensorData() = default;

        // Currently SLAM data ist not supported -> how to implement this?
        template<typename T>
        void setMeasurement(const T &data) {
            using ukf::core::detail::index::getIndex;
            using ukf::core::detail::MeasurementTypeTrait;

            // using reference since we need to assign a new value
            auto &offset = std::get<getIndex<0, MeasurementTypeTrait, T, Sensors...>()>(_fieldOffsets);
            auto s = std::get<getIndex<0, MeasurementTypeTrait, T, Sensors...>()>(_sensors);

            if (offset < _measurement.rows()) {
                _measurement.segment(offset, s.Size) = s._z(data);
                return;
            }

            const auto previousSize = _measurement.size();
            _measurement.resize(previousSize + s.Size);
            _measurement.segment(previousSize, s.Size) = s._z(data);
            offset = previousSize;
        }


    private:


        std::tuple<Sensors...> _sensors;
        std::array<std::size_t, sizeof...(Sensors)> _fieldOffsets = core::detail::generation::create_array(
                std::numeric_limits<std::size_t>::max(), std::make_index_sequence<sizeof...(Sensors)>());

        Eigen::Matrix<float, Eigen::Dynamic, 1, 0, core::detail::generation::calculate_vector_size<Sensors...>(), 1> _measurement;
    };


}


#endif //SRUKF_SENSOR_DATA_H
