//
// Created by bene on 06.05.23.
//
#pragma once

#include <tuple>
#include <vector>
#include <numeric>
#include <unordered_map>
#include <Eigen/Core>
#include "core/util/lookups.h"
#include "core/util/traits.hpp"
#include "core/util/functor.hpp"


namespace ukf {
    namespace slam {

        template<typename ...Sensors>
        class SensorData {

        public:

            template<typename T>
            void setMeasurement(T &&data, std::size_t id) {
                using ukf::core::detail::index::getIndex;
                using ukf::core::detail::MeasurementTypeTrait;

                // should be vector<...>
                auto &sensors = std::get<getIndex<0, MeasurementTypeTrait, T, Sensors...>()>(_sensors);
                sensors.emplace_back(std::forward<T>(data), id);
            }

            void constructMeasurement() {
                using core::detail::operation::transform;
                const auto seq = std::make_index_sequence<sizeof...(Sensors)>();
                const auto size = transform(_sensors, seq, core::detail::MeasurementSizeFunctor{});

                _measurement.resize(size);
                const auto measurements = transform(_sensors, seq, core::detail::MeasurementFunctor{});
                std::accumulate(measurements.begin(), measurements.end(), 0, [&](int offset, const auto &m) {
                    const auto sizeM = m.size();
                    _measurement.segment(offset, sizeM) = m;
                    return offset + sizeM;
                });

            }

            template<typename T>
            std::vector<std::size_t> getOrderedIds() {
                using ukf::core::detail::index::getIndex;
                using ukf::core::detail::SensorTypeTrait;
                auto idSelector = [](auto sensor) { return sensor._id; };

                const std::vector<T> sensors = std::get<getIndex<0, SensorTypeTrait, T, Sensors...>()>(_sensors);
                std::vector<std::size_t> ids(sensors.size());
                std::transform(sensors.begin(), sensors.end(), ids.begin(), idSelector);

                return ids;
            }

            Eigen::MatrixXf calculateNoising() {
                using core::detail::operation::transform;
                const auto seq = std::make_index_sequence<sizeof...(Sensors)>();
                const auto size = transform(_sensors, seq, core::detail::MeasurementSizeFunctor{});


                Eigen::MatrixXf R = Eigen::MatrixXf::Zero(size, size);
                return R;
            }

        private:
            std::tuple<std::vector<Sensors>...>
                    _sensors;

            Eigen::VectorXf _measurement{};
        };

    } // detail
} // ukf
