//
// Created by bwiethof on 3/1/23.
//

#pragma once

//

#include "core/util/EigenInc.h"
#include "core/util/macros.hpp"
#include "noise.hpp"
#include <memory>
#include <unordered_map>
#include <vector>


namespace ukf {

    class Noise;

    class Sensor {

    public:
        explicit Sensor(long dimension) : _dimension(dimension) {}

        virtual ~Sensor() = default;

        constexpr long dimension() const { return _dimension; };

        virtual Eigen::VectorXd measurement() const = 0;

        virtual const std::shared_ptr<Noise> noise() const = 0;

    private:
        long _dimension{};
    };


    class SensorData {

    public:
        using Measurement = Eigen::VectorXd;

        virtual ~SensorData() = default;

        // currently only supports single instance of sensor type.
        // Second sensor will be ignored
        inline void add(const std::shared_ptr<const Sensor> &sensor) {

            _data.emplace_back(sensor);
        }

        template<typename T>
        NO_DISCARD Measurement measurement() const {
            const auto sensor = getMeasurement<T>();
            if (sensor == nullptr) {
                return {};
            }
            return sensor->measurement();
        }

        template<typename T>
        inline bool hasMeasurement() {
            return getMeasurement<T>() != nullptr;
        }

    private:
        template<typename T>
        const T *getMeasurement() const {

            for (const auto &sensor: _data) {
                if (const auto value = dynamic_cast<const T *>(sensor.get())) {
                    return value;
                }
            }
            return nullptr;
        }

        std::vector<std::shared_ptr<const Sensor>> _data;
    };

} // namespace ukf
