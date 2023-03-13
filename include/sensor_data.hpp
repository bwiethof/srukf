//
// Created by bwiethof on 3/1/23.
//

#pragma once

//

#include <vector>
#include <memory>
#include <unordered_map>
#include "EigenInc.h"
#include "noise.hpp"


namespace ukf {

    class Noise;


//    namespace detail {

// maybe something with typetrait?
//        class SensorBase {
//        public:
//            virtual ~SensorBase() = default;
//
//            virtual long dimension() const = 0;
//
//        };
//
//    }


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


        inline void add(const std::shared_ptr<const Sensor> &sensor) {
            _data.emplace_back(sensor);
        }


        template<typename T>
        Measurement measurement() const {
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
        T *getMeasurement() {
            for (const auto &sensor: _data) {
                if (const auto *value = dynamic_cast<T>(sensor.get()) != nullptr) {
                    return value;
                }
            }
            return nullptr;
        }


        std::vector<std::shared_ptr<const Sensor>> _data;
    };

}