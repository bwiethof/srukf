//
// Created by bwiethof on 3/1/23.
//

#pragma once

#include "core/util/EigenInc.h"
#include "core/util/macros.hpp"
#include "sensor_data.hpp"
#include <memory>
#include <vector>

namespace ukf {

    class State;

    class Sensor;

    class SensorData;

/**
 * Base class to describe the state transition model
 */
    class StateTransition {
    public:
        StateTransition() = default;

        virtual ~StateTransition() = default;

        Eigen::MatrixXd transform(const Eigen::MatrixXd &sigmaPoints);

    private:
        virtual Eigen::VectorXd f(State &&) = 0;
    };

/**
 * Base class to describe the measurement prediction model based on the
 */
    class MeasurementPrediction {
    public:
        // Only accepting the supported data(?)
        using SensorExtractor =
                std::function<std::shared_ptr<const Sensor>(SensorData)>;
        using GammaPoints = Eigen::MatrixXd;

        MeasurementPrediction() = default;

        virtual ~MeasurementPrediction() = default;

        virtual GammaPoints
        predict(const Eigen::Ref<const Eigen::MatrixXd> &sigmaPoints);

        NO_DISCARD long dimension() const;

        NO_DISCARD bool hasNewData() const { return dimension() > 0; }

        NO_DISCARD Eigen::MatrixXd calculateMeasurement() const;

        void updateSensorData(const SensorData &sensorData);

        NO_DISCARD MeasurementNoise noise() const { return _R; }

        inline void registerExtractor(SensorExtractor &&extractor) {
            _sensorExtractors.emplace_back(std::move(extractor));
        }

    private:
        virtual Eigen::VectorXd h(State &&) = 0;

        inline void addMeasurement(const std::shared_ptr<const Sensor> &sensor) {
            _availableSensors.emplace_back(sensor);
        }

        std::vector<SensorExtractor> _sensorExtractors;
        std::vector<std::shared_ptr<const Sensor>> _availableSensors{};
        MeasurementNoise _R;
    };

} // namespace ukf
