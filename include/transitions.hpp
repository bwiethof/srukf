//
// Created by bwiethof on 3/1/23.
//

#pragma once

#include <vector>
#include <memory>
#include "EigenInc.h"
#include "sensor_data.hpp"

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
        using SensorExtractor = std::function<std::shared_ptr<const Sensor>(SensorData)>;
        using GammaPoints = Eigen::MatrixXd;

        MeasurementPrediction() = default;

        virtual ~MeasurementPrediction() = default;

        virtual GammaPoints predict(const Eigen::Ref<const Eigen::MatrixXd> &sigmaPoints);

        long dimension() const;

        bool hasNewData() const { return dimension() > 0; }

        Eigen::MatrixXd calculateMeasurement() const;

        void updateSensorData(const SensorData &sensorData);

        MeasurementNoise noise() const {
            return _R;
        }

    protected:

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

}