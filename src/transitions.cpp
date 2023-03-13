//
// Created by bwiethof on 3/1/23.
//


#include <numeric>
#include "../include/transitions.hpp"
#include "../include/sensor_data.hpp"
#include "../include/state.hpp"

namespace ukf {

    Eigen::MatrixXd MeasurementPrediction::predict(const Eigen::Ref<const Eigen::MatrixXd> &sigmaPoints) {

        GammaPoints gammaPoints(dimension(), sigmaPoints.cols());
        for (int i = 0; i < sigmaPoints.cols(); ++i) {
            gammaPoints.col(i) = h(State(sigmaPoints.col(i)));
        }
        return gammaPoints;
    }

    Eigen::MatrixXd MeasurementPrediction::calculateMeasurement() const {
        Eigen::VectorXd measurement = Eigen::VectorXd::Zero(dimension());
        long idx = 0;
        for (const auto &sensor: _availableSensors) {
            measurement.segment(idx, sensor->dimension()) = sensor->measurement();
            idx += sensor->dimension();
        }
        return measurement;

    }

    long MeasurementPrediction::dimension() const {
        return std::accumulate(_availableSensors.begin(), _availableSensors.end(), 0L, [](long dim,
                                                                                          const std::shared_ptr<const Sensor> &sensor) {
            return dim + sensor->dimension();
        });
    }

    void MeasurementPrediction::updateSensorData(const SensorData &sensorData) {
        for (const auto &extractor: _sensorExtractors) {
            auto sensor = extractor(sensorData);
            if (sensor != nullptr) {
                addMeasurement(sensor);
                _R.addNoise(sensor);
            }
        }
    }



    Eigen::MatrixXd StateTransition::transform(const Eigen::MatrixXd &sigmaPoints) {

        Eigen::MatrixXd transformedSigmaPoints(sigmaPoints.rows(), sigmaPoints.cols());

        for (int i = 0; i < sigmaPoints.cols(); ++i) {
            transformedSigmaPoints.col(i) = f(State(sigmaPoints.col(i)));
        }

        return Eigen::MatrixXd();
    }
}