//
// Created by bwiethof on 3/5/23.
//

#pragma once

#include <memory>
#include <vector>
#include "EigenInc.h"
#include "sensor_data.hpp"

// Noise adapter for System and measurement noise necessary -> no injection of px4 in code
namespace ukf {
    class Noise {

    public:
        virtual ~Noise() = default;

        virtual Eigen::MatrixXd matrix() = 0;

        virtual long dimension() = 0; // should either num rows or num cols (Noise is a square matrix)

    };

    class NoiseImpl;

    class Sensor;


/* System Noise
 * Holds the current System Noise
 * resulting matrix is ordered by the add calls?
 */
    class SystemNoise {

    public:

        Eigen::MatrixXd matrix() const;

        void addNoise(const std::shared_ptr<Noise> &noise) const;

    private:

        std::shared_ptr<NoiseImpl> _impl;

    };

    class MeasurementNoise {

    public:

        Eigen::MatrixXd matrix() const;

        void addNoise(const std::shared_ptr<const Sensor> &sensor) const;

    private:
        std::shared_ptr<NoiseImpl> _impl;

    };


}