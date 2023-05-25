//
// Created by bene on 25.05.23.
//
#include <gtest/gtest.h>
#include "slam/sensor.hpp"
#include "slam/sensor_data.h"
#include "mock/sensor.h"

namespace ukf::test::slam {

    /**
     * GIVEN a set of Sensors registered at slam SensorData
     *  WHEN Sensor data with id is provided
     *      THEN each Sensor set has it's own id set
     *      THEN each sensor stores the order of inserted sensor data
     *
     */
    TEST(slamsensor_data, sensor_data_test) {

        using ukf::slam::NoOpSensor;
        using ukf::test::mock::SlamSensor;
        using ukf::test::mock::Data;
        using TestSensorData = ukf::slam::SensorData<NoOpSensor, SlamSensor>;
        TestSensorData sensorData;
        NoOp_t data;
        sensorData.setMeasurement(data, 0);
        sensorData.setMeasurement(data, 2);

        Data mockData = ukf::test::mock::values[0];


        sensorData.setMeasurement(mockData, 1);
        sensorData.setMeasurement(mockData, 4);
        sensorData.setMeasurement(mockData, 2);
        sensorData.setMeasurement(mockData, 5);
        sensorData.setMeasurement(mockData, 3);

        EXPECT_EQ(sensorData.getOrderedIds<SlamSensor>(), std::vector<std::size_t>({1, 4, 2, 5, 3}));
        EXPECT_EQ(sensorData.getOrderedIds<NoOpSensor>(), std::vector<std::size_t>({0, 2}));
    }
} // ukf::test::slam
