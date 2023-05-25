//
// Created by bene on 25.05.23.
//
#include <gtest/gtest.h>
#include "include/base/sensor_data.h"
#include "base/base_sensor.hpp"
#include "mock/sensor.h"

namespace ukf::test::base {

    TEST(sensor_data, sensor_data_test) {
        using ukf::test::mock::BaseSensor;
        using ukf::test::mock::Data;
        using TestSensorData = ukf::StaticSensorData<ukf::NoOpSensor, BaseSensor>;
        TestSensorData sensorData;
        NoOp_t data;
        sensorData.setMeasurement(data);

        Data mockData = ukf::test::mock::values[0];
        sensorData.setMeasurement(mockData);
    }
}
