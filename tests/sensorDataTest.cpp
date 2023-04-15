//
// Created by bene on 15.04.23.
//

#include "sensor_data.hpp"
#include "testUtilities.hpp"
#include <gtest/gtest.h>

TEST(SensorData, ReadWrite) {

  ukf::SensorData sensorData;
  const auto mockSensor = std::make_shared<const ukf::test::SensorMock>();
  sensorData.add(mockSensor);

  EXPECT_TRUE(sensorData.hasMeasurement<ukf::test::SensorMock>());
  EXPECT_EQ(sensorData.measurement<ukf::test::SensorMock>(),
            mockSensor->measurement());
}