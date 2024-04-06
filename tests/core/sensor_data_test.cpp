//
// Created by bene on 01.11.23.
//
#include "core/sensor_data.h"

#include <gtest/gtest.h>

namespace {
struct MockDataSize2 {
  float value_1{};
  float value_2{};
};

struct SensorSize2 : public ukf::core::Sensor<2, MockDataSize2> {
  using Sensor::Sensor;
  Noising noising() const override {
    return (ukf::core::SquaredMatrix<2>() << 2, 0, 0, 2).finished();
  }

  Data predict() const override { return {}; }

  ukf::core::Vector<2UL> toVector(MockDataSize2 &&data) const override {
    return {data.value_1, data.value_2};
  }
};

struct MockDataSize4 {
  float value_1{};
  float value_2{};
  float value_3{};
  float value_4{};
};

struct SensorSize4 : public ukf::core::Sensor<4, MockDataSize4> {
  using Sensor::Sensor;
  Noising noising() const override {
    return (ukf::core::SquaredMatrix<4>().diagonal() << 4.0, 4.0, 4.0, 4.0)
        .finished()
        .asDiagonal()
        .toDenseMatrix();
  }

  Data predict() const override { return {}; }

  ukf::core::Vector<4UL> toVector(MockDataSize4 &&data) const override {
    return {data.value_1, data.value_2, data.value_3, data.value_4};
  }
};

using SensorDataType =
    ukf::core::SensorData<ukf::core::StaticFields<SensorSize2, SensorSize4>>;
}  // namespace

TEST(SensorData, constructionTest) {
  using namespace ukf::core;
  SensorDataType sensorData{};

  ASSERT_EQ(sensorData.vector().size(), 0);

  sensorData.setMeasurement<SensorSize2>({.value_1 = 1.1f, .value_2 = 1.2f});
  // only contains first sensor
  {
    const ukf::core::Vector<2> expectedVector(1.1f, 1.2f);
    const ukf::core::SquaredMatrix<2> expectedMatrix =
        (ukf::core::SquaredMatrix<2>() << 2.0, 0, 0, 2.0).finished();

    ASSERT_TRUE(sensorData.vector().isApprox(expectedVector));
    ASSERT_TRUE(sensorData.noising().isApprox(expectedMatrix));
  }

  sensorData.setMeasurement<SensorSize4>({
      .value_1 = 2.1f,
      .value_2 = 2.2f,
      .value_3 = 2.3f,
      .value_4 = 2.4f,
  });
  // Add second measurement shall be reflected
  {
    const Vector<6> expected(1.1f, 1.2f, 2.1f, 2.2f, 2.3f, 2.4f);
    const SquaredMatrix<6> expectedMatrix =
        (SquaredMatrix<6>().diagonal() << 2.0, 2.0, 4.0, 4.0, 4.0, 4.0)
            .finished()
            .asDiagonal()
            .toDenseMatrix();

    ASSERT_TRUE(sensorData.vector().isApprox(expected));
    ASSERT_TRUE(sensorData.noising().isApprox(expectedMatrix));
  }

  sensorData.setMeasurement<SensorSize2>({.value_1 = 3.1f, .value_2 = 3.2f});
  // New measurement shall overwrite the previous one
  {
    const Vector<6> expected(3.1f, 3.2f, 2.1f, 2.2f, 2.3f, 2.4f);
    const SquaredMatrix<6> expectedMatrix =
        (SquaredMatrix<6>() << 2.0, 0, 0, 0, 0, 0, 0, 2.0, 0, 0, 0, 0, 0, 0,
         4.0, 0, 0, 0, 0, 0, 0, 4.0, 0, 0, 0, 0, 0, 0, 4.0, 0, 0, 0, 0, 0, 0,
         4.0)
            .finished();

    ASSERT_TRUE(sensorData.vector().isApprox(expected));
    ASSERT_TRUE(sensorData.noising().isApprox(expectedMatrix));
  }
}