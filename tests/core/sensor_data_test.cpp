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

struct SensorModelSize2 : public ukf::core::SensorModel<2, MockDataSize2> {
  Eigen::Matrix<float, 2UL, 2UL> noising() const override {
    return Eigen::DiagonalMatrix<float, 2>(2, 2).toDenseMatrix();
  }

  Eigen::Vector<float, 2UL> predict() const override { return {}; }

  Eigen::Vector<float, 2UL> toVector(MockDataSize2 &&data) const override {
    return {data.value_1, data.value_2};
  }
};

struct MockDataSize4 {
  float value_1{};
  float value_2{};
  float value_3{};
  float value_4{};
};

struct SensorModelSize4 : public ukf::core::SensorModel<4, MockDataSize4> {
  Eigen::Matrix<float, 4UL, 4UL> noising() const override {
    return Eigen::DiagonalMatrix<float, 4>(4, 4, 4, 4).toDenseMatrix();
  }

  Eigen::Vector<float, 4UL> predict() const override { return {}; }

  Eigen::Vector<float, 4UL> toVector(MockDataSize4 &&data) const override {
    return {data.value_1, data.value_2, data.value_3, data.value_4};
  }
};

using SensorSize2 = ukf::core::Field<SensorModelSize2>;
using SensorSize4 = ukf::core::Field<SensorModelSize4>;

using SensorDataType =
    ukf::core::SensorData<ukf::core::StaticFields<SensorSize2, SensorSize4>>;
}  // namespace

TEST(SensorData, constructionTest) {
  SensorDataType sensorData{};

  ASSERT_EQ(sensorData.vector().size(), 0);

  sensorData.setMeasurement<SensorSize2>({.value_1 = 1.1f, .value_2 = 1.2f});
  // only contains first sensor
  {
    const Eigen::Vector2f expectedVector(1.1f, 1.2f);
    const Eigen::Matrix2f expectedMatrix =
        (Eigen::Matrix2f() << 2.0, 0, 0, 2.0).finished();

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
    const Eigen::Vector<float, 6> expected(1.1f, 1.2f, 2.1f, 2.2f, 2.3f, 2.4f);
    const Eigen::Matrix<float, 6, 6> expectedMatrix =
        (Eigen::Matrix<float, 6, 6>() << 2.0, 0, 0, 0, 0, 0, 0, 2.0, 0, 0, 0, 0,
         0, 0, 4.0, 0, 0, 0, 0, 0, 0, 4.0, 0, 0, 0, 0, 0, 0, 4.0, 0, 0, 0, 0, 0,
         0, 4.0)
            .finished();

    ASSERT_TRUE(sensorData.vector().isApprox(expected));
    ASSERT_TRUE(sensorData.noising().isApprox(expectedMatrix));
  }

  sensorData.setMeasurement<SensorSize2>({.value_1 = 3.1f, .value_2 = 3.2f});
  // New measurement shall overwrite the previous one
  {
    const Eigen::Vector<float, 6> expected(3.1f, 3.2f, 2.1f, 2.2f, 2.3f, 2.4f);
    const Eigen::Matrix<float, 6, 6> expectedMatrix =
        (Eigen::Matrix<float, 6, 6>() << 2.0, 0, 0, 0, 0, 0, 0, 2.0, 0, 0, 0, 0,
         0, 0, 4.0, 0, 0, 0, 0, 0, 0, 4.0, 0, 0, 0, 0, 0, 0, 4.0, 0, 0, 0, 0, 0,
         0, 4.0)
            .finished();

    ASSERT_TRUE(sensorData.vector().isApprox(expected));
    ASSERT_TRUE(sensorData.noising().isApprox(expectedMatrix));
  }
}