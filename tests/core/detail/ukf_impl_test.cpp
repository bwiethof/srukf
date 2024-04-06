// Include GTest
#include <gmock/gmock.h>
#include <gtest/gtest.h>
// Include the header that contains the class to test
#include "core/covariance.hpp"
#include "core/detail/ukf_impl.hpp"
#include "core/parameters.hpp"
#include "core/sensor.hpp"
#include "core/sensor_data.h"
#include "core/state.hpp"

using ::testing::AtLeast;
using ::testing::Return;

class UKFMock : public ukf::core::detail::Ukf {
 public:
  explicit UKFMock(ukf::core::UkfParameters &&parameters)
      : ukf::core::detail::Ukf(std::move(parameters)) {}

  using ukf::core::detail::Ukf::timeStep;

  MOCK_METHOD(ukf::core::state::NoisingType, getSystemNoise, (), (const));
};

struct MockField1 : public ukf::core::SimpleField<3> {
  using ukf::core::SimpleField<3>::SimpleField;
  Noising noising() const override { return Noising ::Identity(); }

  Data timeUpdate(float) const override { return Data{1, 2, 3}; }
};

// TODO: remove FieldSize  by using MockModel Size

struct DataType {};

struct MockSensor : public ukf::core::Sensor<2, DataType, MockField1> {
  using Sensor::Sensor;
  Noising noising() const override { return Noising::Identity(); }

  Data toVector(DataType &&) const override { return {3, 3}; }

  ukf::core::Vector<2UL> predict(const MockField1 &field) const override {
    return {field.data[0] + field.data[1], field.data[2]};
  }
};

// Test prediction part of Ukf.timeStep method
TEST(UkfTest, TimeStep) {
  using ukf::core::SquaredMatrix;
  using ukf::core::Vector;
  using ukf::core::state::CovarianceBase;
  // Arrange
  // Create an instance of your class

  using StateFieldsType = ukf::core::StateFields<MockField1>;
  using StateType = ukf::core::State<StateFieldsType>;
  using CovarianceType = ukf::core::Covariance<StateFieldsType>;

  UKFMock myUkf(ukf::core::UkfParameters({1, 0, 0}));

  const ukf::core::state::NoisingType systemNoising =
      ukf::core::state::NoisingType::Identity(3, 3);
  EXPECT_CALL(myUkf, getSystemNoise())
      .Times(AtLeast(1))
      .WillRepeatedly(Return(systemNoising));

  const CovarianceType P(CovarianceBase::Identity(3, 3));
  const StateType X(Vector<3>(1, 2, 3));

  {
    const double dt = 1.0;
    const ukf::core::SensorData<ukf::core::StaticFields<>> sensorData;

    // Act
    const auto [resultState, resultCovariance] =
        myUkf.timeStep(X, P, dt, sensorData);

    // only system noise shall be applied
    const SquaredMatrix<3> expectedCovariance = SquaredMatrix<3>::Identity();
    const ukf::core::Vector<3> expectedState = ukf::core::Vector<3>{1, 2, 3};

    ASSERT_TRUE(resultCovariance.isApprox(expectedCovariance));
    ASSERT_TRUE(resultState.isApprox(expectedState));
  }

  {
    const double dt = 1.0;

    ukf::core::SensorData<ukf::core::StaticFields<MockSensor>> sensorData;
    sensorData.setMeasurement<MockSensor>(DataType{});

    ukf::core::SquaredMatrix<3> expectedCovariance;
    expectedCovariance << 0.81649658092772592, 0, 0, -0.40824829046386307,
        0.70710678118654746, 0, 0, 0, 0.70710678118654746;

    const ukf::core::Vector<3> expectedState = ukf::core::Vector<3>{1, 2, 3};

    const auto [resultState, resultCovariance] =
        myUkf.timeStep(X, P, dt, sensorData);
    std::cout << resultState << "\n" << resultCovariance << "\n";

    ASSERT_TRUE(resultCovariance.isApprox(expectedCovariance));
    ASSERT_TRUE(resultState.isApprox(expectedState));
  }
}