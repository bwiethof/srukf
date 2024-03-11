// Include GTest
#include <gmock/gmock.h>
#include <gtest/gtest.h>
// Include the header that contains the class to test
#include "core/covariance.hpp"
#include "core/detail/ukf_impl.hpp"
#include "core/parameters.hpp"
#include "core/state.hpp"

using ::testing::AtLeast;
using ::testing::Return;

class UKFMock : public ukf::core::detail::Ukf {
 public:
  explicit UKFMock(ukf::core::UkfParameters &&parameters)
      : ukf::core::detail::Ukf(std::move(parameters)) {}

  using ukf::core::detail::Ukf::timeStep;

  MOCK_METHOD(Eigen::MatrixXf, getSystemNoise, (), (const));
};

struct MockField1 : public ukf::core::SimpleField<3> {
  using ukf::core::SimpleField<3>::SimpleField;
  Eigen::Matrix<float, 3UL, 3UL> noising() const override {
    return Eigen::Matrix3f::Identity();
  }

  Eigen::Vector<float, 3UL> timeUpdate(float) const override {
    return Eigen::Vector3f{1, 2, 3};
  }
};

// TODO: remove FieldSize  by using MockModel Size

struct DataType {};

struct MockSensor : public ukf::core::Sensor<2, DataType, MockField1> {
  using Sensor::Sensor;
  Eigen::Matrix<float, 2UL, 2UL> noising() const override {
    return Eigen::Matrix2f::Identity();
  }

  Eigen::Vector<float, 2UL> toVector(DataType &&) const override {
    return {3, 3};
  }

  Eigen::Vector<float, 2UL> predict(const MockField1 &field) const override {
    return {field.data[0] + field.data[1], field.data[2]};
  }
};

// Test prediction part of Ukf.timeStep method
TEST(UkfTest, TimeStep) {
  // Arrange
  // Create an instance of your class

  using StateFieldsType = ukf::core::StateFields<MockField1>;
  using StateType = ukf::core::State<StateFieldsType>;
  using CovarianceType = ukf::core::Covariance<StateFieldsType>;

  UKFMock myUkf(ukf::core::UkfParameters({1, 0, 0}));

  const Eigen::MatrixXf systemNoising = Eigen::MatrixXf::Identity(3, 3);
  EXPECT_CALL(myUkf, getSystemNoise())
      .Times(AtLeast(1))
      .WillRepeatedly(Return(systemNoising));

  const CovarianceType P(Eigen::MatrixXf::Identity(3, 3));
  const StateType X(Eigen::Vector3f(1, 2, 3));

  {
    const double dt = 1.0;
    const ukf::core::SensorData<ukf::core::StaticFields<>> sensorData;

    // Act
    const auto [resultState, resultCovariance] =
        myUkf.timeStep(X, P, dt, sensorData);

    // only system noise shall be applied
    const Eigen::Matrix3f expectedCovariance = Eigen::Matrix3f::Identity();
    const Eigen::Vector3f expectedState = Eigen::Vector3f{1, 2, 3};

    ASSERT_TRUE(resultCovariance.isApprox(expectedCovariance));
    ASSERT_TRUE(resultState.isApprox(expectedState));
  }

  {
    const double dt = 1.0;

    ukf::core::SensorData<ukf::core::StaticFields<MockSensor>> sensorData;
    sensorData.setMeasurement<MockSensor>(DataType{});

    Eigen::Matrix3f expectedCovariance;
    expectedCovariance << 0.8164966, 0, 0, -0.408248276, 0.707106769, 0, 0, 0,
        0.707106888;

    const Eigen::Vector3f expectedState = Eigen::Vector3f{1, 2, 3};

    const auto [resultState, resultCovariance] =
        myUkf.timeStep(X, P, dt, sensorData);
    std::cout << resultState << "\n" << resultCovariance << "\n";

    ASSERT_TRUE(resultCovariance.isApprox(expectedCovariance));
    ASSERT_TRUE(resultState.isApprox(expectedState));
  }
}