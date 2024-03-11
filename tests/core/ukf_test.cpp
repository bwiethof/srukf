//
// Created by bene on 04.11.23.
//

#include "core/ukf.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace {
namespace statetest {

struct MockFieldModel;
struct MockFieldModelSize5;

struct MockField : public ukf::core::SimpleField<3> {
  using ukf::core::SimpleField<3>::SimpleField;
  Eigen::Matrix<float, 3UL, 3UL> noising() const override {
    return Eigen::Matrix3f::Identity() * 3;
  }

  Eigen::Vector<float, 3UL> timeUpdate(float dt) const override {
    return {dt, dt, dt};
  }
};

struct MockFieldSize5 : public ukf::core::SimpleField<5> {
  using ukf::core::SimpleField<5>::SimpleField;
  Eigen::Matrix<float, 5UL, 5UL> noising() const override {
    return Eigen::Matrix<float, 5, 5>::Identity() * 5;
  }

  Eigen::Vector<float, 5UL> timeUpdate(float dt) const override {
    return Eigen::Vector<float, 5>(2, 2, 2, 2, 2) * dt;
  }
};

using Fields = ukf::core::StateFields<MockField, MockFieldSize5>;
}  // namespace statetest

namespace sensortest {

struct MockDataSize2 {
  float value_1{};
  float value_2{};
};

struct SensorSize2
    : public ukf::core::SensorModel<2, MockDataSize2, statetest::MockField> {
  using SensorModel::SensorModel;
  Eigen::Matrix<float, 2UL, 2UL> noising() const override {
    return Eigen::DiagonalMatrix<float, 2>(2, 2).toDenseMatrix();
  }

  Eigen::Vector<float, 2UL> predict(
      const statetest::MockField &field) const override {
    return field.data.segment<2>(0);
  }

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

struct SensorSize4 : public ukf::core::SensorModel<4, MockDataSize4,
                                                   statetest::MockFieldSize5> {
  using SensorModel::SensorModel;
  Eigen::Matrix<float, 4UL, 4UL> noising() const override {
    return Eigen::DiagonalMatrix<float, 4>(4, 4, 4, 4).toDenseMatrix();
  }

  Eigen::Vector<float, 4UL> predict(
      const statetest::MockFieldSize5 &field) const override {
    return field.data.segment<4>(0);
  }

  Eigen::Vector<float, 4UL> toVector(MockDataSize4 &&data) const override {
    return {data.value_1, data.value_2, data.value_3, data.value_4};
  }
};

using SensorDataType =
    ukf::core::SensorData<ukf::core::StaticFields<SensorSize2, SensorSize4>>;
}  // namespace sensortest
class UkfRef : public ukf::core::detail::Ukf {
 public:
  using ukf::core::detail::Ukf::timeStep;

  Eigen::MatrixXf getSystemNoise() const override { return _nosing; }

  template <typename Derived>
  void setNoising(const Eigen::MatrixBase<Derived> &otherNoising) {
    _nosing = otherNoising;
  }

 private:
  Eigen::MatrixXf _nosing{};
};
}  // namespace

TEST(Ukf, Initialization_test) {
  // Zero State
  {
    ukf::core::Ukf<statetest::Fields> testUkf;
    ASSERT_EQ(testUkf.getState(), Eigen::VectorXf::Zero(8));
    ASSERT_EQ(testUkf.getCovariance(), Eigen::MatrixXf::Zero(8, 8));
  }

  const auto checkValues = [](const ukf::core::Ukf<statetest::Fields> &ukf) {
    const Eigen::Vector<float, 8> expectedState{1.f, 2.f, 3.f, 4.f,
                                                5.f, 6.f, 7.f, 8.f};
    Eigen::Matrix<float, 8, 8> expectedCovariance;
    expectedCovariance.setZero();
    expectedCovariance.block<3, 3>(0, 0).diagonal().setConstant(3);
    expectedCovariance.block<5, 5>(3, 3).diagonal().setConstant(5);

    ASSERT_EQ(ukf.getState(), expectedState);
    ASSERT_EQ(ukf.getCovariance(), expectedCovariance);
  };

  const auto prepareCovariance = []() {
    Eigen::Matrix<float, 8, 8> covariance;
    covariance.setZero();

    covariance.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * 3;
    covariance.block<5, 5>(3, 3) = Eigen::Matrix<float, 5, 5>::Identity() * 5;

    return covariance;
  };
  // initialize via reference State
  {
    ukf::core::State<statetest::Fields> state =
        Eigen::Vector<float, 8>{1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f};
    ukf::core::Covariance<statetest::Fields> covariance = prepareCovariance();
    ukf::core::Ukf<statetest::Fields> testUkf(state, covariance);

    checkValues(testUkf);
  }
  // initialize via move State
  {
    ukf::core::State<statetest::Fields> state =
        Eigen::Vector<float, 8>{1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f};
    ukf::core::Covariance<statetest::Fields> covariance = prepareCovariance();
    ;
    ukf::core::Ukf<statetest::Fields> testUkf(std::move(state),
                                              std::move(covariance));

    checkValues(testUkf);
  }
  // initialize via reference matrix
  {
    Eigen::Vector<float, 8> state =
        Eigen::Vector<float, 8>{1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f};
    Eigen::Matrix<float, 8, 8> covariance = prepareCovariance();
    ukf::core::Ukf<statetest::Fields> testUkf(state, covariance);

    checkValues(testUkf);
  }
  // initialize via move matrix
  {
    Eigen::Vector<float, 8> state =
        Eigen::Vector<float, 8>{1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f};
    Eigen::Matrix<float, 8, 8> covariance = prepareCovariance();
    ukf::core::Ukf<statetest::Fields> testUkf(std::move(state),
                                              std::move(covariance));

    checkValues(testUkf);
  }
}

TEST(Ukf, Stepping_test) {
  const ukf::core::State<statetest::Fields> initialState =
      Eigen::Vector<float, 8>{1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.7f, 1.8f};
  const ukf::core::Covariance<statetest::Fields> initialCovariance =
      Eigen::Matrix<float, 8, 8>::Identity();

  ukf::core::Ukf<statetest::Fields> testUkf{initialState, initialCovariance};
  UkfRef refUkf{};
  refUkf.setNoising(initialState.noising());

  // Initialize sensorData and dt
  sensortest::SensorDataType sensorData{};

  sensorData.setMeasurement<sensortest::SensorSize2>(
      {.value_1 = 1.f, .value_2 = 2.f});
  double dt = 1.0;

  const auto [expectedState, expectedCovariance] =
      refUkf.timeStep(initialState, initialCovariance, dt, sensorData);

  // Perform a step
  testUkf.step(sensorData, dt);

  auto state = testUkf.getState();
  auto covariance = testUkf.getCovariance();

  ASSERT_TRUE(state.isApprox(expectedState));
  ASSERT_TRUE(covariance.isApprox(expectedCovariance));
}