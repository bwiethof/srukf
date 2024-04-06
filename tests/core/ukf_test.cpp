//
// Created by bene on 04.11.23.
//

#include "core/ukf.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace {
namespace statetest {

struct MockField : public ukf::core::SimpleField<3> {
  using ukf::core::SimpleField<3>::SimpleField;
  Noising noising() const override { return Noising::Identity() * 3; }

  Data timeUpdate(float dt) const override { return {dt, dt, dt}; }
};

struct MockFieldSize5 : public ukf::core::SimpleField<5> {
  using ukf::core::SimpleField<5>::SimpleField;
  Noising noising() const override { return Noising::Identity() * 5; }

  Data timeUpdate(float dt) const override { return Data(2, 2, 2, 2, 2) * dt; }
};

using Fields = ukf::core::StateFields<MockField, MockFieldSize5>;
}  // namespace statetest

namespace sensortest {

struct MockDataSize2 {
  float value_1{};
  float value_2{};
};

struct SensorSize2
    : public ukf::core::Sensor<2, MockDataSize2, statetest::MockField> {
  using Sensor::Sensor;
  Noising noising() const override {
    return (Noising() << 2.f, 0.f, 0.f, 2.f).finished();
  }

  Data predict(const statetest::MockField &field) const override {
    return field.data.segment<2>(0);
  }

  ukf::core::Vector<2> toVector(MockDataSize2 &&data) const override {
    return {data.value_1, data.value_2};
  }
};

struct MockDataSize4 {
  float value_1{};
  float value_2{};
  float value_3{};
  float value_4{};
};

struct SensorSize4
    : public ukf::core::Sensor<4, MockDataSize4, statetest::MockFieldSize5> {
  using Sensor::Sensor;
  Noising noising() const override {
    return (Noising() << 4, 0, 0, 0, 0, 4, 0, 0, 0, 0, 4, 0, 0, 0, 0, 4)
        .finished();
  }

  Data predict(const statetest::MockFieldSize5 &field) const override {
    return field.data.segment<4>(0);
  }

  ukf::core::Vector<4> toVector(MockDataSize4 &&data) const override {
    return {data.value_1, data.value_2, data.value_3, data.value_4};
  }
};

using SensorDataType =
    ukf::core::SensorData<ukf::core::StaticFields<SensorSize2, SensorSize4>>;
}  // namespace sensortest
class UkfRef : public ukf::core::detail::Ukf {
 public:
  using ukf::core::detail::Ukf::timeStep;

  ukf::core::state::NoisingType getSystemNoise() const override {
    return _nosing;
  }

  template <typename Derived>
  void setNoising(const Eigen::MatrixBase<Derived> &otherNoising) {
    _nosing = otherNoising;
  }

 private:
  ukf::core::state::NoisingType _nosing{};
};
}  // namespace

TEST(Ukf, Initialization_test) {
  // Zero State
  {
    ukf::core::Ukf<statetest::Fields> testUkf;
    ASSERT_EQ(testUkf.getState(), ukf::core::Vector<8>::Zero());
    ASSERT_EQ(testUkf.getCovariance(), ukf::core::SquaredMatrix<8>::Zero());
  }

  const auto checkValues = [](const ukf::core::Ukf<statetest::Fields> &ukf) {
    const ukf::core::Vector<8> expectedState{1.f, 2.f, 3.f, 4.f,
                                             5.f, 6.f, 7.f, 8.f};
    ukf::core::SquaredMatrix<8> expectedCovariance;
    expectedCovariance.setZero();
    expectedCovariance.block<3, 3>(0, 0).diagonal().setConstant(3);
    expectedCovariance.block<5, 5>(3, 3).diagonal().setConstant(5);

    ASSERT_EQ(ukf.getState(), expectedState);
    ASSERT_EQ(ukf.getCovariance(), expectedCovariance);
  };

  const auto prepareCovariance = []() {
    ukf::core::SquaredMatrix<8> covariance;
    covariance.setZero();

    covariance.block<3, 3>(0, 0) = ukf::core::SquaredMatrix<3>::Identity() * 3;
    covariance.block<5, 5>(3, 3) = ukf::core::SquaredMatrix<5>::Identity() * 5;

    return covariance;
  };
  // initialize via reference State
  {
    ukf::core::State<statetest::Fields> state =
        ukf::core::Vector<8>{1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f};
    ukf::core::Covariance<statetest::Fields> covariance = prepareCovariance();
    ukf::core::Ukf<statetest::Fields> testUkf(state, covariance);

    checkValues(testUkf);
  }
  // initialize via move State
  {
    ukf::core::State<statetest::Fields> state =
        ukf::core::Vector<8>{1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f};
    ukf::core::Covariance<statetest::Fields> covariance = prepareCovariance();
    ;
    ukf::core::Ukf<statetest::Fields> testUkf(std::move(state),
                                              std::move(covariance));

    checkValues(testUkf);
  }
  // initialize via reference matrix
  {
    ukf::core::Vector<8> state =
        ukf::core::Vector<8>{1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f};
    ukf::core::SquaredMatrix<8> covariance = prepareCovariance();
    ukf::core::Ukf<statetest::Fields> testUkf(state, covariance);

    checkValues(testUkf);
  }
  // initialize via move matrix
  {
    ukf::core::Vector<8> state =
        ukf::core::Vector<8>{1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f};
    ukf::core::SquaredMatrix<8> covariance = prepareCovariance();
    ukf::core::Ukf<statetest::Fields> testUkf(std::move(state),
                                              std::move(covariance));

    checkValues(testUkf);
  }
}

TEST(Ukf, Stepping_test) {
  const ukf::core::State<statetest::Fields> initialState =
      ukf::core::Vector<8>{1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.7f, 1.8f};
  const ukf::core::Covariance<statetest::Fields> initialCovariance =
      ukf::core::SquaredMatrix<8>::Identity();

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