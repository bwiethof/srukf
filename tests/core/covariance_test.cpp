//
// Created by bene on 04.11.23.
//

#include "core/covariance.hpp"

#include <gtest/gtest.h>

#include "core/field.hpp"

namespace {
struct MockField : public ukf::core::SimpleField<3> {
  using ukf::core::SimpleField<3>::SimpleField;
  Noising noising() const override { return Noising::Identity() * 3; }

  Data timeUpdate(float dt) const override { return {dt, dt, dt}; }
};

struct MockFieldSize5 : public ukf::core::SimpleField<5> {
  using ukf::core::SimpleField<5>::SimpleField;
  Noising noising() const override { return Noising::Identity() * 5; }

  Data timeUpdate(float dt) const override { return Data(1, 2, 3, 4, 5) * dt; }
};

using Fields = ukf::core::StateFields<MockField>;
}  // namespace

TEST(CovarianceTest, ConstructorTest) {
  ukf::core::Covariance<Fields> testCovariance;
  using namespace ukf::core;

  // Check if the state is initially zero
  EXPECT_TRUE((testCovariance == ukf::core::SquaredMatrix<3>::Zero()));

  // Assign new values to the state
  testCovariance =
      (SquaredMatrix<3>() << 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f)
          .finished();

  const SquaredMatrix<3> expected =
      (SquaredMatrix<3>() << 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f)
          .finished();
  // Check if the new values were assigned
  EXPECT_TRUE((testCovariance == expected));
}

TEST(CovarianceTest, CopyConstructorTest) {
  using namespace ukf::core;
  // Create a state with specific values
  ukf::core::Covariance<Fields> originalCovariance(
      SquaredMatrix<3>::Identity());

  // Copy construct a new state from the original one
  ukf::core::Covariance<ukf::core::StateFields<MockField>> copiedCovariance(
      originalCovariance);

  // The copied state should equal to the original one
  EXPECT_TRUE((copiedCovariance == originalCovariance));
}

TEST(CovarianceTest, AssignmentTest) {
  using namespace ukf::core;
  // Create a state with specific values
  // Create two states
  ukf::core::Covariance<Fields> firstCovariance(SquaredMatrix<3>::Identity());
  ukf::core::Covariance<Fields> secondCovariance;

  // Assign the first state to the second one
  secondCovariance = firstCovariance;

  // The second state should now equal the first one
  EXPECT_TRUE((firstCovariance == secondCovariance));
}

TEST(CovarianceTest, AccessTest) {
  using namespace ukf::core;
  // Create a state with specific values
  SquaredMatrix<8> cov = SquaredMatrix<8>::Zero();
  cov.block<3, 3>(0, 0) = SquaredMatrix<3>::Constant(3);
  cov.block<5, 5>(3, 3) = SquaredMatrix<5>::Constant(5);

  ukf::core::Covariance<ukf::core::StateFields<MockField, MockFieldSize5>>
      testCovariance(std::move(cov));
  // Field Access
  {
    {
      const auto field = testCovariance.get<MockField>();
      const SquaredMatrix<3> expected = SquaredMatrix<3>::Constant(3);
      ASSERT_TRUE(field.isApprox(expected));
    }

    {
      const auto field = testCovariance.get<MockFieldSize5>();
      const SquaredMatrix<5> expected = SquaredMatrix<5>::Constant(5);
      ASSERT_TRUE(field.isApprox(expected));
    }
  }
}
