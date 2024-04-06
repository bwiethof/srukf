//
// Created by bene on 02.11.23.
//
#include "core/state.hpp"

#include <gtest/gtest.h>

#include "core/field.hpp"

using ukf::core::State;
using ukf::core::StateFields;

namespace {
struct MockField : public ukf::core::SimpleField<3> {
  using ukf::core::SimpleField<3>::SimpleField;
  Noising noising() const override { return Noising::Identity() * 3; }

  Data timeUpdate(float dt) const override { return {dt, dt, dt}; }
};

struct MockFieldSize5 : public ukf::core::SimpleField<5> {
  using ukf::core::SimpleField<5>::SimpleField;
  Noising noising() const override { return Noising ::Identity() * 5; }

  Data timeUpdate(float dt) const override { return Data(1, 2, 3, 4, 5) * dt; }
};

using Fields = ukf::core::StateFields<MockField>;
}  // namespace

TEST(StateTest, ConstructorTest) {
  ukf::core::State<Fields> testState;

  // Check if the state is initially zero
  EXPECT_TRUE((testState == ukf::core::Vector<3>::Zero()));

  // Assign new values to the state
  testState = ukf::core::Vector<3>(1.f, 2.f, 3.f);

  // Check if the new values were assigned
  EXPECT_TRUE((testState == ukf::core::Vector<3>(1.f, 2.f, 3.f)));
}

TEST(StateTest, CopyConstructorTest) {
  // Create a state with specific values
  State<Fields> originalState(ukf::core::Vector<3>(1.f, 2.f, 3.f));

  // Copy construct a new state from the original one
  State<StateFields<MockField>> copiedState(originalState);

  // The copied state should equal to the original one
  EXPECT_TRUE((copiedState == originalState));
}

TEST(StateTest, AssignmentTest) {
  // Create two states
  State<Fields> firstState(ukf::core::Vector<3>(1.f, 2.f, 3.f));
  State<Fields> secondState;

  // Assign the first state to the second one
  secondState = firstState;

  // The second state should now equal the first one
  EXPECT_TRUE((firstState == secondState));
}

TEST(StateTest, TimeUpdateTest) {
  // Create a state with specific values
  State<StateFields<MockField, MockFieldSize5>> testState(
      ukf::core::Vector<8>(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f));

  // Apply a time update
  auto updatedState = testState.f(0.1f);
  const ukf::core::Vector<8> expected(0.1f, 0.1f, 0.1f, 0.1f, 0.2f, 0.3f, 0.4f,
                                      0.5f);

  // The state should have changed
  EXPECT_TRUE(updatedState.isApprox(expected, 1e-4));
}

TEST(StateTest, AccessTest) {
  // Create a state with specific values
  State<StateFields<MockField, MockFieldSize5>> testState(
      ukf::core::Vector<8>(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f));
  // Field Access
  {
    {
      const auto field = testState.get<MockField>();
      const ukf::core::Vector<3> expected = {1.f, 2.f, 3.f};
      ASSERT_TRUE(field.data.isApprox(expected));
    }

    {
      const auto field = testState.get<MockFieldSize5>();
      const ukf::core::Vector<5> expected = {4.f, 5.f, 6.f, 7.f, 8.f};
      ASSERT_TRUE(field.data.isApprox(expected));
    }
  }
  // Correct noising
  {
    ukf::core::SquaredMatrix<8> expected;
    expected.setIdentity();
    expected.block<3, 3>(0, 0) *= 3;
    expected.block<5, 5>(3, 3) *= 5;

    ASSERT_TRUE(testState.noising().isApprox(expected));
  }
}
