//
// Created by bene on 02.11.23.
//
#include <gtest/gtest.h>
#include "core/state.hpp"
#include "core/field.hpp"

using ukf::core::State;
using ukf::core::StateFields;

namespace {
    struct MockFieldModel : public ukf::core::Model<3> {
        Eigen::Matrix<float, 3UL, 3UL> noising() const override {
            return Eigen::Matrix3f::Identity() * 3;
        }

        Eigen::Vector<float, 3UL> timeUpdate(float dt) const override {
            return {dt, dt, dt};
        }
    };


    struct MockFieldModelSize5 : public ukf::core::Model<5> {
        Eigen::Matrix<float, 5UL, 5UL> noising() const override {
            return Eigen::Matrix<float, 5, 5>::Identity() * 5;
        }

        Eigen::Vector<float, 5UL> timeUpdate(float dt) const override {
            return Eigen::Vector<float, 5>(1, 2, 3, 4, 5) * dt;
        }
    };

    using MockField = ukf::core::Field<3, MockFieldModel>;
    using MockFieldSize5 = ukf::core::Field<5, MockFieldModelSize5>;
    using Fields = ukf::core::StateFields<MockField>;
}

TEST(StateTest, ConstructorTest) {
    ukf::core::State<Fields> testState;

    // Check if the state is initially zero
    EXPECT_TRUE((testState == Eigen::Vector3f::Zero()));

    // Assign new values to the state
    testState = Eigen::Vector3f(1.f, 2.f, 3.f);

    // Check if the new values were assigned
    EXPECT_TRUE((testState == Eigen::Vector3f(1.f, 2.f, 3.f)));
}

TEST(StateTest, CopyConstructorTest) {
    // Create a state with specific values
    State<Fields> originalState(Eigen::Vector3f(1.f, 2.f, 3.f));

    // Copy construct a new state from the original one
    State<StateFields<MockField>> copiedState(originalState);

    // The copied state should equal to the original one
    EXPECT_TRUE((copiedState == originalState));
}

TEST(StateTest, AssignmentTest) {
    // Create two states
    State<Fields> firstState(Eigen::Vector3f(1.f, 2.f, 3.f));
    State<Fields> secondState;

    // Assign the first state to the second one
    secondState = firstState;

    // The second state should now equal the first one
    EXPECT_TRUE((firstState == secondState));
}

TEST(StateTest, TimeUpdateTest) {
    // Create a state with specific values
    State<StateFields<MockField, MockFieldSize5>> testState(
            Eigen::Vector<float, 8>(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f));

    // Apply a time update
    auto updatedState = testState.f(0.1f);
    const Eigen::Vector<float, 8> expected(0.1f, 0.1f, 0.1f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f);

    // The state should have changed
    EXPECT_TRUE(updatedState.isApprox(expected));
}

TEST(StateTest, AccessTest) {
    // Create a state with specific values
    State<StateFields<MockField, MockFieldSize5>> testState(
            Eigen::Vector<float, 8>(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f));
    // Field Access
    {
        {
            const auto field = testState.get<MockField>();
            const Eigen::Vector3f expected = {1.f, 2.f, 3.f};
            ASSERT_TRUE(field.data.isApprox(expected));
        }

        {
            const auto field = testState.get<MockFieldSize5>();
            const Eigen::Vector<float, 5> expected = {4.f, 5.f, 6.f, 7.f, 8.f};
            ASSERT_TRUE(field.data.isApprox(expected));
        }
    }
    // Correct noising
    {
        Eigen::Matrix<float, 8, 8> expected;
        expected.setIdentity();
        expected.block<3, 3>(0, 0) *= 3;
        expected.block<5, 5>(3, 3) *= 5;

        ASSERT_TRUE(testState.noising().isApprox(expected));
    }

}
