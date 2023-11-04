//
// Created by bene on 04.11.23.
//

#include <gtest/gtest.h>
#include "core/covariance.hpp"
#include "core/field.hpp"


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

TEST(CovarianceTest, ConstructorTest) {
    ukf::core::Covariance<Fields> testCovariance;

    // Check if the state is initially zero
    EXPECT_TRUE((testCovariance == Eigen::Matrix3f::Zero()));

    // Assign new values to the state
    testCovariance = (Eigen::Matrix3f() <<
                                        1.f, 2.f, 3.f,
            4.f, 5.f, 6.f,
            7.f, 8.f, 9.f).finished();

    const Eigen::Matrix3f expected = (Eigen::Matrix3f() <<
                                                        1.f, 2.f, 3.f,
            4.f, 5.f, 6.f,
            7.f, 8.f, 9.f).finished();
    // Check if the new values were assigned
    EXPECT_TRUE((testCovariance == expected));
}

TEST(CovarianceTest, CopyConstructorTest) {
    // Create a state with specific values
    ukf::core::Covariance<Fields> originalCovariance(Eigen::Matrix3f::Identity());

    // Copy construct a new state from the original one
    ukf::core::Covariance<ukf::core::StateFields<MockField>> copiedCovariance(originalCovariance);

    // The copied state should equal to the original one
    EXPECT_TRUE((copiedCovariance == originalCovariance));
}

TEST(CovarianceTest, AssignmentTest) {
    // Create two states
    ukf::core::Covariance<Fields> firstCovariance(Eigen::Matrix3f::Identity());
    ukf::core::Covariance<Fields> secondCovariance;

    // Assign the first state to the second one
    secondCovariance = firstCovariance;

    // The second state should now equal the first one
    EXPECT_TRUE((firstCovariance == secondCovariance));
}


TEST(CovarianceTest, AccessTest) {
    Eigen::Matrix<float, 8, 8> cov = Eigen::Matrix<float, 8, 8>::Zero();
    cov.block<3, 3>(0, 0) = Eigen::Matrix3f::Constant(3);
    cov.block<5, 5>(3, 3) = Eigen::Matrix<float, 5, 5>::Constant(5);


    ukf::core::Covariance<ukf::core::StateFields<MockField, MockFieldSize5>> testCovariance(std::move(cov));
    // Field Access
    {
        {
            const auto field = testCovariance.get<MockField>();
            const Eigen::Matrix3f expected = Eigen::Matrix3f::Constant(3);
            ASSERT_TRUE(field.isApprox(expected));
        }

        {
            const auto field = testCovariance.get<MockFieldSize5>();
            const Eigen::Matrix<float, 5, 5> expected = Eigen::Matrix<float, 5, 5>::Constant(5);
            ASSERT_TRUE(field.isApprox(expected));
        }
    }

}
