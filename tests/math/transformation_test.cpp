#include "core/math/transformations.hpp"
#include "gtest/gtest.h"

class UKFMathTest : public ::testing::Test {

public:
    UKFMathTest() : _params({.alpha=1, .beta=1, .kappa=1}), _expectedSigmaPoints() {
        _params.update(3);
        _expectedSigmaPoints = Eigen::MatrixXf::Zero(L, 2 * L + 1);
        _expectedSigmaPoints << 1, 3, 1, 1, -1, 1, 1,
                2, 2, 4, 2, 2, 0, 2,
                3, 3, 3, 5, 3, 3, 1;
    }

protected:

    ukf::core::UkfParameters _params;
    Eigen::MatrixXf _expectedSigmaPoints{};

    const std::size_t L{3};
    const Eigen::Vector3f _state = {1.0f, 2.0f, 3.0f};
    const Eigen::Matrix3f _covariance = Eigen::Matrix3f::Identity();


};


TEST_F(UKFMathTest, DrawSigmaPoints) {
    Eigen::MatrixXf actualSigmaPoints = ukf::core::math::drawSigmaPoints(_state, _covariance, _params);
    ASSERT_TRUE(_expectedSigmaPoints.isApprox(actualSigmaPoints, 0.0001))
                                << "drawSigmaPoints does not produce expected result.";
}

TEST_F(UKFMathTest, CalculateMean) {

    Eigen::Vector3f expectedMean = {1.0f, 2.0f, 3.0f};
    Eigen::Vector3f actualMean = ukf::core::math::calculateMean(_expectedSigmaPoints, _params);
    ASSERT_FLOAT_EQ(expectedMean[0], actualMean[0]);
    ASSERT_FLOAT_EQ(expectedMean[1], actualMean[1]);
    ASSERT_FLOAT_EQ(expectedMean[2], actualMean[2]);


}

TEST_F(UKFMathTest, CalculateCrossVarianceMatrix) {

    const Eigen::Matrix<float, 1, 1> z{14};
    Eigen::Matrix<float, 1, 7> gammaPoints = {14, 22, 26, 30, 14, 10, 6};

    const Eigen::Vector3f expected{2, 4, 6};
    const Eigen::MatrixXf actual = ukf::core::math::calculateCrossVarianceMatrix(_expectedSigmaPoints, _state,
                                                                                 gammaPoints, z, _params);

    ASSERT_TRUE(expected.isApprox(actual)) << "cross variance matrix does not produce expected result.";
}

TEST_F(UKFMathTest, CalculateSquareRootCovariance) {
    Eigen::Matrix3f expected = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f actual = ukf::core::math::calculateSquareRootCovariance(_expectedSigmaPoints, _state,
                                                                            Eigen::Matrix3f::Zero(), _params);

    ASSERT_TRUE(expected.isApprox(actual)) << "square root covariance does not produce expected result.";
}

TEST_F(UKFMathTest, Propagate) {
    Eigen::MatrixXf expected = 2 * _expectedSigmaPoints;
    Eigen::MatrixXf actual = Eigen::MatrixXf::Zero(L, 2 * L + 1);

    ukf::core::math::propagate(_expectedSigmaPoints, actual, [](auto value) { return 2 * value; });
    ASSERT_TRUE(expected.isApprox(actual)) << "propagate does not produce expected result.";
}


