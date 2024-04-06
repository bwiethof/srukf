#include "core/math/transformations.hpp"
#include "gtest/gtest.h"

class UKFMathTest : public ::testing::Test {
 public:
  UKFMathTest()
      : _params({.alpha = 1, .beta = 1, .kappa = 1}), _expectedSigmaPoints() {
    _params.update(3);
    _expectedSigmaPoints = ukf::core::math::SigmaPoints::Zero(L, 2 * L + 1);
    _expectedSigmaPoints << 1, 3, 1, 1, -1, 1, 1, 2, 2, 4, 2, 2, 0, 2, 3, 3, 3,
        5, 3, 3, 1;
  }

 protected:
  ukf::core::UkfParameters _params;
  ukf::core::math::SigmaPoints _expectedSigmaPoints{};

  const std::size_t L{3};
  const ukf::core::Vector<3> _state = {1.0f, 2.0f, 3.0f};
  const ukf::core::SquaredMatrix<3> _covariance =
      ukf::core::SquaredMatrix<3>::Identity();
};

TEST_F(UKFMathTest, DrawSigmaPoints) {
  ukf::core::math::SigmaPoints actualSigmaPoints =
      ukf::core::math::drawSigmaPoints(_state, _covariance, _params);
  ASSERT_TRUE(_expectedSigmaPoints.isApprox(actualSigmaPoints, 0.0001))
      << "drawSigmaPoints does not produce expected result.";
}

TEST_F(UKFMathTest, CalculateMean) {
  ukf::core::Vector<3> expectedMean = {1.0f, 2.0f, 3.0f};
  ukf::core::Vector<3> actualMean =
      ukf::core::math::calculateMean(_expectedSigmaPoints, _params);
  ASSERT_FLOAT_EQ(expectedMean[0], actualMean[0]);
  ASSERT_FLOAT_EQ(expectedMean[1], actualMean[1]);
  ASSERT_FLOAT_EQ(expectedMean[2], actualMean[2]);
}

TEST_F(UKFMathTest, CalculateCrossVarianceMatrix) {
  const ukf::core::Matrix<1, 1> z{14};
  ukf::core::Matrix<1, 7> gammaPoints = {14, 22, 26, 30, 14, 10, 6};

  const ukf::core::Vector<3> expected{2, 4, 6};
  const ukf::core::math::SigmaPoints actual =
      ukf::core::math::calculateCrossVarianceMatrix(
          _expectedSigmaPoints, _state, gammaPoints, z, _params);

  ASSERT_TRUE(expected.isApprox(actual))
      << "cross variance matrix does not produce expected result.";
}

TEST_F(UKFMathTest, CalculateSquareRootCovariance) {
  ukf::core::SquaredMatrix<3> expected =
      ukf::core::SquaredMatrix<3>::Identity();
  ukf::core::SquaredMatrix<3> actual =
      ukf::core::math::calculateSquareRootCovariance(
          _expectedSigmaPoints, _state, ukf::core::SquaredMatrix<3>::Zero(),
          _params);

  ASSERT_TRUE(expected.isApprox(actual))
      << "square root covariance does not produce expected result.";
}

TEST_F(UKFMathTest, Propagate) {
  ukf::core::math::SigmaPoints expected = 2 * _expectedSigmaPoints;
  ukf::core::math::SigmaPoints actual =
      ukf::core::math::SigmaPoints::Zero(L, 2 * L + 1);

  ukf::core::math::propagate(_expectedSigmaPoints, actual,
                             [](auto value) { return 2 * value; });
  ASSERT_TRUE(expected.isApprox(actual))
      << "propagate does not produce expected result.";
}
