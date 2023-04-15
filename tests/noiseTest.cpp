//
// Created by bene on 15.04.23.
//

#include "noise.hpp"
#include "testUtilities.hpp"
#include <gtest/gtest.h>

TEST(Noise, SystemNoise) {

  ukf::SystemNoise noise;

  Eigen::MatrixXd expected;

  auto addNoise = [&](long dim) {
    const auto noiseMock = std::make_shared<ukf::test::NoiseMock>(dim);
    noise.addNoise(noiseMock);

    const auto old = expected;
    const auto oldDim = expected.rows();
    expected.resize(oldDim + dim, +oldDim + dim);
    expected.setZero();
    expected.block(0, 0, oldDim, oldDim) = old;
    expected.block(oldDim, oldDim, dim, dim) = noiseMock->matrix();
  };

  // Only single matrix existing
  addNoise(5);
  EXPECT_EQ(noise.matrix(), expected);

  addNoise(6);
  EXPECT_EQ(noise.matrix(), expected);
}
