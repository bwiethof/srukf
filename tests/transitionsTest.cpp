//
// Created by bene on 07.04.23.
//
#include "transitions.hpp"
#include "testUtilities.hpp"
#include <gtest/gtest.h>

TEST(Transitions, StateTransition) {
  ukf::test::StateTransitionTest transition;

  // state =n -> 2n +1 -> n = 3 -> 7
  const auto sigmaPoints = Eigen::MatrixXd::Identity(3, 7);

  const auto &transformed = transition.transform(sigmaPoints);
  const Eigen::MatrixXd expected = Eigen::MatrixXd::Identity(3, 7) * 2;

  EXPECT_EQ(transformed.size(), expected.size());
  EXPECT_EQ(transformed, expected);
}

TEST(Transitions, MeasurementPrediction_Predict) {
  ukf::test::MeasurementPredictionMock mock;

  const auto sigmaPoints = Eigen::MatrixXd::Identity(3, 7);
  const auto &transformed = mock.predict(sigmaPoints);
  const Eigen::MatrixXd expected = Eigen::MatrixXd::Identity(3, 7) * 2;

  EXPECT_EQ(transformed, ukf::MeasurementPrediction::GammaPoints());

  ukf::SensorData sensorData;
  const auto mockSensor = std::make_shared<ukf::test::SensorMock>();
  sensorData.add(mockSensor);
}
