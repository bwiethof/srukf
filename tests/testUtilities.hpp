//
// Created by bene on 15.04.23.
//

#pragma once

#include "noise.hpp"
#include "state.hpp"
#include "transitions.hpp"

namespace ukf::test {

class NoiseMock : public ukf::Noise {

  long _dimension{};

public:
  NoiseMock() = default;
  explicit NoiseMock(long dimension) : _dimension(dimension) {}

  Eigen::MatrixXd matrix() override {
    return Eigen::MatrixXd::Identity(_dimension, _dimension);
  }
  long dimension() override { return _dimension; }
};

class StateTransitionTest : public ukf::StateTransition {
  Eigen::VectorXd f(ukf::State &&state) override { return state.vector() * 2; }
};

class MeasurementPredictionMock : public ukf::MeasurementPrediction {
  Eigen::VectorXd h(ukf::State &&state) override {
    const auto dim = dimension();
    return Eigen::VectorXd::Identity(dim, 1);
  }
};

class SensorMock : public ukf::Sensor {
public:
  SensorMock() : ukf::Sensor(3) {}
  explicit SensorMock(int dim) : ukf::Sensor(dim) {}
  ~SensorMock() override = default;

  [[nodiscard]] Eigen::VectorXd measurement() const override {
    return Eigen::VectorXd::Identity(dimension(), 1); // == (1,0,0...,0)^T
  }
  [[nodiscard]] const std::shared_ptr<ukf::Noise> noise() const override {
    return std::make_shared<NoiseMock>(dimension());
  }
};

} // namespace ukf::test
