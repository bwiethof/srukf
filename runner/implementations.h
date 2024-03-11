//
// Created by bene on 01.07.23.
//

#pragma once

#include "core/ukf.hpp"
#include "include/slam/sensor.hpp"
#include "slam/covariance.h"
#include "slam/field.h"

struct sensor_accel_t {
  float x;
  float y;
  float z;
  float temperature;
};

struct NoOpField : ukf::core::SimpleField<1> {
  using ukf::core::SimpleField<1>::SimpleField;
  Eigen::Vector<float, 1UL> timeUpdate(float) const override { return {}; }

  Eigen::Matrix<float, 1UL, 1UL> noising() const override { return {}; }
};

struct StandardFieldImpl : public ukf::core::SimpleField<4, NoOpField> {
  using Base = ukf::core::SimpleField<4, NoOpField>;
  StandardFieldImpl() = default;
  explicit StandardFieldImpl(std::size_t offset) : Base(offset) {}
  Eigen::Vector<float, 4> timeUpdate(float,
                                     const NoOpField &field) const override {
    (void)field;
    return {};
  }

  Eigen::Matrix<float, 4, 4> noising() const override { return {}; }
};

/*struct SecondFieldModel;
using SlamFieldImpl = ukf::slam::Field<SecondFieldModel>;

struct SecondFieldModel
    : public ukf::slam::Model<4, sensor_accel_t, SlamFieldImpl, NoOpField> {
  SecondFieldModel() = default;

  Eigen::Vector<float, 4> mapUpdate(float, const SlamFieldImpl &self,
                                    const NoOpField &field) const override {
    (void)self;
    (void)field;

    return {};
  }

  Eigen::Vector<float, 4> toState(const sensor_accel_t &) const override {
    return {};
  }

  Eigen::Matrix<float, 4, 4> toCovariance(
      const sensor_accel_t &) const override {
    return {};
  }

  Eigen::Matrix<float, 4, 4> noising() const override { return {}; }
};
*/

struct ExpSensorImpl
    : public ukf::core::Sensor<4, sensor_accel_t, StandardFieldImpl> {
  Eigen::Vector<float, 4> predict(
      const StandardFieldImpl &field) const override {
    (void)field;
    return {};
  }

  Eigen::Vector<float, 4> toVector(sensor_accel_t &&) const override {
    return {};
  }

  Eigen::Matrix<float, 4, 4> noising() const override { return {}; }
};

/*
struct ExpSlamSensorModelImpl
    : public ukf::slam::SensorModel<4, sensor_accel_t, SlamFieldImpl> {
  Eigen::Vector<float, 4> predict(const SlamFieldImpl &field) const override {
    (void)field;
    return {};
  }

  Eigen::Vector<float, 4> toState(const sensor_accel_t &) const override {
    return {};
  }

  Eigen::Matrix<float, 4, 4> toCovariance(
      const sensor_accel_t &) const override {
    return {};
  }

  Eigen::Matrix<float, 4, 4> noising() const override { return {}; }
};
using ExpSlamSensorImpl = ukf::slam::Field<ExpSlamSensorModelImpl>;
*/
