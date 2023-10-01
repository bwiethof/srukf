//
// Created by bene on 01.07.23.
//

#pragma once

#include "include/slam/sensor.hpp"
#include "core/ukf.hpp"
#include "slam/covariance.h"
#include "slam/field.h"


struct sensor_accel_t {
    float x;
    float y;
    float z;
    float temperature;
};


struct NoOpModel : public ukf::core::Model<1> {

    Eigen::Vector<float, 1UL> timeUpdate(float) const override {
        return {};
    }

    Eigen::Matrix<float, 1UL, 1UL> noising() const override {
        return {};
    }
};

using NoOpField = ukf::core::Field<1, NoOpModel>;

struct FirstFieldModel : public ukf::core::Model<4, NoOpField> {

    FirstFieldModel() = default;

    Eigen::Vector<float, 4> timeUpdate(float, const NoOpField &field) const override {
        (void) field;
        return {};
    }


    Eigen::Matrix<float, 4, 4> noising() const override {
        return {};
    }
};

struct SecondFieldModel;
using SlamFieldImpl = ukf::slam::Field<4, SecondFieldModel>;

struct SecondFieldModel : public ukf::slam::Model<4, sensor_accel_t, SlamFieldImpl, NoOpField> {

    SecondFieldModel() = default;


    Eigen::Vector<float, 4> mapUpdate(float, const SlamFieldImpl &self, const NoOpField &field) const override {
        (void) self;
        (void) field;

        return {};
    }

    Eigen::Vector<float, 4> toState(const sensor_accel_t &) const override {
        return {};
    }

    Eigen::Matrix<float, 4, 4> toCovariance(const sensor_accel_t &) const override {
        return {};
    }

    Eigen::Matrix<float, 4, 4> noising() const override {
        return {};
    }
};


using StandardFieldImpl = ukf::core::Field<4, FirstFieldModel>;

struct ExpSensorModelImpl : public ukf::core::SensorModel<4, sensor_accel_t, StandardFieldImpl> {

    Eigen::Vector<float, 4> predict(const StandardFieldImpl &field) const override {
        (void) field;
        return {};
    }

    Eigen::Vector<float, 4> toVector(sensor_accel_t &&) const override {
        return {};
    }


    Eigen::Matrix<float, 4, 4> noising() const override {
        return {};
    }

};


struct ExpSlamSensorModelImpl : public ukf::slam::SensorModel<4, sensor_accel_t, SlamFieldImpl> {

    Eigen::Vector<float, 4> predict(const SlamFieldImpl &field) const override {
        (void) field;
        return {};
    }

    Eigen::Vector<float, 4> toState(const sensor_accel_t &) const override {
        return {};
    }

    Eigen::Matrix<float, 4, 4> toCovariance(const sensor_accel_t &) const override {
        return {};
    }


    Eigen::Matrix<float, 4, 4> noising() const override {
        return {};
    }

};

using ExpSensorImpl = ukf::core::Field<4, ExpSensorModelImpl>;
using ExpSlamSensorImpl = ukf::slam::Field<4, ExpSlamSensorModelImpl>;
