//
// Created by bene on 01.11.23.
//
#include <gtest/gtest.h>
#include "core/sensor.hpp"

namespace {
    struct Data { float value{}; };

    struct MySensor : public ukf::core::SensorModel<1, Data> {
        Eigen::Matrix<float, 1UL, 1UL> noising() const override {
            return Eigen::Vector<float, 1>{1};
        }

        Eigen::Vector<float, 1UL> predict() const override {
            return Eigen::Vector<float, 1>{1};
        }

        Eigen::Vector<float, 1UL> toVector(Data &&data) const override {
            return Eigen::Vector<float, 1>{data.value};
        }
    };
}

TEST(Sensor, interface) {
    MySensor model{};

    ASSERT_EQ(model.toVector({.value=10.0f})[0], 10.0f);
    ASSERT_EQ(model.timeUpdate(1.0f)[0], 1.0f);
    ASSERT_EQ(model.noising()[0], 1.0f);

}