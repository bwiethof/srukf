//
// Created by bene on 01.11.23.
//
#include "core/sensor.hpp"

#include <gtest/gtest.h>

#include "core/typedefs.h"

namespace {
struct Data {
  double value{};
};

struct MySensor : public ukf::core::Sensor<1, Data> {
  Noising noising() const override {
    return ukf::core::state::FieldNoising<1>{1};
  }

  Data predict() const override { return ukf::core::state::FieldData<1>{1}; }

  ukf::core::Vector<1> toVector(::Data&& data) const override {
    return ukf::core::Vector<1>{data.value};
  }
};
}  // namespace

TEST(Sensor, interface) {
  MySensor model{};

  ASSERT_EQ(model.toVector({.value = 10.0f})[0], 10.0f);
  ASSERT_EQ(model.timeUpdate(1.0f)[0], 1.0f);
  ASSERT_EQ(model.noising()[0], 1.0f);
}