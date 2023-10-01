//
// Created by bwiethof on 3/13/23.
//
#include <gtest/gtest.h>
#include "core/parameters.hpp"


void matches(const ukf::core::UkfParameters::CompositionParameters &computed,
             const ukf::core::UkfParameters::CompositionParameters &expected) {
    EXPECT_DOUBLE_EQ(computed.gamma, expected.gamma);
    EXPECT_DOUBLE_EQ(computed.W0_s, expected.W0_s);
    EXPECT_DOUBLE_EQ(computed.W0_c, expected.W0_c);
    EXPECT_DOUBLE_EQ(computed.Wi, expected.Wi);

}

TEST(UkfParameters, Defaults) {

    auto params = ukf::core::UkfParameters({0, 0, 0});

    const auto computed = params.params();
    matches(computed, {});
}

TEST(UkfParameters, Simpleupdate) {

    auto params = ukf::core::UkfParameters({.alpha=1, .beta= 1, .kappa= 1});
    params.update(8);

    const auto computed = params.params();

    matches(computed, {.W0_s=1.0 / 9, .W0_c = 1.0 / 9 + 1, .Wi=0.0, .gamma = 3.0});


}
