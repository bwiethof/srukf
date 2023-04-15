//
// Created by bwiethof on 3/13/23.
//
#include <gtest/gtest.h>
#include "parameters.hpp"


//class ParametersTest : public testing::TestWithParam<ukf::UKFParameters::ScalingParameters> {
//
//};


void matches(const ukf::UKFParameters::CompositionParameters &computed,
             const ukf::UKFParameters::CompositionParameters &expected) {
    EXPECT_DOUBLE_EQ(computed.gamma, expected.gamma);
    EXPECT_DOUBLE_EQ(computed.W0, expected.W0);
    EXPECT_DOUBLE_EQ(computed.Wc, expected.Wc);

}

TEST(UKFParameters, Defaults) {

    auto params = ukf::UKFParameters({0, 0, 0});

    const auto computed = params.params();
    matches(computed, {});
}

TEST(UKFParameters, Simpleupdate) {

    auto params = ukf::UKFParameters({.alpha=1, .beta= 1, .kappa= 1});
    params.update(8);

    const auto computed = params.params();

    matches(computed, {.W0=1.0/9, .Wc = 1.0/9+1, .gamma = 3.0});


}