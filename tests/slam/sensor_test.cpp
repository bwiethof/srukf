//
// Created by bene on 25.05.23.
//

#include <gtest/gtest.h>
#include "base/base_sensor.hpp"
#include "mock/sensor.h"

namespace ukf::test::slam {

    class SlamSensorFixture : public testing::TestWithParam<ukf::test::mock::Data> {
    };

    TEST_P(SlamSensorFixture, SensorImpl) {
        ukf::test::mock::Data data = GetParam();
        const Eigen::Vector4f expectedMeasurement = {data.measure.a, data.measure.b, data.measure.c, data.measure.d};
        Eigen::Matrix4f expectedNoising;
        expectedNoising << data.noising.a[0], data.noising.a[1], data.noising.a[2], data.noising.a[3],
                data.noising.b[0], data.noising.b[1], data.noising.b[2], data.noising.b[3],
                data.noising.c[0], data.noising.c[1], data.noising.c[2], data.noising.c[3],
                data.noising.d[0], data.noising.d[1], data.noising.d[2], data.noising.d[3];

        ukf::test::mock::SlamSensor sensor(data, 0);
        EXPECT_EQ(sensor.z(), expectedMeasurement);
        EXPECT_EQ(sensor.R(), expectedNoising);
    };


    INSTANTIATE_TEST_SUITE_P(BaseSensor, SlamSensorFixture, testing::ValuesIn(ukf::test::mock::values));


// NoOp Sensor shall do no Operation
    TEST(sensor, NoOp) {
        ukf::NoOpSensor s;

        const Eigen::Vector<float, 0> expectedMeasurement;
        const Eigen::DiagonalMatrix<float, 0> expectedNoising;

        EXPECT_EQ(s.z({}), expectedMeasurement);
        EXPECT_EQ(s.R({}), expectedNoising.toDenseMatrix());
    }
}
