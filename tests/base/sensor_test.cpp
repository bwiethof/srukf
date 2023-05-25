//
// Created by bene on 25.05.23.
//

#include <gtest/gtest.h>
#include "include/base/base_sensor.hpp"
#include "include/core/helper.hpp"
#include "mock/sensor.h"

namespace ukf::basetest {
    // What to test?
    // NoOp Sensor should really do No Op
    /*
     * GIVEN a user implementation of a StaticSensor
     *  WHEN new data is provided
     *      THEN an Eigen vector with the corresponding measurement is returned
     *      THEN an Eigen matrix with the corresponding noising is returned
     * */

    class BaseSensorFixture : public testing::TestWithParam<ukf::test::mock::Data> {
    public:
        ukf::test::mock::BaseSensor sensor;
    };

    TEST_P(BaseSensorFixture, SensorImpl) {
        ukf::test::mock::Data data = GetParam();
        const Eigen::Vector4f expectedMeasurement = {data.measure.a, data.measure.b, data.measure.c, data.measure.d};
        Eigen::Matrix4f expectedNoising;
        expectedNoising << data.noising.a[0], data.noising.a[1], data.noising.a[2], data.noising.a[3],
                data.noising.b[0], data.noising.b[1], data.noising.b[2], data.noising.b[3],
                data.noising.c[0], data.noising.c[1], data.noising.c[2], data.noising.c[3],
                data.noising.d[0], data.noising.d[1], data.noising.d[2], data.noising.d[3];

        EXPECT_EQ(sensor._z(data), expectedMeasurement);
        EXPECT_EQ(sensor._R(data), expectedNoising);
    };

    INSTANTIATE_TEST_SUITE_P(BaseSensor, BaseSensorFixture, testing::ValuesIn(ukf::test::mock::values));


// NoOp Sensor shall do no Operation
    TEST(sensor, NoOp) {
        ukf::NoOpSensor s;

        const Eigen::Vector<float, 0> expectedMeasurement;
        const Eigen::DiagonalMatrix<float, 0> expectedNoising;

        EXPECT_EQ(s._z({}), expectedMeasurement);
        EXPECT_EQ(s._R({}), expectedNoising.toDenseMatrix());
    }

}
