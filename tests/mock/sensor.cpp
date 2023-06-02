//
// Created by bene on 02.06.23.
//

#include "sensor.h"

namespace ukf::test::mock {

    std::array<float, 4> BaseSensor::zImpl(const ukf::test::mock::Data &data) const {
        return {data.measure.a, data.measure.b, data.measure.c, data.measure.d};
    }

    std::array<std::array<float, 4>, 4> BaseSensor::RImpl(const Data &data) const {
        using ukf::core::to_array;
        return {{
                        to_array<4>(data.noising.a),
                        to_array<4>(data.noising.b),
                        to_array<4>(data.noising.c),
                        to_array<4>(data.noising.d)
                }};
    }

    std::array<float, 4> SlamSensor::zImpl(const Data &data) const {
        return {data.measure.a, data.measure.b, data.measure.c, data.measure.d};
    }

    std::array<std::array<float, 4>, 4> SlamSensor::RImpl(const Data &data) const {
        using ukf::core::to_array;
        return {{
                        to_array<4>(data.noising.a),
                        to_array<4>(data.noising.b),
                        to_array<4>(data.noising.c),
                        to_array<4>(data.noising.d)
                }};
    }
} // ukf::test::mock
