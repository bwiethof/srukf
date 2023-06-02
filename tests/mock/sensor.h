//
// Created by bene on 02.06.23.
//


#pragma once

#include "base/base_sensor.hpp"
#include "slam/sensor.hpp"
#include "core/helper.hpp"

namespace ukf::test::mock {
    struct Data {
        struct {
            float a;
            float b;
            float c;
            float d;
        } measure;
        struct {
            float a[4];
            float b[4];
            float c[4];
            float d[4];
        } noising;
    };

    const ukf::test::mock::Data values[] = {
            {
                    {1, 2, 3, 4},
                    {{1},                      {0,    2}, {0,    0,    3},          {0,    0,    0,    4}}
            },
            {
                    {1, 2, 3, 4},
                    {{1.1f, 1.2f, 1.3f, 1.4f}, {2.1f, 2.2f, 2.2f, 2.3f},
                                                          {3.1f, 3.2f, 3.3f, 3.4f}, {4.1f, 4.2f, 4.3f, 4.4f}}
            }
    };


    class BaseSensor : public ukf::StaticSensor<4, Data> {
    private:
        [[nodiscard]] DataContainerType zImpl(const Data &data) const override;

        [[nodiscard]] NoiseContainerType RImpl(const Base::Type &data) const override;
    };

    class SlamSensor : public ukf::slam::Sensor<4, Data> {
    public:
        using ukf::slam::Sensor<4, Data>::Sensor;

    private:
        [[nodiscard]] DataContainerType zImpl(const Type &data) const override;

        [[nodiscard]] NoiseContainerType RImpl(const Type &data) const override;
    };
} // ukf::test::mock
