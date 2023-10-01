//
// Created by bene on 13.05.23.
//

#pragma once

#include <type_traits>
#include <algorithm>
#include "core/detail/traits.hpp"
#include "core/sensor.hpp"

namespace ukf {
    namespace slam {
        template<std::size_t N, typename Data, typename Self, typename ...Inputs>
        struct SensorModel : public ukf::slam::Model<N, Data, Self, Inputs...> {
            virtual ~SensorModel() = default;

            // timeUpdate in Sensor only uses current State. Therefore, dt is not needed
            virtual Eigen::Vector<float, N> mapUpdate(float, const Self &self, const Inputs &...input) const final {
                return predict(self, input...);
            }

            virtual Eigen::Vector<float, N> predict(const Self &, const Inputs &...inputs) const = 0;
        };
    } // namespace slam
} // namespace ukf
