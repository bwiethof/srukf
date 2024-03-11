//
// Created by bene on 13.05.23.
//

#pragma once

#include <Eigen/Core>

#include "core/field.hpp"

namespace ukf {

namespace core {

template <std::size_t N, typename Data, typename... Inputs>
struct SensorModel : SimpleField<N, Inputs...> {
  ~SensorModel() override = default;

  using SimpleField<N, Inputs...>::SimpleField;

  // timeUpdate in Sensor only uses current State. Therefore, dt is not needed
  virtual Eigen::Vector<float, N> timeUpdate(
      float, const Inputs &...input) const final {
    return predict(input...);
  }

  virtual Eigen::Vector<float, N> toVector(Data &&data) const = 0;

  using DataType = Data;

 private:
  virtual Eigen::Vector<float, N> predict(const Inputs &...inputs) const = 0;
};
}  // namespace core
}  // namespace ukf
