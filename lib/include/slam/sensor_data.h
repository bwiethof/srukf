//
// Created by bene on 06.05.23.
//
#pragma once

#include <Eigen/Core>
#include <iostream>
#include <numeric>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "core/detail/functor.hpp"
#include "core/detail/traits.hpp"
#include "core/detail/transform.hpp"
#include "core/sensor_data.h"

namespace ukf {
namespace slam {

template <typename SingleFields, typename MultiFields>
class SensorData;

template <typename... SingleFields, typename... MultiFields>
class SensorData<ukf::core::StaticFields<SingleFields...>,
                 ukf::slam::MapFields<MultiFields...>>
    : public ukf::core::SensorData<ukf::core::StaticFields<SingleFields...>> {
  using Base = ukf::core::SensorData<ukf::core::StaticFields<SingleFields...>>;

 public:
  using Base::operator=;

  template <typename State>
  Eigen::VectorXf h(const State &X) const {
    return _multiFields.apply(X, -1);
  }

  template <typename Field, typename FieldData>
  void addMeasurement(FieldData &&data, std::size_t id) {
    const auto offset = append<Field>(std::forward<FieldData>(data));

    _multiFields.template add<Field>(offset, id);
  }

  Eigen::MatrixXf noising() const override {
    const auto baseNoising = Base::noising();
    const long baseNoisingSize = baseNoising.rows();
    const long size = _noising.rows();

    const auto fullSize = baseNoisingSize + size;
    Eigen::MatrixXf noising = Eigen::MatrixXf::Zero(fullSize, fullSize);
    noising.block(0, 0, baseNoisingSize, baseNoisingSize) = baseNoising;
    noising.block(baseNoisingSize, baseNoisingSize, size, size) = _noising;
    return noising;
  }

  std::size_t size() const override {
    return Base::size() + _measurement.size();
  }

  virtual Eigen::VectorXf vector() const override {
    const Eigen::VectorXf baseMeasurement = Base::vector();
    Eigen::VectorXf measurement(baseMeasurement.rows() + _measurement.rows());
    measurement.segment(0, baseMeasurement.rows()) = baseMeasurement;
    measurement.segment(baseMeasurement.rows(), _measurement.rows()) =
        _measurement;

    return measurement;
  }

 private:
  template <typename Field, typename Data>
  void set(Data &&data, std::size_t offset) {
    _noising.block<Field::Size, Field::Size>(offset, offset) =
        Field::model.noising(/*data*/);
    _measurement.segment<Field::Size>(offset) =
        Field::model.toState(std::forward<Data>(data));
  }

  template <typename Field, typename Data>
  std::size_t append(Data &&data) {
    const auto offset = _measurement.size();
    const auto newSize = offset + Field::Size;
    _noising.conservativeResize(newSize, newSize);
    _measurement.conservativeResize(newSize);

    set<Field>(std::forward<Data>(data), offset);
    return offset;
  }

  Eigen::VectorXf _measurement{};
  Eigen::MatrixXf _noising{};

  ukf::slam::MapFields<MultiFields...> _multiFields{};
};

}  // namespace slam
}  // namespace ukf
