//
// Created by bene on 04.06.23.
//

#pragma once

#include <Eigen/Core>

#include "sensor.hpp"

namespace ukf {
namespace core {

template <typename... Tp>
std::tuple<Tp...> constructTuple() {
  return {Tp{std::numeric_limits<std::size_t>::max()}...};
}

template <typename... Fields>
struct StaticFields {
  static constexpr std::size_t StateSize =
      detail::generation::calculateStaticSize<Fields...>();
  using StateVectorType = Vector<StateSize>;

  template <typename Field>
  Field get() const {
    return std::get<Field>(_fields);
  }

  template <typename Field>
  void updateOffset(std::size_t offset) {
    std::get<Field>(_fields).offset = offset;
  }

  template <typename State>
  ukf::core::state::MeasurementType apply(const State &state,
                                          std::size_t measurementSize) const {
    transition::Performer<State> performer{state};
    return performer.perform(-1, _fields).template segment(0, measurementSize);
  }

 private:
  // Create tuple with offset given input order
  std::tuple<Fields...> _fields = constructTuple<Fields...>();
};

template <typename Fields>
class SensorData;

template <typename... State_Fields>
class SensorData<StaticFields<State_Fields...>> {
 public:
  virtual ~SensorData() = default;

  template <typename State>
  state::MeasurementType h(const State &X) const {
    return _stateFields.apply(X, _measurement.rows());
  }

  template <typename Field, typename FieldData = typename Field::DataType>
  void setMeasurement(FieldData &&data) {
    const Field field = _stateFields.template get<Field>();
    if (field.offset != std::numeric_limits<std::size_t>::max()) {
      set<Field>(std::forward<FieldData>(data), field.offset);
      return;
    }

    const auto offset = append<Field>(std::forward<FieldData>(data));
    _stateFields.template updateOffset<Field>(offset);
  }

  virtual state::NoisingType noising() const { return _noising; }

  virtual std::size_t size() const { return _measurement.rows(); }

  virtual state::MeasurementType vector() const { return _measurement; }

 private:
  template <typename Field, typename Data>
  void set(Data &&data, std::size_t offset) {
    _noising.block<Field::Size, Field::Size>(offset, offset) =
        Field{}.noising(/*data*/);
    _measurement.segment<Field::Size>(offset) =
        Field{}.toVector(std::forward<Data>(data));
  }

  template <typename Field, typename Data>
  std::size_t append(Data &&data) {
    const auto offset = _measurement.size();
    const auto newSize = offset + Field::Size;

    const Matrix<DynamicSize, DynamicSize> zeroMatrix =
        Matrix<DynamicSize, DynamicSize>::Zero(newSize, Field::Size);
    _noising.conservativeResize(newSize, newSize);

    _noising.block(0, offset, newSize, Field::Size) = zeroMatrix;
    _noising.block(offset, 0, Field::Size, newSize) = zeroMatrix.transpose();
    _measurement.conservativeResize(newSize);

    set<Field>(std::forward<Data>(data), offset);
    return offset;
  }

  state::MeasurementType _measurement{};
  state::NoisingType _noising{};

  StaticFields<State_Fields...> _stateFields{};

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace core
}  // namespace ukf
