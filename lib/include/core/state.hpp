//
// Created by bene on 16.07.23.
//

#pragma once

#include <Eigen/Core>

#include "field.hpp"

namespace ukf {
namespace core {

namespace detail {

template <std::size_t Offset = 0, typename Derived>
constexpr auto compose(Eigen::MatrixBase<Derived> &) {
  // nothing to do here since nothing to set
}

template <std::size_t Offset = 0, typename Derived, typename T>
auto compose(Eigen::MatrixBase<Derived> &Q, const T &field) {
  Q.block(Offset, Offset, FieldSize<T>, FieldSize<T>) = field.noising();
}

template <std::size_t Offset = 0, typename Derived, typename T,
          typename... Args>
auto compose(Eigen::MatrixBase<Derived> &Q, const T &field,
             const Args &...args) {
  Q.block(Offset, Offset, FieldSize<T>, FieldSize<T>) = field.noising();
  compose<Offset + FieldSize<T>>(Q, args...);
}

template <typename... StateFields>
auto constructNoising() {
  const core::StateFields<StateFields...> fields;
  SquaredMatrix<DynamicSize> Q =
      SquaredMatrix<DynamicSize>::Zero(fields.StateSize, fields.StateSize);
  compose(Q, fields.template getField<StateFields>()...);
  return Q;
}
}  // namespace detail

template <typename Fields, int SIZE = 0>
class State;

template <typename... State_Fields>
class State<StateFields<State_Fields...>> : public state::StateBase {
  using Base = state::StateBase;

 public:
  virtual ~State() = default;

  // region constructors and assignments
  //  Constructs the state with initial values to zero
  State()
      : state::StateBase(
            StateFields<State_Fields...>::StateVectorType::Zero()) {}

  State(State &&other) noexcept = default;

  State(const State &other) = default;

  State &operator=(const State &other) = default;

  State &operator=(State &&other) noexcept = default;

  // endregion

  // region Eigen related constructors and operators
  template <typename OtherDerived>
  State(const Eigen::EigenBase<OtherDerived> &other)
      : Vector<DynamicSize>(other) {}

  template <typename OtherDerived>
  State(Eigen::EigenBase<OtherDerived> &&other)
      : state::StateBase(std::move(other)) {}

  template <typename OtherDerived>
  State &operator=(const Eigen::EigenBase<OtherDerived> &other) {
    this->Base::operator=(other);
    return *this;
  }

  template <typename OtherDerived>
  State &operator=(Eigen::EigenBase<OtherDerived> &&other) {
    this->Base::operator=(std::move(other));
    return *this;
  }

  // endregion Eigen related constructors and operators

  // performs timeupdate for given time interval
  virtual state::StateBase f(float dt) const {
    return _stateFields.apply(*this, dt);
  }

  template <typename... Inputs>
  state::StateBase f(float dt, Inputs &&...inputs) {
    return _stateFields.apply(*this, dt, std::forward<Inputs>(inputs)...);
  }

  template <class Field>
  Field get() const {
    Field field = _stateFields.template getField<Field>();
    field.data = fieldData<Field>();
    return field;
  }

  state::NoisingType noising() const { return _systemNoise; }

 private:
  template <typename Field>
  Vector<Field::Size> fieldData() const {
    return Base::segment<detail::FieldSize<Field>>(
        _stateFields.template getField<Field>().offset);
  }

  // manages fields including position and transition
  StateFields<State_Fields...> _stateFields{};
  state::NoisingType _systemNoise = detail::constructNoising<
      State_Fields...>();  // should be constant -> external maybe better and
                           // doing a compile time check?
};
}  // namespace core
}  // namespace ukf
