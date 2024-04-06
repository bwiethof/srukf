//
// Created by bene on 14.07.23.
//

#pragma once

#include <Eigen/Core>

#include "field.hpp"

namespace ukf {
namespace core {

template <typename Fields>
class Covariance;

template <typename... State_Fields>
class Covariance<StateFields<State_Fields...>> : public state::CovarianceBase {
  static constexpr std::size_t Size = StateFields<State_Fields...>::StateSize;
  using Base = state::CovarianceBase;

 public:
  virtual ~Covariance() = default;

  // region constructors and assignments

  Covariance()
      : state::CovarianceBase(state::CovarianceBase::Zero(Size, Size)) {}

  Covariance(Covariance &&other) noexcept = default;

  Covariance(const Covariance &other) = default;

  Covariance &operator=(const Covariance &other) = default;

  Covariance &operator=(Covariance &&other) noexcept = default;

  // endregion

  // region Eigen related constructors and operators
  template <typename OtherDerived>
  Covariance(const Eigen::MatrixBase<OtherDerived> &other)
      : state::CovarianceBase(other) {}

  template <typename OtherDerived>
  Covariance(Eigen::MatrixBase<OtherDerived> &&other)
      : state::CovarianceBase(std::move(other)) {}

  template <typename OtherDerived>
  Covariance &operator=(const Eigen::EigenBase<OtherDerived> &other) {
    this->Base::operator=(other);
    return *this;
  }

  template <typename OtherDerived>
  Covariance &operator=(Eigen::EigenBase<OtherDerived> &&other) {
    this->Base::operator=(std::move(other));
    return *this;
  }

  // endregion Eigen related constructors and operators

  template <typename Field>
  SquaredMatrix<Field::Size> get() const {
    const auto &field = _stateFields.template getField<Field>();
    return Base::block<detail::FieldSize<Field>, detail::FieldSize<Field>>(
        field.offset, field.offset);
  }

 private:
  // manages fields including position and transition
  StateFields<State_Fields...> _stateFields{};
};

}  // namespace core
}  // namespace ukf
