//
// Created by bene on 10.08.23.
//

#pragma once

#include <Eigen/Cholesky>
#include <vector>

#include "core/detail/traits.hpp"
#include "core/field_base.h"

namespace ukf {
namespace core {
namespace transition {

namespace __detail {
template <typename State, typename Model, typename... Args, typename... Inputs>
auto performTimeUpdate(const State &state, const Model &model, double dt,
                       ukf::core::detail::Pack<Args...>, Inputs &&...inputs) {
  return model.timeUpdate(dt, state.template get<Args>()..., inputs...);
}

template <typename State, typename Field, typename Derived, typename... Inputs>
void fieldTimeUpdate(const State &state, const Field &field,
                     Eigen::DenseBase<Derived> &newState, double dt,
                     Inputs &&...inputs) {
  if (field.offset != std::numeric_limits<std::size_t>::max())
    newState.template segment<Field::Size>(field.offset) =
        performTimeUpdate(state, field, dt, typename Field::depsType{},
                          std::forward<Inputs>(inputs)...);
}
}  // namespace __detail

template <std::size_t I = 0, typename State, typename Derived, typename... Tp,
          typename... Inputs>
inline typename std::enable_if_t<I == sizeof...(Tp), void> timeUpdate(
    const State &, Eigen::MatrixBase<Derived> &, const std::tuple<Tp...> &,
    double, Inputs &&...) {
  // Implementation to enable compilation
}

template <std::size_t I = 0, typename State, typename Derived, typename... Tp,
          typename... Inputs>
    inline typename std::enable_if_t <
    I<sizeof...(Tp), void> timeUpdate(const State &state,
                                      Eigen::MatrixBase<Derived> &newState,
                                      const std::tuple<Tp...> &t, double dt,
                                      Inputs &&...) {
  __detail::fieldTimeUpdate(state, std::get<I>(t), newState, dt);
  timeUpdate<I + 1, State, Derived, Tp...>(state, newState, t, dt);
}

template <typename State>
class Performer {
 public:
  explicit Performer(State state)
      : _state(std::move(state)),
        _updatesState(State::Zero(_state.rows(), _state.cols())) {}

  template <std::size_t I = 0, typename... Tp, typename... Inputs>
  std::enable_if_t<I == sizeof...(Tp), State> perform(float,
                                                      const std::tuple<Tp...> &,
                                                      Inputs &&...) {
    return {};
  }

  template <std::size_t I = 0, typename... Tp, typename... Inputs>
      std::enable_if_t <
      I<sizeof...(Tp), State> perform(float dt, const std::tuple<Tp...> &t,
                                      Inputs &&...inputs) {
    fieldUpdate(dt, std::get<I>(t), std::forward<Inputs>(inputs)...);
    perform<I + 1>(dt, t, std::forward<decltype(inputs)>(inputs)...);
    return _updatesState;
  }

 private:
  static bool isValidOffset(std::size_t offset) {
    return offset != std::numeric_limits<std::size_t>::max();
  }

  template <typename FieldType, typename... Input>
  auto fieldUpdate(float dt, const FieldType &field, Input &&...input) {
    if (isValidOffset(field.offset))
      _updatesState.template segment<FieldType::Size>(field.offset) =
          withExpandArgs(dt, field, field.GetDependencies(), field.GetInputs(),
                         std::forward<Input>(input)...);
  }

  template <std::size_t N, typename... Args1, typename... Deps,
            typename... Args2>
  auto withExpandArgs(float dt, const HasModel<N, Args1..., Args2...> &field,
                      const HasDependencies<Args1...> &,
                      const HasInputs<Deps...> &, Args2 &&...args2) {
    return field.timeUpdate(
        dt, _state.template get<Args1>()...,
        std::get<Deps>(std::tuple<Args2...>(std::forward<Args2>(args2)...))...);
  }

  State _state{};
  State _updatesState{};
};

}  // namespace transition
}  // namespace core

namespace slam {
namespace transition {

namespace detail {

template <typename State, typename Model, typename Field, typename... Args>
auto performTimeUpdate(const State &state, const Field &self,
                       const Model &model, double dt,
                       ukf::core::detail::Pack<Args...>) {
  return model.mapUpdate(
      dt, state.template getById<typename Model::Self_Type>(self._id),
      state.template get<Args>()...);
}

template <typename State, typename Field, typename Derived>
void fieldTimeUpdate(const State &state, const std::vector<Field> &fields,
                     Eigen::DenseBase<Derived> &newState, double dt) {
  const std::size_t shift = state.rows() - newState.rows();
  for (auto const &field : fields) {
    newState.template segment<Field::Size>(field.offset - shift) =
        performTimeUpdate(state, field, Field::model, dt,
                          typename Field::ModelType::deps{});
  }
}

}  // namespace detail

template <std::size_t I = 0, typename State, typename Derived, typename... Tp>
inline typename std::enable_if_t<I == sizeof...(Tp), void> timeUpdate(
    const State &, Eigen::MatrixBase<Derived> &, const std::tuple<Tp...> &,
    double) {
  // Implementation to enable compilation
}

template <std::size_t I = 0, typename State, typename Derived, typename... Tp>
    inline typename std::enable_if_t <
    I<sizeof...(Tp), void> timeUpdate(const State &state,
                                      Eigen::MatrixBase<Derived> &newState,
                                      const std::tuple<Tp...> &t, double dt) {
  detail::fieldTimeUpdate(state, std::get<I>(t), newState, dt);
  timeUpdate<I + 1, State, Derived, Tp...>(state, newState, t, dt);
}
}  // namespace transition
}  // namespace slam
}  // namespace ukf
