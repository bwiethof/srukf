//
// Created by bene on 05.06.23.
//

#pragma once

#include <Eigen/Core>

#include "core/detail/traits.hpp"
#include "core/detail/transform.hpp"
#include "core/field_base.h"
#include "core/transition.hpp"

namespace ukf {
namespace core {

namespace detail {

template <std::size_t I, typename Field>
constexpr Field constructField() {
  return Field(I);
}

template <std::size_t I = 0, std::size_t Offset = 0, typename... Tp>
inline typename std::enable_if_t<I == sizeof...(Tp), void> constructTupleImpl(
    const std::tuple<Tp...> &) {
  // Implementation to enable compilation
}

template <std::size_t I, typename... Tp>
struct TupleTypeTrait {
  using type = typename std::tuple_element<I, std::tuple<Tp...>>::type;
};

template <std::size_t I = 0, std::size_t Offset = 0, typename... Tp>
    inline typename std::enable_if_t <
    I<sizeof...(Tp), void> constructTupleImpl(std::tuple<Tp...> &t) {
  std::get<I>(t) =
      constructField<Offset, typename TupleTypeTrait<I, Tp...>::type>();

  constructTupleImpl<I + 1, Offset + TupleTypeTrait<I, Tp...>::type::Size,
                     Tp...>(t);
}

template <typename... Args>
constexpr std::tuple<Args...> constructTuple() {
  std::tuple<Args...> tp;
  constructTupleImpl(tp);
  return tp;
}

}  // namespace detail

template <std::size_t N, typename T1, typename T2>
struct Field;

template <std::size_t N, typename... Dependencies, typename... Input>
struct Field<N, StateDependencies<Dependencies...>, Inputs<Input...>>
    : HasData<N>,
      HasModel<N, Dependencies..., Input...>,
      HasInputs<Input...>,
      HasDependencies<Dependencies...> {
  explicit Field(std::size_t offset) : HasData<N>(offset) {}
  Field() = default;
};

template <std::size_t N, typename... Dependencies>
using SimpleField = Field<N, StateDependencies<Dependencies...>, Inputs<>>;

/**
 * @class StateFields
 * @tparam Fields
 *
 * Holds information about fields in State
 * State fields are only allowed once per state
 */
template <typename... Fields>
struct StateFields {
  static constexpr std::size_t StateSize =
      detail::generation::calculateStaticSize<Fields...>();
  using StateVectorType = Eigen::Vector<float, StateSize>;

  template <typename Field>
  Field getField() const {
    return std::get<Field>(_fields);
  }

  template <typename State, typename... Inputs>
  StateVectorType apply(const State &state, double dt,
                        Inputs &&...inputs) const {
    transition::Performer<State> performer{state};

    return performer.perform(dt, _fields, std::forward<Inputs>(inputs)...);
  }

 private:
  // Create tuple with offset given input order
  std::tuple<Fields...> _fields = detail::constructTuple<Fields...>();
};
}  // namespace core
}  // namespace ukf
