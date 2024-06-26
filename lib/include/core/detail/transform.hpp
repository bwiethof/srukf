//
// Created by bene on 13.05.23.
//

#pragma once

#include <numeric>
#include <tuple>

#include "core/typedefs.h"

namespace ukf {
namespace core {
namespace detail {

namespace generation {

template <typename T>
constexpr std::size_t VectorDimension = T::Size;

constexpr std::size_t adder() { return 0; }

template <typename T, typename... Args>
constexpr T adder(T first, Args... args) {
  return first + adder(args...);
}

template <typename... Sensors>
constexpr std::size_t calculateStaticSize() {
  return adder(VectorDimension<Sensors>...);
}
}  // namespace generation

namespace operation {
// expands tuple values, passes to a given functor and returns the result
template <typename TupleT, typename Func, std::size_t... Indices>
auto transform(const TupleT &tp, std::index_sequence<Indices...>, Func f) {
  return f(std::get<Indices>(tp)...);
}

template <typename T, std::size_t N, std::size_t M>
std::array<T, N + M> concat(const std::array<T, N> &ar1,
                            const std::array<T, M> &ar2) {
  std::array<T, N + M> result;
  std::copy(ar1.cbegin(), ar1.cend(), result.begin());
  std::copy(ar2.cbegin(), ar2.cend(), result.begin() + N);
  return result;
}

template <typename>
struct array_size;

template <typename T, std::size_t N>
struct array_size<std::array<T, N>> {
  static std::size_t const Size = N;
};

// calculates the resulting array after concatenating
template <typename T, typename T2, typename... Ts>
constexpr std::size_t calcSize() {
  return generation::calculateStaticSize<array_size<std::decay_t<T>>,
                                         array_size<std::decay_t<T2>>,
                                         array_size<std::decay_t<Ts>>...>();
}

template <typename T, typename T2, typename... Ts>
std::array<typename std::decay_t<T>::value_type, calcSize<T, T2, Ts...>()>
concat(T &&t, T2 &&t2, Ts &&...args) {
  const auto temp = concat(std::forward<T>(t), std::forward<T2>(t2));
  return concat(temp, std::forward<Ts>(args)...);
}

template <std::size_t M, std::size_t N>
ukf::core::Array<N * M> flatten(const std::array<ukf::core::Array<N>, M> &arr) {
  ukf::core::Array<N * M> result;
  std::accumulate(arr.begin(), arr.end(), 0,
                  [&](int offset, const ukf::core::Array<N> &elem) {
                    std::move(elem.begin(), elem.end(),
                              result.begin() + offset);
                    return offset + N;
                  });
  return result;
}
}  // namespace operation

}  // namespace detail
}  // namespace core
}  // namespace ukf
