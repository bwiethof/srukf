//
// Created by bene on 29.05.23.
//

#pragma once

#include <array>

#include "core/typedefs.h"

namespace ukf {
namespace core {

/*
 * Implementation of C++20 feature "to_array" to create array from C-Style array
 * (only for float for now) Use "to_array" function to generate an array
 */
template <std::size_t N, std::size_t... I>
constexpr Array<N> to_array_impl(const PRECISION (&in)[N],
                                 std::index_sequence<I...>) {
  return {{in[I]...}};
}

template <std::size_t N>
Array<N> to_array(const PRECISION (&in)[N]) {
  return to_array_impl(in, std::make_index_sequence<N>());
}

}  // namespace core
}  // namespace ukf
