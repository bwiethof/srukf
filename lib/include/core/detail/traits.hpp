//
// Created by bene on 13.05.23.
//
#pragma once

#include <cstddef>

namespace ukf {
namespace core {
namespace detail {
template <typename... Args>
struct Pack {};

template <typename T>
constexpr std::size_t FieldSize = T::Size;
};  // namespace detail

template <typename... Dependencies>
struct StateDependencies : detail::Pack<Dependencies...> {};

template <typename... Dependencies>
struct Inputs : detail::Pack<Dependencies...> {};

};  // namespace core
};  // namespace ukf
