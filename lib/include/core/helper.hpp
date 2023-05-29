//
// Created by bene on 29.05.23.
//

#pragma once

#include <array>

namespace ukf {
    namespace core {

        /*
         * Implementation of C++20 feature "to_array" to create array from C-Style array (only for float for now)
         * Use "to_array" function to generate an array
         */
        template<std::size_t N, std::size_t... I>
        constexpr std::array<float, N> to_array_impl(const float (&in)[N], std::index_sequence<I...>) {
            return {{in[I]...}};
        }

        template<std::size_t N>
        std::array<float, N> to_array(const float (&in)[N]) {
            return to_array_impl(in, std::make_index_sequence<N>());
        }

    }
}
