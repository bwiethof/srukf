//
// Created by bene on 13.05.23.
//

#pragma once

#include <vector>
#include <tuple>
#include <numeric>
#include "core/util/EigenInc.h"
#include "core/util/functor.hpp"

namespace ukf {
    namespace core {
        namespace detail {

            namespace operation {
                // expands tuple values, passes to a given functor and returns the result
                template<typename TupleT, typename Func, std::size_t ...Indices>
                auto transform(const TupleT &tp, std::index_sequence<Indices...>, Func f) {
                    return f(std::get<Indices>(tp)...);
                }

                template<typename T, typename T2, typename ... Ts>
                auto concat(T &&t, T2 &&t2, Ts &&...args) {
                    return concat(concat(std::forward<T>(t), std::forward<T2>(t2)), std::forward<Ts>(args)...);
                }

                template<typename T, std::size_t N, std::size_t M>
                auto concat(const std::array<T, N> &ar1, const std::array<T, M> &ar2) {
                    std::array<T, N + M> result;
                    std::copy(ar1.cbegin(), ar1.cend(), result.begin());
                    std::copy(ar2.cbegin(), ar2.cend(), result.begin() + N);
                    return result;
                }

                template<std::size_t M, std::size_t N>
                auto flatten(const std::array<std::array<float, N>, M> &arr) {
                    std::array<float, N * M> result;
                    std::accumulate(arr.begin(), arr.end(), 0, [&](int offset, const std::array<float, N> &elem) {
                        std::move(elem.begin(), elem.end(), result.begin() + offset);
                        return offset + N;
                    });
                    return result;
                }
            }


            namespace generation {

                template<typename T>
                constexpr std::size_t VectorDimension = T::Size;

                template<typename T>
                constexpr T adder(T v) { return v; }

                template<typename T, typename... Args>
                constexpr T adder(T first, Args...args) {
                    return first + adder(args...);
                }

                template<typename... Sensors>
                constexpr std::size_t calculate_vector_size() { return adder(VectorDimension<Sensors>...); }

                template<typename T, std::size_t... Indices>
                constexpr std::array<T, sizeof...(Indices)> create_array(T value, std::index_sequence<Indices...>) {
                    // Cast Indices to void to remove the unused value warning.
                    return {{(static_cast<void>(Indices), value)...}};
                }

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

            // Helper to get tuple indices
            // recursive comparison of given type with trait type to get the index
            namespace index {
                template<std::size_t Offset, template<class> class trait, typename T1, typename T2>
                constexpr std::size_t getIndex() {
                    return std::is_same<typename std::remove_reference_t<T1>, typename trait<T2>::Type>::value ? Offset
                                                                                                               : std::numeric_limits<std::size_t>::max();
                }

                template<std::size_t Offset, template<class> class trait, typename T1, typename T2, typename T3, typename... Fields>
                constexpr std::size_t getIndex() {
                    return std::is_same<typename std::remove_reference_t<T1>, typename trait<T2>::Type>::value ? Offset
                                                                                                               : getIndex<
                                    Offset + 1, trait, T1, T3, Fields...>();
                }
            }
        }
    }
}
