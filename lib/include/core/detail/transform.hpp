//
// Created by bene on 13.05.23.
//

#pragma once

#include <tuple>
#include <numeric>

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

                template<typename T> constexpr std::size_t VectorDimension = T::Size;

                constexpr std::size_t adder() { return 0; }


                template<typename T, typename... Args>
                constexpr T adder(T first, Args...args) {
                    return first + adder(args...);
                }

                template<typename... Sensors>
                constexpr std::size_t calculateStaticSize() { return adder(VectorDimension<Sensors>...); }
            }
        }
    }
}
