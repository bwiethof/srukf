//
// Created by bene on 13.05.23.
//

#pragma once

#include <Eigen/Core>
#include <vector>

namespace ukf {
    namespace core {
        namespace detail {

            // Implementations for Functors
            namespace impl {

                template<typename T>
                std::size_t calculateSize(const std::vector<T> &s) {
                    return s.size() * T::Size;
                }

                template<typename T, typename T2, typename ...Args>
                std::size_t calculateSize(const T &t, const T2 &t2, Args ...args) {
                    return calculateSize(t) + calculateSize(t2, args...);
                }
            }

            // Interface to calculate absolute measurement vector size
            struct SizeFunctor {
                template<typename ...Args>
                std::size_t operator()(Args ...args) const {
                    return impl::calculateSize(args...);
                }
            };

        } // detail
    } // core
} // ukf
