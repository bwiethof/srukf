//
// Created by bene on 13.05.23.
//

#pragma once

#include <Eigen/Core>
#include <cstddef>

namespace ukf {
    namespace core {
        namespace detail {

            template<typename T>
            struct MeasurementConstructor {
                explicit MeasurementConstructor(std::size_t size) : m(Eigen::VectorXf(size)) {}

                void operator()(const T &d) {
                    m.segment<T::Size>(_offset) = d._z();
                    _offset += T::Size;
                }

                int _offset{};
                Eigen::VectorXf m{};
            };



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

                template<typename T>
                Eigen::VectorXf getMeasurement(const std::vector<T> &v) {
                    Eigen::VectorXf vec(calculateSize(v));
                    return std::for_each(v.begin(), v.end(), MeasurementConstructor<T>(vec.size())).m;
                }
            }

            // Interface to calculate absolute measurement vector size
            struct MeasurementSizeFunctor {
                template<typename ...Args>
                std::size_t operator()(Args ...args) const {
                    return impl::calculateSize(args...);
                }
            };

            // Interface to calculate all measurements and parse as array
            struct MeasurementFunctor {
                template<typename ...Indices>
                std::array<Eigen::VectorXf, sizeof...(Indices)> operator()(Indices ...args) {
                    return {{impl::getMeasurement(args)...}};
                }
            };


        } // detail
    } // core
} // ukf
