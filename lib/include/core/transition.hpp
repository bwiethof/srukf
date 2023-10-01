//
// Created by bene on 10.08.23.
//

#pragma once

#include <vector>
#include "core/field.hpp"
#include "core/detail/traits.hpp"

namespace ukf {
    namespace core {
        namespace transition {

            namespace detail {
                template<typename State, typename Model, typename ...Args>
                auto
                performTimeUpdate(const State &state, const Model &model, double dt, ukf::core::detail::Pack<Args...>) {
                    return model.timeUpdate(dt, state.template get<Args>()...);
                }

                template<typename State, typename Field, typename Derived>
                void fieldTimeUpdate(const State &state, const Field &field, Eigen::DenseBase<Derived> &newState,
                                     double dt) {
                    newState.template segment<Field::Size>(field.offset) = performTimeUpdate(state, Field::model, dt,
                                                                                             typename Field::ModelType::deps{});
                }
            }

            template<std::size_t I = 0, typename State, typename Derived, typename... Tp>
            inline typename std::enable_if_t<I == sizeof...(Tp), void>
            timeUpdate(const State &, Eigen::MatrixBase<Derived> &, const std::tuple<Tp...> &, double) {
                // Implementation to enable compilation
            }

            template<std::size_t I = 0, typename State, typename Derived, typename... Tp>
            inline typename std::enable_if_t<I < sizeof...(Tp), void>
            timeUpdate(const State &state, Eigen::MatrixBase<Derived> &newState, const std::tuple<Tp...> &t,
                       double dt) {
                detail::fieldTimeUpdate(state, std::get<I>(t), newState, dt);
                timeUpdate<I + 1, State, Derived, Tp...>(state, newState, t, dt);
            }
        }
    }

    namespace slam {
        namespace transition {

            namespace detail {


                template<typename State, typename Model, typename Field, typename ...Args>
                auto performTimeUpdate(const State &state, const Field &self, const Model &model, double dt,
                                       ukf::core::detail::Pack<Args...>) {

                    return model.mapUpdate(dt, state.template getById<typename Model::Self_Type>(self._id),
                                           state.template get<Args>()...);
                }


                template<typename State, typename Field, typename Derived>
                void fieldTimeUpdate(const State &state, const std::vector<Field> &fields,
                                     Eigen::DenseBase<Derived> &newState, double dt) {
                    const std::size_t shift = state.rows() - newState.rows();
                    for (auto const   &field: fields) {

                        newState.template segment<Field::Size>(field.offset - shift) = performTimeUpdate(state, field,
                                                                                                         Field::model,
                                                                                                         dt,
                                                                                                         typename Field::ModelType::deps{});
                    }
                }


            }

            template<std::size_t I = 0, typename State, typename Derived, typename... Tp>
            inline typename std::enable_if_t<I == sizeof...(Tp), void>
            timeUpdate(const State &, Eigen::MatrixBase<Derived> &, const std::tuple<Tp...> &, double) {
                // Implementation to enable compilation
            }

            template<std::size_t I = 0, typename State, typename Derived, typename... Tp>
            inline typename std::enable_if_t<I < sizeof...(Tp), void>
            timeUpdate(const State &state, Eigen::MatrixBase<Derived> &newState, const std::tuple<Tp...> &t,
                       double dt) {
                detail::fieldTimeUpdate(state, std::get<I>(t), newState, dt);
                timeUpdate<I + 1, State, Derived, Tp...>(state, newState, t, dt);
            }
        }
    }
}
