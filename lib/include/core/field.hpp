//
// Created by bene on 05.06.23.
//

#pragma once


#include <Eigen/Core>
#include "core/transition.hpp"
#include "core/detail/traits.hpp"
#include "core/detail/transform.hpp"

namespace ukf {
    namespace core {


        namespace detail {

            template<std::size_t I, typename Field>
            constexpr Field constructField() {
                return Field(I);
            }

            template<std::size_t I = 0, std::size_t Offset = 0, typename... Tp>
            inline typename std::enable_if_t<I == sizeof...(Tp), void> constructTupleImpl(const std::tuple<Tp...> &) {
                // Implementation to enable compilation
            }

            template<std::size_t I, typename ...Tp>
            struct TupleTypeTrait {
                using type = typename std::tuple_element<I, std::tuple<Tp...>>::type;
            };

            template<std::size_t I = 0, std::size_t Offset = 0, typename... Tp>
            inline typename std::enable_if_t<I < sizeof...(Tp), void> constructTupleImpl(std::tuple<Tp...> &t) {
                std::get<I>(t) = constructField<Offset, typename TupleTypeTrait<I, Tp...>::type>();

                constructTupleImpl<I + 1, Offset + TupleTypeTrait<I, Tp...>::type::Size, Tp...>(t);
            }

            template<typename ...Args>
            constexpr std::tuple<Args...> constructTuple() {
                std::tuple<Args...> tp;
                constructTupleImpl(tp);
                return tp;
            }
        }


        template<std::size_t N, typename ...Inputs>
        struct Model {
            virtual ~Model() = default;

            using deps = detail::Pack<Inputs...>;

            static constexpr std::size_t Size = N;

            // TODO: expect value for noising
            virtual Eigen::Matrix<float, N, N> noising() const = 0;

            virtual Eigen::Vector<float, N> timeUpdate(float dt, const Inputs &...input) const = 0;
        };

        template<std::size_t N, typename Model>
        struct Field {
            Field() = default;

            explicit Field(std::size_t offset) : offset{offset} {}

            virtual ~Field() = default;

            const static Model model;
            using ModelType = Model;

            static constexpr std::size_t Size = Model::Size;
            // Offset to be used to track position in vector
            std::size_t offset{};

            Eigen::Vector<float, N> data{};
        };

        template<std::size_t N, typename Model> const Model Field<N, Model>::model;

        /**
         * @class StateFields
         * @tparam Fields
         *
         * Holds information about fields in State
         * State fields are only allowed once per state
         */
        template<typename ...Fields>
        struct StateFields {

            static constexpr std::size_t StateSize = detail::generation::calculateStaticSize<Fields...>();
            using StateVectorType = Eigen::Vector<float, StateSize>;

            template<typename Field>
            Field getField() const {
                return std::get<Field>(_fields);
            }

            template<typename State>
            StateVectorType apply(const State &state, double dt) const {
                StateVectorType updatedState = StateVectorType::Zero();
                ukf::core::transition::timeUpdate(state, updatedState, _fields, dt);
                return updatedState;
            }


        private:
            // Create tuple with offset given input order
            std::tuple<Fields...> _fields = detail::constructTuple<Fields...>();
        };
    }


}
