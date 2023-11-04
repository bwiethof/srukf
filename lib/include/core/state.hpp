//
// Created by bene on 16.07.23.
//

#pragma once

#include <Eigen/Core>
#include <ostream>
#include "field.hpp"
#include "slam/field.h"

namespace ukf {
    namespace core {

        namespace detail {
            template<std::size_t Offset, typename Derived, typename T>
            auto compose(Eigen::MatrixBase<Derived> &Q, const T &) {
                Q.block(Offset, Offset, FieldSize<T>, FieldSize<T>) = T::model.noising();
            }

            template<std::size_t Offset, typename Derived, typename T, typename ...Args>
            auto compose(Eigen::MatrixBase<Derived> &Q, const T &, const Args &...args) {
                Q.block(Offset, Offset, FieldSize<T>, FieldSize<T>) = Model<T>.noising();
                compose<Offset + FieldSize<T>>(Q, args...);
            }

            template<typename ...StateFields>
            auto constructNoising() {
                const ukf::core::StateFields<StateFields...> fields;
                Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(fields.StateSize, fields.StateSize);
                compose<0>(Q, fields.template getField<StateFields>()...);
                return Q;
            }
        }

        template<typename Fields, int SIZE = 0>
        class State;

        template<typename ...State_Fields>
        class State<ukf::core::StateFields<State_Fields...>> : public Eigen::VectorXf {

        public:
            virtual ~State() = default;

            //region constructors and assignments
            // Constructs the state with initial values to zero
            State() : Eigen::VectorXf(ukf::core::StateFields<State_Fields...>::StateVectorType::Zero()) {}

            State(State &&other) noexcept = default;

            State(const State &other) = default;

            State &operator=(const State &other) = default;

            State &operator=(State &&other) noexcept = default;

            //endregion

            //region Eigen related constructors and operators
            template<typename OtherDerived>
            explicit State(const Eigen::EigenBase<OtherDerived> &other)
                    : Eigen::VectorXf(other) {}

            template<typename OtherDerived>
            explicit State(Eigen::EigenBase<OtherDerived> &&other)
                    : Eigen::VectorXf(std::move(other)) {}

            template<typename OtherDerived>
            State &operator=(const Eigen::EigenBase<OtherDerived> &other) {
                this->Eigen::VectorXf::operator=(other);
                return *this;
            }

            template<typename OtherDerived>
            State &operator=(Eigen::EigenBase<OtherDerived> &&other) {
                this->Eigen::VectorXf::operator=(std::move(other));
                return *this;
            }

            //endregion Eigen related constructors and operators

            // performs timeupdate for given time interval
            virtual Eigen::VectorXf f(float dt) const {
                return _stateFields.apply(*this, dt);
            }

            template<class Field>
            Field get() const {
                Field field = _stateFields.template getField<Field>();
                field.data = fieldData<Field>();
                return field;
            }

            Eigen::MatrixXf noising() const {
                return _systemNoise;
            }


        private:

            template<typename Field>
            Eigen::Vector<float, Field::Size> fieldData() const {
                return segment<detail::FieldSize<Field >>(_stateFields.template getField<Field>().offset);
            }

            // manages fields including position and transition
            ukf::core::StateFields<State_Fields...> _stateFields{};
            Eigen::MatrixXf _systemNoise = detail::constructNoising<State_Fields...>(); // should be constant -> external maybe better and doing a compile time check?
        };
    }
}
