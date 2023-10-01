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

        template<typename Fields, int SIZE = 0>
        class State;

        // TODO: how to work this out with size
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
            explicit State(const Eigen::MatrixBase<OtherDerived> &other)
                    : Eigen::VectorXf(other) {}

            template<typename OtherDerived>
            explicit State(Eigen::MatrixBase<OtherDerived> &&other)
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


            // Get field information  -> TODO check if necessary to expose
            template<typename Field>
            inline Field field() const {
                return _stateFields.template getField<Field>();
            }

            template<typename Field>
            Eigen::Vector<float, Field::Size> fieldData() const {
                return segment<detail::FieldSize<Field>>(_stateFields.template getField<Field>().offset);
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
            // manages fields including position and transition
            ukf::core::StateFields<State_Fields...> _stateFields{};
            Eigen::MatrixXf _systemNoise{}; // should be constant -> external maybe better and doing a compile time check?
        };
    }
}
