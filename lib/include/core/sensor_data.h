//
// Created by bene on 04.06.23.
//

#pragma once

#include <Eigen/Core>
#include "sensor.hpp"

namespace ukf {
    namespace core {


        template< typename ...Tp >
        std::tuple<Tp...> constructTuple() {
            return { Tp{ std::numeric_limits<std::size_t>::max() }... };
        }

        template< typename ...Fields >
        struct StaticFields {

            static constexpr std::size_t StateSize = detail::generation::calculateStaticSize<Fields...>();
            using StateVectorType = Eigen::Vector<float, StateSize>;

            template< typename Field >
            Field get() const {
                return std::get<Field>(_fields);
            }

            template< typename Field >
            void updateOffset(std::size_t offset) {
                std::get<Field>(_fields).offset = offset;
            }

            template< typename State >
            StateVectorType apply(const State &state, double dt) const {
                StateVectorType updatedState = StateVectorType::Zero();
                ukf::core::transition::timeUpdate(state, updatedState, _fields, dt);
                return updatedState;
            }


        private:
            // Create tuple with offset given input order
            std::tuple<Fields...> _fields = constructTuple<Fields
                                                           ...>();
            //= detail::constructTuple<Fields...>();
        };


        template< typename Fields >
        class SensorData;

        template< typename ...State_Fields >
        class SensorData<StaticFields<State_Fields...>> {

        public:

            template< typename State >
            Eigen::VectorXf h(const State &X) const {
                return _stateFields.apply(X, -1);
            }

            template< typename Field, typename FieldData >
            void setMeasurement(FieldData &&data) {
                const Field field = _stateFields.template get<Field>();
                if (field.offset != std::numeric_limits<std::size_t>::max()) {
                    set<Field>(std::forward<FieldData>(data), field.offset);
                    return;
                }

                const auto offset = append<Field>(std::forward<FieldData>(data));
                _stateFields.template updateOffset<Field>(offset);
            }


            virtual Eigen::MatrixXf noising() const {
                return _noising;
            }

            virtual std::size_t size() const {
                return _measurement.rows();
            }

            virtual Eigen::VectorXf vector() const {
                return _measurement;
            }


        private:
            template< typename Field, typename Data >
            void set(Data &&data, std::size_t offset) {
                _noising.block<Field::Size, Field::Size>(offset, offset) = Field::model.noising(/*data*/);
                _measurement.segment<Field::Size>(offset) = Field::model.toVector(
                        std::forward<Data>(data));
            }

            // problem: mix of multi and non multi
            template< typename Field, typename Data >
            std::size_t append(Data &&data) {
                const auto offset = _measurement.size();
                const auto newSize = offset + Field::Size;
                _noising.conservativeResize(newSize, newSize);
                _measurement.conservativeResize(newSize);

                set<Field>(std::forward<Data>(data), offset);
                return offset;
            }

            Eigen::VectorXf _measurement{};
            Eigen::MatrixXf _noising{};

            StaticFields<State_Fields...> _stateFields{};
        };


    }
}