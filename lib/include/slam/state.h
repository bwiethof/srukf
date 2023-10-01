//
// Created by bene on 27.06.23.
//

#pragma once


#include <vector>
#include "core/detail/transform.hpp"
#include "core/detail/traits.hpp"
#include "core/state.hpp"
#include "field.h"
#include <tl/optional.hpp>

namespace ukf {
    namespace slam {


        template<typename StateFields, typename MapFields>
        class State;


        template<typename ...State_Fields, typename ...Map_Fields>
        class State<ukf::core::StateFields<State_Fields...>, MapFields<Map_Fields...>>
                : public ukf::core::State<ukf::core::StateFields<State_Fields...>> {
            using Base = ukf::core::State<ukf::core::StateFields<State_Fields...>>;
        public:

            State() = default;


            template<typename OtherDerived>
            explicit State(const Eigen::MatrixBase<OtherDerived> &other)
                    : Base(other) {}

            template<typename OtherDerived>
            explicit State(Eigen::MatrixBase<OtherDerived> &&other)
                    :  Base(std::move(other)) {}

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


            template<typename Field, typename FieldType = typename std::remove_reference_t<Field>>
            tl::optional<Eigen::Vector<float, FieldType::Size>> getFieldData(std::size_t id) const {
                const auto field = _mapFields.template get<FieldType>(id);
                if (!field) {
                    return tl::nullopt;
                }

                return this->segment<FieldType::Size>(field->offset);
            }

            Eigen::VectorXf f(float dt) const override {
                const auto baseState = Base::f(dt);
                const auto mapState = _mapFields.template apply(*this, dt);
                return baseState;
            }


            template<typename Field, typename FieldData>
            void add(FieldData &&fieldData, std::size_t id) {
                // Resize state representation
                const auto previousSize = resizeInternal(std::remove_reference<Field>::type::Size);

                // Extend state representation
                addToState<Field>(previousSize, std::forward<FieldData>(fieldData));
                addToSystemNoise<Field>(previousSize);

                // store field reference
                _mapFields.template add<Field>(previousSize, id);
            }

            template<typename Field>
            void remove(std::size_t id) {
                const auto field = _mapFields.template get<Field>(id);
                const auto indexToRemove = field->offset;

                // Remove entry
                removeFromState<Field>(indexToRemove);
                removeFromSystemNoise<Field>(indexToRemove);
                _mapFields.template remove<Field>(id);
                // resize accordingly
                resizeInternal(Field::Size * -1);
            }

            template<typename Field, typename ... FieldIds, std::enable_if_t<std::is_integral<FieldIds...>::value>>
            int remove(FieldIds ...ids) {
                const auto fields = _mapFields.template getFields<Field>(std::forward(ids...));
                int count = 0;
                for (const auto &field: fields) {
                    removeFromState<Field>(field->offset);
                    removeFromSystemNoise<Field>(field->offset);
                    _mapFields.template remove<Field>(field._id);
                    ++count;
                }

                resizeInternal(Field::Size * -1 * sizeof...(FieldIds));
                return count;
            }

            template<typename Field, typename ...OtherFields>
            void add(std::pair<std::size_t, OtherFields> &&...fields) {
                const auto extraSize = sizeof...(OtherFields) * Field::Size;
                // resize with new size
                const auto previousSize = resizeInternal(extraSize);
                // Add all fields to state
                const auto fieldContainer = {fields...};
                const auto stateSetter = [this](std::size_t pos, auto f) {
                    // Store field with offset and given id
                    _mapFields.template add<Field>(pos, f.first);

                    // add to state representation
                    addToState<Field>(pos, f.second);
                    addToSystemNoise<Field>(pos);

                    // shift index
                    return pos + Field::Size;
                };
                std::accumulate(fieldContainer.begin(), fieldContainer.end(), previousSize, stateSetter);

            }

            template<typename Derived>
            State operator+=(const Eigen::EigenBase<Derived> &other) {
                this->operator+=(other);
                return *this;
            }

            template<typename Derived>
            State operator+=(Eigen::EigenBase<Derived> &&other) {
                this->operator+=(std::move(other));
                return *this;
            }


            // Store system noise for now in state
            Eigen::MatrixXf systemNoise() const {
                return _systemNoise;
            }

            template<typename Field>
            Field getById(long id) const {
                auto field = _mapFields.template get<Field>(id);
                field->data = this->template segment<Field::Size>(field->offset);
                return field.value();
            }


        private:

            template<typename Field, typename DataField>
            void addToState(std::size_t index, DataField &&fieldData) {
                this->template segment<Field::Size>(index) = Field::model.toState(std::forward<DataField>(fieldData));

            }

            template<typename Field>
            void removeFromState(std::size_t indexToRemove) {
                const auto shiftSize = this->size() - indexToRemove - Field::Size;
                const auto existingPos = indexToRemove + Field::Size;
                this->segment(indexToRemove, shiftSize) = this->segment(existingPos, shiftSize);

            }

            template<typename Field, typename DataField>
            void removeFromSystemNoise(std::size_t indexToRemove) {
                const auto shiftSize = this->size() - indexToRemove - Field::Size;
                const auto existingPos = indexToRemove + Field::Size;
                _systemNoise.block(indexToRemove, indexToRemove, shiftSize, shiftSize) = _systemNoise.block(existingPos,
                                                                                                            shiftSize);
            }

            template<typename Field>
            void addToSystemNoise(std::size_t index) {
                _systemNoise.block<Field::Size, Field::Size>(index, index) = Field::model.noising();
            }


            std::size_t resizeInternal(long extraSize) {
                const auto previousSize = this->size();
                const auto newSize = previousSize + extraSize;
                this->conservativeResize(newSize);
                _systemNoise.conservativeResize(newSize, newSize);
                return previousSize;
            }

            // marking private to prevent accidental resize
            using Eigen::VectorXf::resize;

            Eigen::MatrixXf _systemNoise;

            // Holds reference to fields exiting multiple times in state vector
            ukf::slam::MapFields<Map_Fields...> _mapFields;
        };
    }
}
