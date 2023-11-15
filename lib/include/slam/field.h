//
// Created by bene on 01.07.23.
//

#pragma once

#include <Eigen/Core>
#include "core/field.hpp"
#include "core/detail/transform.hpp"
#include "core/detail/functor.hpp"
#include <tl/optional.hpp>

namespace ukf {

    namespace slam {
        template<std::size_t N, typename Measurement, typename Self, typename ...Inputs>
        struct Model : public core::Model<N, Inputs...> {
            // Shall implement transformation from measurement to representation
            virtual Eigen::Vector<float, N> toState(const Measurement &measurement) const = 0;

            virtual Eigen::Matrix<float, N, N> toCovariance(const Measurement &measurement) const = 0;

            using Self_Type = Self;

            Eigen::Vector<float, N> timeUpdate(float, const Inputs &...) const override {
                // should not work
                return {};
            }

            virtual Eigen::Vector<float, N> mapUpdate(float, const Self &, const Inputs &...) const = 0;

        };

        template<typename Model>
        struct Field : public core::Field<Model> {
            using core::Field<Model>::Size;
            using Base = core::Field<Model>;

            Field(std::size_t offset, std::size_t id) : Base(offset), _id(id) {}

            std::size_t _id{};
        };


        template<typename ...Fields>
        class MapFields {

            template<typename Field>
            struct IdMatcher {
                explicit IdMatcher(std::size_t id) : _id(id) {};

                bool operator()(const Field &f) {
                    return f._id == _id;
                }

                std::size_t _id{};
            };


        public:
            template<typename Field>
            tl::optional <Field> get(std::size_t id) const {
                const std::vector<Field> fields = getFields<Field>();
                const auto it = std::find_if(fields.begin(), fields.end(), IdMatcher<Field>{id});
                if (it == fields.end()) {
                    return tl::nullopt;
                }
                return *it;
            }

            template<class Field, typename ... FieldIds>
            std::vector<Field> getFields(FieldIds ...ids) const {
                std::array<std::size_t, sizeof...(FieldIds)> idContainer = {ids...};
                std::vector<Field> fields;

                const auto it = std::copy_if(_fields.begin(), _fields.end(), fields.begin(),
                                             [&idContainer](const Field &f) {
                                                 return std::find(idContainer.begin(), idContainer.end(), f._id) !=
                                                        idContainer.end();
                                             });

                return fields;
            }

            template<typename Field>
            bool add(std::size_t offset, std::size_t id) {
                if (get<Field>(id)) {
                    return false;
                }

                getFieldRef<Field>().emplace_back(offset, id);
                return true;
            }

            template<typename Field>
            bool remove(std::size_t id) {
                const std::vector<Field> fields = getFieldRef<Field>();
                const auto it = std::remove_if(fields.begin(), fields.end(), IdMatcher<Field>{id});
                if (it == fields.end()) {
                    return false;
                }

                fields.erase(it);
                return true;
            }

            template<typename State>
            Eigen::VectorXf apply(const State &state, double dt) const {
                Eigen::VectorXf updatedState = Eigen::VectorXf::Zero(size());
                ukf::slam::transition::timeUpdate(state, updatedState, _fields, dt);
                return updatedState;
            }

            std::size_t size() const {
                using core::detail::operation::transform;
                const auto seq = std::make_index_sequence<sizeof...(Fields)>();
                return transform(_fields, seq, core::detail::SizeFunctor{});
            }

        private:


            template<typename Field>
            std::vector<Field> getFields() const {
                return std::get<std::vector<Field>>(_fields);
            }

            template<class Field>
            std::vector<Field> &getFieldRef() {
                return std::get<std::vector<Field>>(_fields);
            }

            std::tuple<std::vector<Fields>...> _fields;
        };

    }
}
