//
// Created by bene on 14.07.23.
//

#pragma once

#include <Eigen/Core>
#include "field.hpp"


namespace ukf {
    namespace core {


        template<typename Fields>
        class Covariance;

        template<typename ...State_Fields>
        class Covariance<ukf::core::StateFields<State_Fields...>> : public Eigen::MatrixXf {
            static constexpr std::size_t Size = ukf::core::StateFields<State_Fields...>::StateSize;
        public:
            virtual ~Covariance() = default;

            //region constructors and assignments

            Covariance()
                    : Eigen::MatrixXf(Eigen::MatrixXf::Zero(Size, Size)) {}

            Covariance(Covariance &&other) noexcept = default;

            Covariance(const Covariance &other) = default;

            Covariance &operator=(const Covariance &other) = default;

            Covariance &operator=(Covariance &&other) noexcept = default;

            //endregion

            //region Eigen related constructors and operators
            template<typename OtherDerived>
            Covariance(const Eigen::MatrixBase<OtherDerived> &other)
                    : Eigen::MatrixXf(other) {
            }

            template<typename OtherDerived>
            Covariance(Eigen::MatrixBase<OtherDerived> &&other)
                    :Eigen::MatrixXf(std::move(other)) {}

            template<typename OtherDerived>
            Covariance &operator=(const Eigen::EigenBase<OtherDerived> &other) {
                this->Eigen::MatrixXf::operator=(other);
                return *this;
            }

            template<typename OtherDerived>
            Covariance &operator=(Eigen::EigenBase<OtherDerived> &&other) {
                this->Eigen::MatrixXf::operator=(std::move(other));
                return *this;
            }

            //endregion Eigen related constructors and operators

            template<typename Field>
            Eigen::Matrix<float, Field::Size, Field::Size> get() const {
                const auto &field = _stateFields.template getField<Field>();
                return block<detail::FieldSize<Field>, detail::FieldSize<Field >>(field.offset, field.offset);
            }


        private:
            // manages fields including position and transition
            ukf::core::StateFields<State_Fields...> _stateFields{};
        };


    }
}
