//
// Created by bene on 14.07.23.
//

#pragma once

#include <Eigen/Core>


namespace ukf {
    namespace core {


        template<typename Fields>
        class Covariance;

        template<typename ...State_Fields>
        class Covariance<ukf::core::StateFields<State_Fields...>> : public Eigen::MatrixXf {

        public:
            virtual ~Covariance() = default;

            //region constructors and assignments

            // TODO tuple initialization is missing -> currently the offset is 0 for all fields => clash
            // Constructs the state with initial values to zero
            Covariance() : Eigen::MatrixXf(ukf::core::StateFields<State_Fields...>::StateVectorType::Zero()) {}

            Covariance(Covariance &&other) noexcept = default;

            Covariance(const Covariance &other) = default;

            Covariance &operator=(const Covariance &other) = default;

            Covariance &operator=(Covariance &&other) noexcept = default;

            //endregion

            //region Eigen related constructors and operators
            template<typename OtherDerived>
            explicit Covariance(const Eigen::MatrixBase<OtherDerived> &other)
                    : Eigen::MatrixXf(other) {}

            template<typename OtherDerived>
            explicit Covariance(Eigen::MatrixBase<OtherDerived>
                                &&other)
                    :Eigen::VectorXd(std::move(other)) {}

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

            // Get field information  -> TODO check if necessary to expose
            template<typename Field>
            inline Field field() const {
                return _stateFields.template getField<Field>();
            }

            template<typename Field>
            Eigen::Matrix<float, Field::Size, Field::Size> fieldCovariance() const {
                const auto &field = _stateFields.template getField<Field>();
                return block<detail::FieldSize<Field>, detail::FieldSize<Field>>(field.offset, field.offset);
            }


        private:
            // manages fields including position and transition
            ukf::core::StateFields<State_Fields...> _stateFields{};
        };


    }
}
