//
// Created by bwiethof on 2/28/23.
//

#pragma once


#include <Eigen/Dense>
#include "noise.hpp"

namespace ukf {
    class State {

    public:
        using Size = Eigen::Index;

        Size size() const { return _data.size(); }

        inline Eigen::VectorXd vector() const {
            return _data;
        }

        State() = default;


        // Implicit cast intentional
        template<typename Derived>
        State(Eigen::EigenBase<Derived> &&other):_data(std::move(other)) {}

        // Implicit cast intentional
        template<typename Derived>
        State(const Eigen::EigenBase<Derived> &other):_data(other) {}

        template<typename Derived>
        State &operator+=(const Eigen::EigenBase<Derived> &other) {
            _data += other;
            return *this;
        }

        template<typename Derived>
        State &operator=(Eigen::EigenBase<Derived> &&other) {
            _data = std::move(other);
            return *this;
        }

        void update(State &&other) {
            *this = std::move(other);
        }

        SystemNoise noise() const { return _Q; }

    protected:

        inline void addNoise(std::unique_ptr<Noise> &&noise) const {
            _Q.addNoise(std::move(noise));
        }

    private:

        Eigen::VectorXd _data;
        SystemNoise _Q{};

    };

}