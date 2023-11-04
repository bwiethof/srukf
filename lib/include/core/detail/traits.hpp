//
// Created by bene on 13.05.23.
//
#pragma once

#include <cstddef>

namespace ukf {
    namespace core {
        namespace detail {
            template<typename ...Args>
            struct Pack {
            };

            template<typename T> constexpr std::size_t FieldSize = T::Size;

            template<typename T> typename T::ModelType Model = T::model;
        };
    };
};
