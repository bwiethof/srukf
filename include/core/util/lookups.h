//
// Created by bene on 13.05.23.
//

#pragma once

#include <vector>
#include <tuple>
#include "core/util/EigenInc.h"
#include "core/util/functor.hpp"

namespace ukf {
    namespace core {
        namespace detail {

            namespace operation {
                // expands tuple values, passes to a given functor and returns the result
                template<typename TupleT, typename Func, std::size_t ...Indices>
                auto transform(const TupleT &tp, std::index_sequence<Indices...>, Func f) {
                    return f(std::get<Indices>(tp)...);
                }

            }

            // Helper to get tuple indices
            // recursive comparison of given type with trait type to get the index
            namespace index {
                template<std::size_t Offset, template<class> class trait, typename T1, typename T2>
                constexpr std::size_t getIndex() {
                    return std::is_same<typename std::remove_reference_t<T1>, typename trait<T2>::Type>::value ? Offset
                                                                                                               : std::numeric_limits<std::size_t>::max();
                }

                template<std::size_t Offset, template<class> class trait, typename T1, typename T2, typename T3, typename... Fields>
                constexpr std::size_t getIndex() {
                    return std::is_same<typename std::remove_reference_t<T1>, typename trait<T2>::Type>::value ? Offset
                                                                                                               : getIndex<
                                    Offset + 1, trait, T1, T3, Fields...>();
                }
            }
        }
    }
}
