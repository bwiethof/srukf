


#include <gtest/gtest.h>
#include "core/detail/functor.hpp"

namespace {
    struct TestType {
        static const std::size_t Size = 3;
    };

    struct TestType2 {
        static const std::size_t Size = 2;
    };
}

TEST(SizeFunctor, Calculations) {
    ukf::core::detail::SizeFunctor functor{};

    std::vector<TestType> v;
    ASSERT_EQ(functor(v), 0);


    v.push_back({});
    v.push_back({});

    ASSERT_EQ(functor(v), 6);


    std::vector<TestType2> v2;
    ASSERT_EQ(functor(v, v2), 6);

    // Add values to the other vector
    v2.push_back({});
    v2.push_back({});
    v2.push_back({});

    ASSERT_EQ(functor(v, v2), 12);

}
