


#include <gtest/gtest.h>
#include "core/detail/transform.hpp"


TEST(Transformation, operation) {

    using namespace ukf::core::detail::operation;

    std::tuple<int, std::string> tp{};

    const auto f = [](int a, const std::string &b) {
        return b + std::to_string(a);
    };
    ASSERT_EQ(transform(std::make_tuple(2, "x is "), std::make_index_sequence<2>(), f), "x is 2");

    // concat

    const std::array<int, 3> a{1, 2, 3};
    const std::array<int, 2> b{4, 5};
    const std::array<int, 5> c{6, 7, 8, 9, 10};
    const std::array<int, 7> d{11, 12, 13, 14, 15, 16, 17};

    const std::array<int, 5> resultA{1, 2, 3, 4, 5};
    ASSERT_EQ(concat(a, b), resultA);


    const std::array<int, 17> resultB{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17};

    ASSERT_EQ(concat(a, b, c, d), resultB);
}

namespace {
    struct TestType {
        static const std::size_t Size = 3;
    };
}

TEST(Transformation, generation) {
    using ukf::core::detail::generation::calculateStaticSize;

    auto result = calculateStaticSize<TestType, TestType>();
    ASSERT_EQ(result, 6);

    result = calculateStaticSize<TestType, TestType, TestType, TestType, TestType>();
    ASSERT_EQ(result, 15);


}