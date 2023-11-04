//
// Created by bene on 01.11.23.
//

#include <gtest/gtest.h>
#include "core/field.hpp"
#include "core/state.hpp"


namespace {

    struct MockModel1 : public ukf::core::Model<3> {
        Eigen::Matrix<float, 3UL, 3UL> noising() const override {
            return {};
        }

        Eigen::Vector<float, 3UL> timeUpdate(float dt) const override {
            return Eigen::Vector3f::Constant(dt);
        }
    };

    using MockFieldImpl1 = ukf::core::Field<3, MockModel1>;

    struct MockModel2 : public ukf::core::Model<2, MockFieldImpl1> {
        Eigen::Matrix<float, 2UL, 2UL> noising() const override {
            return Eigen::Matrix2f::Identity();
        }

        Eigen::Vector<float, 2UL> timeUpdate(float, const MockFieldImpl1 &field) const override {
            return {field.data[0], field.data[1]};
        }
    };

    using MockFieldImpl2 = ukf::core::Field<2, MockModel2>;


    template<std::size_t N>
    struct MockWithOffsetAndSize {

        MockWithOffsetAndSize() = default;

        explicit MockWithOffsetAndSize(std::size_t s) : _value(s) {}

        std::size_t _value{};

        static constexpr std::size_t Size = N;
    };

    using MockFieldSize3 = MockWithOffsetAndSize<3>;
    using MockFieldSize5 = MockWithOffsetAndSize<5>;
    using MockFieldSize4 = MockWithOffsetAndSize<4>;
    using MockFieldSize7 = MockWithOffsetAndSize<7>;

}

TEST(detail, TupleConstruction) {
    using ukf::core::detail::constructTuple;

    // single field
    {
        const auto tp = constructTuple<MockFieldSize3>();
        ASSERT_EQ(get<MockFieldSize3>(tp)._value, 0);
    }

    // 2 fields
    {
        const auto tp = constructTuple<MockFieldSize3, MockFieldSize5>();
        ASSERT_EQ(get<MockFieldSize3>(tp)._value, 0);
        ASSERT_EQ(get<MockFieldSize5>(tp)._value, 3);
    }

    // multiple fields
    {
        const auto tp = constructTuple<MockFieldSize3, MockFieldSize5, MockFieldSize4, MockFieldSize7>();
        ASSERT_EQ(get<MockFieldSize3>(tp)._value, 0);
        ASSERT_EQ(get<MockFieldSize5>(tp)._value, 3);
        ASSERT_EQ(get<MockFieldSize4>(tp)._value, 8);
        ASSERT_EQ(get<MockFieldSize7>(tp)._value, 12);
    }

    // order is relevant
    {
        const auto tp = constructTuple<MockFieldSize5, MockFieldSize7, MockFieldSize4, MockFieldSize3>();
        ASSERT_EQ(get<MockFieldSize5>(tp)._value, 0);
        ASSERT_EQ(get<MockFieldSize7>(tp)._value, 5);
        ASSERT_EQ(get<MockFieldSize4>(tp)._value, 12);
        ASSERT_EQ(get<MockFieldSize3>(tp)._value, 16);
    }
}

TEST(StateFields, construction) {

    {
        ukf::core::StateFields<MockFieldImpl1, MockFieldImpl2> fields{};

        const auto field1 = fields.getField<MockFieldImpl1>();
        const auto field2 = fields.getField<MockFieldImpl2>();

        ASSERT_EQ(field1.offset, 0);
        ASSERT_EQ(field2.offset, MockFieldImpl1::Size);
    }

    {
        ukf::core::StateFields<MockFieldImpl2, MockFieldImpl1> fields{};

        const auto field1 = fields.getField<MockFieldImpl1>();
        const auto field2 = fields.getField<MockFieldImpl2>();

        ASSERT_EQ(field1.offset, MockFieldImpl2::Size);
        ASSERT_EQ(field2.offset, 0);
    }

}

TEST(StateFields, apply) {
    using StateFieldsTestImpl = ukf::core::StateFields<MockFieldImpl1, MockFieldImpl2>;
    using StateType = ukf::core::State<StateFieldsTestImpl>;
    StateFieldsTestImpl fields{};

    const StateType X((Eigen::VectorXf(5) << 1, 2, 3, 4, 5).finished());

    const Eigen::Vector<float, 5> expected(1, 1, 1, 1, 2);

    const auto result = fields.apply(X, 1.0);

    ASSERT_TRUE(result.isApprox(expected));

}