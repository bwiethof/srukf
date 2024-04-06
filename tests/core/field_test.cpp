//
// Created by bene on 01.11.23.
//

#include "core/field.hpp"

#include <gtest/gtest.h>

#include "core/state.hpp"

namespace {
using ukf::core::state::FieldData;
using ukf::core::state::FieldNoising;

struct MockFieldImpl1
    : public ukf::core::Field<3, ukf::core::StateDependencies<>,
                              ukf::core::Inputs<>> {
  using Field::Field;
  Noising noising() const override { return {}; }

  Data timeUpdate(float dt) const override { return Data::Constant(dt); }
};

struct MockFieldImpl2
    : public ukf::core::Field<2, ukf::core::StateDependencies<MockFieldImpl1>,
                              ukf::core::Inputs<>> {
  using Field::Field;
  Noising noising() const override { return Noising::Identity(); }

  Data timeUpdate(float, const MockFieldImpl1 &field) const override {
    return {field.data[0], field.data[1]};
  }
};

template <std::size_t N>
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

}  // namespace

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
    const auto tp = constructTuple<MockFieldSize3, MockFieldSize5,
                                   MockFieldSize4, MockFieldSize7>();
    ASSERT_EQ(get<MockFieldSize3>(tp)._value, 0);
    ASSERT_EQ(get<MockFieldSize5>(tp)._value, 3);
    ASSERT_EQ(get<MockFieldSize4>(tp)._value, 8);
    ASSERT_EQ(get<MockFieldSize7>(tp)._value, 12);
  }

  // order is relevant
  {
    const auto tp = constructTuple<MockFieldSize5, MockFieldSize7,
                                   MockFieldSize4, MockFieldSize3>();
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
  using namespace ukf::core;
  using StateFieldsTestImpl =
      ukf::core::StateFields<MockFieldImpl1, MockFieldImpl2>;
  using StateType = ukf::core::State<StateFieldsTestImpl>;
  StateFieldsTestImpl fields{};

  const StateType X((Vector<DynamicSize>(5) << 1, 2, 3, 4, 5).finished());

  const Vector<5> expected(1, 1, 1, 1, 2);

  const auto result = fields.apply(X, 1.0);

  ASSERT_TRUE(result.isApprox(expected));
}