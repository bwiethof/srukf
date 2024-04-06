//
// Created by bene on 01.07.23.
//

#pragma once

#include <Eigen/Core>
#include <tuple>
#include <vector>

#include "core/covariance.hpp"
#include "core/detail/traits.hpp"
#include "core/detail/transform.hpp"
#include "tl/optional.hpp"

namespace ukf {
namespace slam {

template <typename StateFields, typename MapFields>
class Covariance;

template <typename... State_Fields, typename... Map_Fields>
class Covariance<ukf::core::StateFields<State_Fields...>,
                 MapFields<Map_Fields...>>
    : public ukf::core::Covariance<ukf::core::StateFields<State_Fields...>> {
 public:
  Covariance() = default;

  template <typename OtherDerived>
  explicit Covariance(const Eigen::MatrixBase<OtherDerived> &other)
      : Eigen::MatrixXf(other) {}

  template <typename OtherDerived>
  explicit Covariance(Eigen::MatrixBase<OtherDerived> &&other)
      : Eigen::MatrixXf(std::move(other)) {}

  template <typename OtherDerived>
  Covariance &operator=(const Eigen::EigenBase<OtherDerived> &other) {
    this->Eigen::MatrixXf::operator=(other);
    return *this;
  }

  template <typename OtherDerived>
  Covariance &operator=(Eigen::EigenBase<OtherDerived> &&other) {
    this->Eigen::MatrixXf::operator=(std::move(other));
    return *this;
  }

  template <typename Field,
            typename FieldType = typename std::remove_reference_t<Field>>
  tl::optional<
      Eigen::Matrix<ukf::core::Float_t, FieldType::Size, FieldType::Size>>
  getFieldData(std::size_t id) const {
    const auto field = _mapFields.template get<FieldType>(id);
    if (!field) {
      return tl::nullopt;
    }

    return this->block<FieldType::Size, FieldType::Size>(field->offset,
                                                         field->offset);
  }

  template <typename Field, typename FieldData>
  void add(FieldData &&fieldData, std::size_t id) {
    // Resize state representation
    const auto previousSize =
        resizeInternal(std::remove_reference<Field>::type::Size);

    // Extend state representation
    addToState<Field>(previousSize, std::forward<FieldData>(fieldData));

    // store field reference
    std::get<std::vector<Field>>(_mapFields).emplace_back(id, previousSize);
  }

  template <typename Field>
  void remove(std::size_t id) {
    const auto &fieldEntries = getFieldReference<Field>();
    // Find element
    auto it = std::find_if(fieldEntries.begin(), fieldEntries.end(),
                           [id](const auto &f) { return f._id == id; });
    if (it == fieldEntries.end()) return;

    // shift state
    const auto indexToRemove = it->offset;
    removeFromState<Field>(indexToRemove);
    resize(Field::Size * -1);
  }

  template <typename Field, typename... FieldIds,
            std::enable_if_t<std::is_integral<FieldIds...>::value>>
  void remove(FieldIds... ids) {
    std::array<std::size_t, sizeof...(FieldIds)> idContainer = {ids...};
    auto &ref = getFieldReference<Field>();
    std::sort(idContainer.begin(), idContainer.end());
    for (const auto &id : idContainer) {
      const auto it = std::binary_search(ref.begin(), ref.end(),
                                         id);  // is a bool so should not work
      if (it == ref.end()) {
        continue;
      }
      removeFromState<Field>(it->offset);
    }

    resize(Field::Size * -1 * sizeof...(FieldIds));
  }

  template <typename Field, typename... OtherFields>
  void add(std::pair<std::size_t, OtherFields> &&...fields) {
    const auto extraSize = sizeof...(OtherFields) * Field::Size;
    // resize with new size
    const auto previousSize = resizeInternal(extraSize);
    // this->_mapVector.conservativeResize(this->_mapVector.size() + extraSize);
    // Add all fields to state
    const auto fieldContainer = {fields...};
    const auto stateSetter = [this](std::size_t pos, auto f) {
      // Store field with offset and given id
      getFieldReference<Field>().emplace_back(pos, f.first);

      // add to state representation
      addToState<Field>(pos, f.second);

      // shift index
      return pos + Field::Size;
    };
    std::accumulate(fieldContainer.begin(), fieldContainer.end(), previousSize,
                    stateSetter);
  }

  template <typename Derived>
  Covariance operator+=(const Eigen::EigenBase<Derived> &other) {
    this->operator+=(other);
    return *this;
  }

  template <typename Derived>
  Covariance operator+=(Eigen::EigenBase<Derived> &&other) {
    this->operator+=(std::move(other));
    return *this;
  }

 private:
  template <typename Field, typename DataField>
  void addToState(std::size_t index, DataField &&fieldData) {
    this->segment<Field::Size>(index) =
        Field::model.toState(std::forward<DataField>(fieldData));
  }

  template <typename Field>
  void removeFromState(std::size_t indexToRemove) {
    const auto shiftSize = this->size() - indexToRemove - Field::Size;
    const auto existingPos = indexToRemove + Field::Size;
    this->segment(indexToRemove, shiftSize) =
        this->segment(existingPos, shiftSize);
  }

  std::size_t resizeInternal(long extraSize) {
    const auto previousSize = this->size();
    const auto newSize = previousSize + extraSize;
    this->conservativeResize(newSize);
    return previousSize;
  }
  // marking private to prevent accidental resize
  using Eigen::MatrixXf::resize;

  template <typename Field>
  constexpr std::vector<Field> &getFieldReference() {
    return std::get<std::vector<Field>>(_mapFields);
  }

  // Holds reference to fields only existing once in the state vector
  std::tuple<State_Fields...> _stateFields;
  // Holds reference to fields exiting multiple times in state vector
  std::tuple<std::vector<Map_Fields>...> _mapFields;
};
}  // namespace slam
}  // namespace ukf
