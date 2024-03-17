

#pragma once
#include <cstddef>

namespace ukf {
namespace core {

template <typename... Input>
struct HasInputs {
  using inputs = Inputs<Input...>;
  const HasInputs &GetInputs() const { return *this; }
};

template <typename... Deps>
struct HasDependencies {
  using depsType = StateDependencies<Deps...>;
  const HasDependencies &GetDependencies() const { return *this; }
};

template <std::size_t N, typename... Inputs>
struct HasModel {
  virtual ~HasModel() = default;
  HasModel() = default;

  HasModel(const HasModel &) noexcept = default;
  HasModel &operator=(const HasModel &) noexcept = default;

  HasModel(HasModel &&) noexcept = default;
  HasModel &operator=(HasModel &&) noexcept = default;

  virtual Eigen::Matrix<float, N, N> noising() const = 0;
  virtual Eigen::Vector<float, N> timeUpdate(float,
                                             const Inputs &...) const = 0;
};

template <std::size_t N>
struct HasData {
  explicit HasData(std::size_t offset) : offset(offset) {}
  HasData() = default;

  HasData(const HasData &) noexcept = default;
  HasData &operator=(const HasData &) noexcept = default;

  HasData(HasData &&) noexcept = default;
  HasData &operator=(HasData &&) noexcept = default;

  virtual ~HasData() = default;

  static constexpr std::size_t Size = N;
  // Offset to be used to track position in vector
  std::size_t offset{};

  Eigen::Vector<float, Size> data{};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace core
}  // namespace ukf