#include <core/field.hpp>
#include <utility>

#include "core/state.hpp"
#include "implementations.h"
#include "slam/sensor_data.h"
#include "slam/ukf.hpp"
// Sample to test impl
// Example Data

namespace test {

template <typename... Input>
struct HasInputs {
  using inputs = ukf::core::Inputs<Input...>;
  const HasInputs &GetInputs() const { return *this; }
};

template <typename... Deps>
struct HasDependencies {
  using depsType = ukf::core::StateDependencies<Deps...>;
  const HasDependencies &GetDependencies() const { return *this; }
};

template <std::size_t N, typename... Inputs>
struct HasModel {
  virtual ~HasModel() = default;

  virtual Eigen::Matrix<float, N, N> noising() const = 0;
  virtual Eigen::Vector<float, N> timeUpdate(float,
                                             const Inputs &...) const = 0;
};

template <std::size_t N>
struct HasData {
  virtual ~HasData() = default;
  static constexpr std::size_t Size = N;
  // Offset to be used to track position in vector
  std::size_t offset{};

  Eigen::Vector<float, Size> data{};

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <std::size_t N, typename T1, typename T2>
struct FieldExtension;

template <std::size_t N, typename... Dependencies, typename... Input>
struct FieldExtension<N, ukf::core::StateDependencies<Dependencies...>,
                      ukf::core::Inputs<Input...>>
    : HasData<N>,
      HasModel<N, Dependencies..., Input...>,
      HasInputs<Input...>,
      HasDependencies<Dependencies...> {};

template <std::size_t N, typename... Dependencies>
using Simple = FieldExtension<N, ukf::core::StateDependencies<Dependencies...>,
                              ukf::core::Inputs<>>;

template <typename State>
struct TestPerformer {
  template <typename FieldType, typename... Input>
  auto performOnModel(float dt, const FieldType &field, Input &&...input) {
    return performOnArgs(dt, field, field.GetDependencies(), field.GetInputs(),
                         std::forward<Input>(input)...);
  }

  template <std::size_t N, typename... Args1, typename... Deps,
            typename... Args2>
  auto performOnArgs(float dt,
                     const test::HasModel<N, Args1..., Args2...> &field,
                     const HasDependencies<Args1...> &,
                     const HasInputs<Deps...> &, Args2 &&...args2) {
    return field.timeUpdate(
        dt, _state.template get<Args1>()...,
        std::get<Deps>(std::tuple<Args2...>(std::forward<Args2>(args2)...))...);
  }

  template <std::size_t N, typename... Args>
  static auto performImpl(float dt, HasModel<N, Args...> &&model,
                          Args &&...args) {
    return model.timeUpdate(dt, model, args...);
  }

  State _state;
};

struct TestState {
  template <typename T>
  T get() {
    return {};
  }
};

}  // namespace test

struct TestField
    : test::FieldExtension<
          3, ukf::core::StateDependencies<TestField, float, double>,
          ukf::core::Inputs<int>> {
  TestField() { data = Eigen::Vector3f(1, 2, 3); }

  Eigen::Matrix<float, 3, 3> noising() const override { return {}; }
  Eigen::Vector<float, 3> timeUpdate(float, const TestField &, const float &,
                                     const double &,
                                     const int &) const override {
    std::cout << "i'm here" << std::endl;
    return {};
  }
};
template <std::size_t N, typename... Inputs>
void doIt(const test::HasModel<N, Inputs...> &hasModel) {
  hasModel.timeUpdate(1.0f, std::get<Inputs>(std::tuple<Inputs...>())...);
}

/**
 * this should already have everything including data
 */

int main() {
  test::TestPerformer<test::TestState> p{};
  doIt(TestField{});
  p.performOnModel(0.1f, TestField{}, 1);

  /*
  ukf::slam::SensorData<ukf::core::StaticFields<>,
                        ukf::slam::MapFields<ExpSlamSensorImpl>>
      slamSensor;
  slamSensor.addMeasurement<ExpSlamSensorImpl>(sensor_accel_t{}, 1);

  ukf::slam::State<ukf::core::StateFields<NoOpField>,
                   ukf::slam::MapFields<SlamFieldImpl>>
      X;
  X << 0.1f;

  X.add<SlamFieldImpl>(sensor_accel_t{}, 0);

  X.f(0.5);
*/
  ukf::core::State<ukf::core::StateFields<StandardFieldImpl, NoOpField>> Xcore;
  Xcore << 0.1f, 0.2f, 0.3f, 0.4f, 1.1f;

  // sensorData.h(Xcore);
  // slamSensor.h(X);

  ukf::core::Ukf<ukf::core::StateFields<NoOpField, StandardFieldImpl>> ukf;
  /* ukf::slam::Ukf<ukf::core::StateFields<NoOpField>,
                  ukf::slam::MapFields<SlamFieldImpl>>
       slamUkf{};
   slamUkf.step(slamSensor, 0.5);
 */
  //  ukf.step(sensorData, 0.5);

  Xcore.f(0.5);
}
