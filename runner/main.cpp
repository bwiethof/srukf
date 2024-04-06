#include <core/field.hpp>
#include <utility>

#include "core/state.hpp"
#include "implementations.h"
#include "slam/sensor_data.h"
#include "slam/ukf.hpp"
// Sample to test impl
// Example Data

namespace logging {

enum class LogLevel { Debug, Info, Warning, Error };
class BaseLogger {
 public:
  virtual ~BaseLogger() = default;
  virtual std::ostream& info() = 0;

  bool isEnabled(LogLevel level) { return _currentLevel >= level; }
  //  virtual std::ostream& debug() = 0;
  //  virtual std::ostream& warn() = 0;
  //  virtual std::ostream& error() = 0;
 private:
  LogLevel _currentLevel{};
};

class Logger : public BaseLogger {
 public:
  std::ostream& info() override { return std::cout << "[INFO] [<timestamp>] "; }
};

std::unique_ptr<BaseLogger> currentLogger = std::unique_ptr<Logger>();

template <typename T>
std::ostream& Info(T&& value) {
  auto& infoStream = currentLogger->info();
  return infoStream;
}

std::ostream& Debug();

std::ostream& Warn();

std::ostream& Error();

}  // namespace logging

namespace doublePrecision {
using SigmaPoints = Eigen::MatrixXd;
using Mean = Eigen::VectorXd;

template <typename Derived_X, typename Derived_P>
Eigen::MatrixXd drawSigmaPoints(const Eigen::MatrixBase<Derived_X>& X,
                                const Eigen::MatrixBase<Derived_P>& P,
                                const ukf::core::UkfParameters& parameters) {
  assert(X.rows() == P.rows());

  const auto L = X.size();  // missing sensor data for augmented

  const auto params = parameters.params();

  SigmaPoints sigmaPoints = SigmaPoints::Zero(L, 2 * L + 1);

  sigmaPoints.col(0) = X;
  for (long i = 0; i < L; ++i) {
    sigmaPoints.col(i + 1) = X + params.gamma * P.col(i);
    sigmaPoints.col(i + L + 1) = X - params.gamma * P.col(i);
  }

  return sigmaPoints;
}

template <typename Derived>
Mean calculateMean(const Eigen::MatrixBase<Derived>& sigmaPoints,
                   const ukf::core::UkfParameters& parameters) {
  const auto params = parameters.params();
  return std::accumulate(
      ++sigmaPoints.colwise().begin(), sigmaPoints.colwise().end(),
      Eigen::VectorXd(params.W0_s * sigmaPoints.col(0)),
      [weight = params.Wi](const auto& currentMean, const auto& sigmaPoint) {
        return currentMean + weight * sigmaPoint;
      });
}
}  // namespace doublePrecision

int main() {
  std::tuple<const int&> tp{1};
  std::get<const int&>(tp);

  ukf::core::UkfParameters params{{1e-3, 2, 0}};
  params.update(3);
  // Eigen::Vector3d state{1000.123f, 2000.456f, 3000.789f};
  Eigen::Vector<double, 3> initialState{4293286, 627853, 4659166};
  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 3000;
  // double
  const auto sigmaPoints =
      doublePrecision::drawSigmaPoints(initialState, cov, params);
  const auto mean = doublePrecision::calculateMean(sigmaPoints, params);

  // float
  const auto mathSigmaPoints =
      ukf::core::math::drawSigmaPoints(initialState, cov, params);
  const auto mathMean = ukf::core::math::calculateMean(mathSigmaPoints, params);

  std::cout << "mean" << mean << "\n";

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
  ukf::core::SensorData<ukf::core::StaticFields<>> sensor_data;

  // sensorData.h(Xcore);
  // slamSensor.h(X);

  NoOpField noOp;

  ukf::core::Ukf<ukf::core::StateFields<NoOpField, StandardFieldImpl>> ukf;
  ukf.step(sensor_data, 0.1f, 4, 1.f);
  /* ukf::slam::Ukf<ukf::core::StateFields<NoOpField>,
                  ukf::slam::MapFields<SlamFieldImpl>>
       slamUkf{};
   slamUkf.step(slamSensor, 0.5);
 */
  //  ukf.step(sensorData, 0.5);

  Xcore.f(0.5);
}
