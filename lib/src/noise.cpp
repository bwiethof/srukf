//
// Created by bwiethof on 3/5/23.
//

#include "../include/noise.hpp"
#include <numeric>

namespace ukf {

class NoiseImpl {

public:
  long dimension() const {
    const auto accumulator = [](long dim, const std::shared_ptr<Noise> &noise) {
      return dim + noise->dimension();
    };
    return std::accumulate(_noises.begin(), _noises.end(), 0L, accumulator);
  }

  Eigen::MatrixXd calculateNoise() const {
    const long dim = dimension();
    Eigen::MatrixXd noise(dim, dim);

    Eigen::Index idx = 0;
    for (const auto &currentNoise : _noises) {
      const long currentDimension = currentNoise->dimension();
      noise.block(idx, idx, currentDimension, currentDimension) =
          currentNoise->matrix();
      idx += currentDimension;
    }

    return noise;
  }

  void addNoise(const std::shared_ptr<Noise> &noise) {
    _noises.emplace_back(noise);
  }

private:
  std::vector<std::shared_ptr<Noise>> _noises{};
};

Eigen::MatrixXd SystemNoise::matrix() const { return _impl->calculateNoise(); }

void SystemNoise::addNoise(const std::shared_ptr<Noise> &noise) const {
  _impl->addNoise(noise);
}
SystemNoise::SystemNoise() : _impl(std::make_shared<NoiseImpl>()) {}

Eigen::MatrixXd MeasurementNoise::matrix() const {
  return _impl->calculateNoise();
}

void MeasurementNoise::addNoise(
    const std::shared_ptr<const Sensor> &sensor) const {
  _impl->addNoise(sensor->noise());
}

} // namespace ukf