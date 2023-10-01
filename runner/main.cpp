#include "core/state.hpp"
#include "core/sensor.hpp"
#include "slam/sensor_data.h"
#include "implementations.h"
#include "slam/covariance.h"
#include "slam/ukf.hpp"
// Sample to test impl
// Example Data






int main() {

    ukf::core::SensorData<ukf::core::StaticFields<ExpSensorImpl>> sensorData;
    sensorData.setMeasurement<ExpSensorImpl>(sensor_accel_t{});

    ukf::slam::SensorData<ukf::core::StaticFields<>, ukf::slam::MapFields<ExpSlamSensorImpl>> slamSensor;
    slamSensor.addMeasurement<ExpSlamSensorImpl>(sensor_accel_t{}, 1);


    ukf::slam::State<ukf::core::StateFields<NoOpField>, ukf::slam::MapFields<SlamFieldImpl>> X;
    X << 0.1f;

    X.add<SlamFieldImpl>(sensor_accel_t{}, 0);

    X.f(0.5);


    ukf::core::State<ukf::core::StateFields<StandardFieldImpl, NoOpField>> Xcore;
    Xcore << 0.1f, 0.2f, 0.3f, 0.4f, 1.1f;

    sensorData.h(Xcore);
    slamSensor.h(X);


    ukf::core::Ukf<ukf::core::StateFields<NoOpField, StandardFieldImpl>> ukf;
    ukf::slam::Ukf<ukf::core::StateFields<NoOpField>, ukf::slam::MapFields<SlamFieldImpl>> slamUkf{};

    slamUkf.step(slamSensor, 0.5);
    ukf.step(sensorData, 0.5);


    Xcore.f(0.5);

}
