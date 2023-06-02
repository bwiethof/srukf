
#include "include/slam/sensor_data.h"
#include "include/slam/sensor.hpp"

// Sample to test impl
// Example Data
struct sensor_accel_s {
    float x;
    float y;
    float z;
    float temperature;
};

struct sensor_accel_t {
    float x;
    float y;
    float z;
    float temperature;
};

using TestSlamSensorData = ukf::slam::Sensor<4, sensor_accel_s>;
using TestSlamSensorData2 = ukf::slam::Sensor<4, sensor_accel_t>;


struct CRTPSlamImpl : public ukf::slam::Sensor<4, sensor_accel_s> {
    using BaseType::Sensor;

protected:
    std::array<float, 4UL> zImpl(const sensor_accel_s &data) const override {
        return {data.x, data.y, data.z, data.temperature};
    }

    std::array<std::array<float, 4UL>, 4UL> RImpl(const sensor_accel_s &) const override {
        return {{}};
    }


};

using SlamSensorData = ukf::slam::SensorData<ukf::slam::NoOpSensor, CRTPSlamImpl>;


// End sample Data

int main() {
    SlamSensorData slamD;

    sensor_accel_s data3{0.1f, 0.2f, 0.3f, 0.4f};
//    ukf::NoOp_t noOp;


    CRTPSlamImpl sensor(data3, 1);
    sensor.R();


//    slamD.setMeasurement(data3, 3);
//    slamD.setMeasurement(noOp, 2);
//    slamD.getOrderedIds<CRTPSlamImpl>();
}
