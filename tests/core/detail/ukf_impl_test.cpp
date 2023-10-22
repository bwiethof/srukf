// Include GTest
#include <gmock/gmock.h>
#include <gtest/gtest.h>
// Include the header that contains the class to test
#include "core/detail/ukf_impl.hpp"
#include "core/parameters.hpp"
#include "core/state.hpp"
#include "core/covariance.hpp"

using ::testing::AtLeast;
using ::testing::Return;


class UKFMock : public ukf::core::detail::Ukf {

public:
    explicit UKFMock(ukf::core::UkfParameters &&parameters) : ukf::core::detail::Ukf(std::move(parameters)) {}

    using ukf::core::detail::Ukf::timeStep;

    MOCK_METHOD(Eigen::MatrixXf, getSystemNoise, (), (const));
};


struct DataType {};

struct MockModel : public ukf::core::Model<3> {

    MOCK_METHOD((Eigen::Matrix<float, 3, 3>), noising, (), (const));
    MOCK_METHOD((Eigen::Vector<float, 3>), timeUpdate, (float), (const));
};


// TODO: remove FieldSize  by using MockModel Size
using MockField1 = ukf::core::Field<3, MockModel>;


// Test prediction part of Ukf.timeStep method
TEST(UkfTest, TimeStep) {
    // Arrange
    // Create an instance of your class
    ukf::core::UkfParameters parameters({0, 0, 0});
    UKFMock myUkf(ukf::core::UkfParameters({0, 0, 0}));

    const Eigen::MatrixXf systemNoising = Eigen::MatrixXf::Identity(3, 3);
    EXPECT_CALL(myUkf, getSystemNoise()).Times(AtLeast(1))
                                        .WillRepeatedly(Return(systemNoising));

    // Initialize the parameters
    ukf::core::State<ukf::core::StateFields<MockField1>> X(
            Eigen::Vector3f(1, 2, 3)); // Where 3 is the number of rows and columns of your matrix
    EXPECT_CALL(X.field<MockField1>()
                 .model, timeUpdate(1.0)).Times(AtLeast(1))
                                         .WillRepeatedly(Return(Eigen::Vector3f(1, 2, 3)));


    EXPECT_CALL(X.field<MockField1>()
                 .model, noising()).Times(AtLeast(1))
                                   .WillRepeatedly(Return(Eigen::Matrix3f::Identity()));

    ukf::core::Covariance<ukf::core::StateFields<MockField1>> P(Eigen::MatrixXf::Identity(3, 3));
    double dt = 1.0;
    ukf::core::SensorData<ukf::core::StaticFields<>> sensorData; // You need to define your SensorData

    // Act
    auto result = myUkf.timeStep(X, P, dt, sensorData);

    // Assert
    // Write your assertions here to test whether the returned result is as we expected
    // You may need to use Google Test's floating point comparison ASSERT_FLOAT_EQ(a, b) for results as these results are calculated using floating point arithmetic
    // As an example:
    // The following assertion is just an example and might not make sense with your calculations.
    // Adjust your assertions according to your needs.
    ASSERT_FLOAT_EQ(result.first(0, 0), X(0, 0));
    ASSERT_FLOAT_EQ(result.second(0, 0), P(0, 0));
}