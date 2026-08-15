#include "gtest/gtest.h"

#include "src/core/actuators/electric_motor.h"

namespace
{

void configure_motor_database(Xml_document& database)
{
    database.create_root_element("vehicle");
    const std::string path = "vehicle/rear-axle/motor/";
    database.add_element(path + "maximum-torque").set_value("100.0");
    database.add_element(path + "maximum-power").set_value("60.0");
    database.add_element(path + "maximum-speed").set_value("12000.0");
    database.add_element(path + "gear-ratio").set_value("4.0");
    database.add_element(path + "transmission-efficiency").set_value("0.95");
    database.add_element(path + "drive-efficiency").set_value("0.92");
    database.add_element(path + "battery-maximum-discharge-power").set_value("80.0");
    database.add_element(path + "regenerative-maximum-torque").set_value("50.0");
    database.add_element(path + "regenerative-minimum-speed").set_value("300.0");
    database.add_element(path + "battery-maximum-charge-power").set_value("30.0");
    database.add_element(path + "regenerative-efficiency").set_value("0.80");
}

} // namespace

TEST(Electric_motor_test, follows_constant_torque_and_power_envelope)
{
    Xml_document database;
    configure_motor_database(database);
    Electric_motor<scalar> motor(database,"vehicle/rear-axle/motor/",true);

    EXPECT_NEAR(motor(1.0,10.0),100.0*4.0*0.95,1.0e-12);
    EXPECT_NEAR(motor.get_motor_speed(),40.0,1.0e-12);
    EXPECT_NEAR(motor.get_motor_torque(),100.0,1.0e-12);

    const scalar axle_speed = 200.0;
    const scalar expected_motor_torque = 60.0e3/(4.0*axle_speed);
    EXPECT_NEAR(motor(1.0,axle_speed),
                expected_motor_torque*4.0*0.95,1.0e-10);
    EXPECT_NEAR(motor.get_power(),60.0e3/0.92,1.0e-8);

    EXPECT_DOUBLE_EQ(motor(1.0,400.0),0.0);
    EXPECT_DOUBLE_EQ(motor.get_power(),0.0);
}

TEST(Electric_motor_test, limits_battery_discharge_power)
{
    Xml_document database;
    configure_motor_database(database);
    Electric_motor<scalar> motor(database,"vehicle/rear-axle/motor/",true);
    motor.set_parameter(
        "vehicle/rear-axle/motor/battery-maximum-discharge-power",20.0);

    const scalar axle_speed = 200.0;
    const scalar shaft_power = 20.0e3*0.92;
    const scalar expected_wheel_torque =
        shaft_power/(4.0*axle_speed)*4.0*0.95;
    EXPECT_NEAR(motor(1.0,axle_speed),expected_wheel_torque,1.0e-10);
    EXPECT_NEAR(motor.get_power(),20.0e3,1.0e-9);
}

TEST(Electric_motor_test, supports_regenerative_braking)
{
    Xml_document database;
    configure_motor_database(database);
    Electric_motor<scalar> motor(database,"vehicle/rear-axle/motor/",true);

    EXPECT_NEAR(motor(-1.0,10.0),-50.0*4.0/0.95,1.0e-12);
    EXPECT_NEAR(motor.get_motor_torque(),-50.0,1.0e-12);
    EXPECT_NEAR(motor.get_power(),-50.0*40.0*0.80,1.0e-12);

    EXPECT_DOUBLE_EQ(motor(-1.0,1.0),0.0);
    EXPECT_DOUBLE_EQ(motor.get_power(),0.0);

    EXPECT_DOUBLE_EQ(motor(0.0,10.0),0.0);
    EXPECT_DOUBLE_EQ(motor.get_power(),0.0);
}

TEST(Electric_motor_test, is_compatible_with_cppad)
{
    Xml_document database;
    configure_motor_database(database);
    Electric_motor<CppAD::AD<scalar>> motor(
        database,"vehicle/rear-axle/motor/",true);

    const CppAD::AD<scalar> torque = motor(1.0,200.0);
    EXPECT_TRUE(std::isfinite(Value(torque)));
    EXPECT_GT(Value(torque),0.0);
}
