#include "gtest/gtest.h"

#include "src/core/vehicles/fsae2026.h"
#include "src/core/vehicles/fsae2026_electric.h"
#include "src/core/vehicles/fsae2026_reduced.h"
#include "src/core/applications/steady_state.h"
#include "src/main/c/fastestlapc.h"

#include "lion/foundation/constants.h"

#include <array>
#include <cmath>
#include <filesystem>
#include <string>

namespace
{

void configure_fsae_database_test_fit(Xml_document& database)
{
#ifdef FRUCD_TIRE_TEST_FILE
    database.get_element("vehicle/front-tire/mat-file").set_value(
        std::string(FRUCD_TIRE_TEST_FILE));
    database.get_element("vehicle/rear-tire/mat-file").set_value(
        std::string(FRUCD_TIRE_TEST_FILE));
#endif
}

template<typename Car_t>
void expect_finite_fsae_evaluation(Car_t& car)
{
    std::array<double,Car_t::number_of_inputs> q{};
    const auto defaults = car.get_state_and_control_upper_lower_and_default_values();
    auto u = defaults.controls_def;
    using Chassis = typename Car_t::Chassis_type;

    q[Chassis::front_axle_type::input_names::KAPPA_LEFT] = 0.0;
    q[Chassis::front_axle_type::input_names::KAPPA_RIGHT] = 0.0;
    q[Chassis::rear_axle_type::input_names::KAPPA_LEFT] = 0.0;
    q[Chassis::rear_axle_type::input_names::KAPPA_RIGHT] = 0.0;
    q[Chassis::input_names::force_z_fl_g] = -0.1125;
    q[Chassis::input_names::force_z_fr_g] = -0.1125;
    q[Chassis::input_names::force_z_rl_g] = -0.1375;
    q[Chassis::input_names::force_z_rr_g] = -0.1375;
    q[Chassis::input_names::velocity_x_mps] = 20.0;

    const auto [states,dstates] = car(q,u,0.0);
    for (const auto value : states)
        EXPECT_TRUE(std::isfinite(value));
    for (const auto value : dstates)
        EXPECT_TRUE(std::isfinite(value));

    const auto outputs = car.get_outputs_map();
    EXPECT_TRUE(std::isfinite(outputs.at("front-axle.left-tire.force.x")));
    EXPECT_TRUE(std::isfinite(outputs.at("rear-axle.right-tire.force.y")));
}

} // namespace

TEST(Fsae2026_reduced_vehicle_test, constructs_both_reduced_tire_variants)
{
    Xml_document standard_database(
        "./database/vehicles/fsae/fsae-2026-pacejka.xml",true);
    fsae2026_pacejka<double>::cartesian standard_car(standard_database);
    EXPECT_TRUE(standard_car.is_ready());
    EXPECT_EQ(decltype(standard_car)::number_of_inputs,14u);
    EXPECT_EQ(decltype(standard_car)::number_of_controls,4u);
    const auto& standard_model = standard_car.get_chassis().get_front_axle()
        .get_tire<0>().get_model();
    EXPECT_GT(standard_model.maximum_kappa(700.0),0.0);
    EXPECT_TRUE(std::isfinite(standard_model.maximum_kappa(700.0)));
    EXPECT_GT(standard_model.maximum_lambda(700.0),0.0);
    EXPECT_TRUE(std::isfinite(standard_model.maximum_lambda(700.0)));
    EXPECT_GT(standard_model.maximum_lambda(0.0),0.0);
    EXPECT_TRUE(std::isfinite(standard_model.maximum_lambda(0.0)));
    expect_finite_fsae_evaluation(standard_car);

    Xml_document simple_database(
        "./database/vehicles/fsae/fsae-2026-pacejka-simple.xml",true);
    fsae2026_pacejka_simple<double>::cartesian simple_car(simple_database);
    EXPECT_TRUE(simple_car.is_ready());
    EXPECT_EQ(decltype(simple_car)::number_of_inputs,14u);
    EXPECT_EQ(decltype(simple_car)::number_of_controls,4u);
    expect_finite_fsae_evaluation(simple_car);
}

TEST(Fsae2026_reduced_vehicle_test, both_variants_are_available_through_c_api)
{
    struct Case
    {
        const char* type;
        const char* name;
        const char* xml;
    };
    const std::array<Case,2> cases = {{
        {"fsae-pacejka-3dof","fsae-pacejka-test",
         "./database/vehicles/fsae/fsae-2026-pacejka.xml"},
        {"fsae-pacejka-simple-3dof","fsae-pacejka-simple-test",
         "./database/vehicles/fsae/fsae-2026-pacejka-simple.xml"}
    }};

    for (const auto& c : cases)
    {
        int n_inputs = 0;
        int n_controls = 0;
        int n_outputs = 0;
        vehicle_type_get_sizes(&n_inputs,&n_controls,&n_outputs,c.type);
        EXPECT_EQ(n_inputs,14);
        EXPECT_EQ(n_controls,4);
        EXPECT_GT(n_outputs,0);

        create_vehicle_from_xml(c.name,c.xml);
        char type[40] = {};
        variable_type(type,static_cast<int>(sizeof(type)),c.name);
        EXPECT_STREQ(type,c.type);
        delete_variable(c.name);
    }
}

TEST(Fsae2026_vehicle_test, constructs_and_evaluates_with_frucd_tires)
{
#ifndef FRUCD_TIRE_TEST_FILE
    GTEST_SKIP() << "FRUCD_TIRE_TEST_FILE was not configured";
#else
    Xml_document database("./database/vehicles/fsae/fsae-2026-frucd.xml",true);
    configure_fsae_database_test_fit(database);
    fsae2026<double>::cartesian car(database);

    EXPECT_TRUE(car.is_ready());
    EXPECT_EQ(decltype(car)::number_of_inputs,14u);
    EXPECT_EQ(decltype(car)::number_of_controls,4u);
    EXPECT_NEAR(car.get_chassis().get_mass(),280.0,1.0e-12);
    EXPECT_NEAR(car.get_chassis().get_front_axle().get_track(),1.250,1.0e-12);
    EXPECT_NEAR(car.get_chassis().get_rear_axle().get_track(),1.210,1.0e-12);
    EXPECT_DOUBLE_EQ(car.get_chassis().get_front_axle().get_tire<0>()
        .get_model().longitudinal_force_correction_factor(),0.7);
    EXPECT_DOUBLE_EQ(car.get_chassis().get_front_axle().get_tire<0>()
        .get_model().lateral_force_correction_factor(),0.7);

    car.set_parameter(
        "vehicle/front-tire/longitudinal-force-correction-factor",0.65);
    EXPECT_DOUBLE_EQ(car.get_chassis().get_front_axle().get_tire<0>()
        .get_model().longitudinal_force_correction_factor(),0.65);
    car.set_parameter(
        "vehicle/front-tire/longitudinal-force-correction-factor",0.7);

    std::array<double,decltype(car)::number_of_inputs> q{};
    const auto defaults = car.get_state_and_control_upper_lower_and_default_values();
    auto u = defaults.controls_def;

    using Chassis = typename decltype(car)::Chassis_type;
    q[Chassis::front_axle_type::input_names::KAPPA_LEFT] = 0.0;
    q[Chassis::front_axle_type::input_names::KAPPA_RIGHT] = 0.0;
    q[Chassis::rear_axle_type::input_names::KAPPA_LEFT] = 0.0;
    q[Chassis::rear_axle_type::input_names::KAPPA_RIGHT] = 0.0;
    q[Chassis::input_names::force_z_fl_g] = -0.1125;
    q[Chassis::input_names::force_z_fr_g] = -0.1125;
    q[Chassis::input_names::force_z_rl_g] = -0.1375;
    q[Chassis::input_names::force_z_rr_g] = -0.1375;
    q[Chassis::input_names::velocity_x_mps] = 20.0;
    q[Chassis::input_names::velocity_y_mps] = 0.0;
    q[Chassis::input_names::yaw_rate_radps] = 0.0;

    const auto [states,dstates] = car(q,u,0.0);
    for (const auto value : states)
        EXPECT_TRUE(std::isfinite(value));
    for (const auto value : dstates)
        EXPECT_TRUE(std::isfinite(value));

    const auto outputs = car.get_outputs_map();
    EXPECT_TRUE(std::isfinite(outputs.at("front-axle.left-tire.force.x")));
    EXPECT_TRUE(std::isfinite(outputs.at("rear-axle.right-tire.force.y")));
#endif
}

TEST(Fsae2026_vehicle_test, is_available_through_c_and_python_api_type_registry)
{
#ifndef FRUCD_TIRE_TEST_FILE
    GTEST_SKIP() << "FRUCD_TIRE_TEST_FILE was not configured";
#else
    Xml_document database("./database/vehicles/fsae/fsae-2026-frucd.xml",true);
    configure_fsae_database_test_fit(database);
    const auto runtime_xml = std::filesystem::temp_directory_path()
        / "fsae-2026-frucd-test.xml";
    database.save(runtime_xml.string());

    int n_inputs = 0;
    int n_controls = 0;
    int n_outputs = 0;
    vehicle_type_get_sizes(&n_inputs,&n_controls,&n_outputs,"fsae-3dof");
    EXPECT_EQ(n_inputs,14);
    EXPECT_EQ(n_controls,4);
    EXPECT_GT(n_outputs,0);

    create_vehicle_from_xml("fsae-test",runtime_xml.string().c_str());
    char type[32] = {};
    variable_type(type,static_cast<int>(sizeof(type)),"fsae-test");
    EXPECT_STREQ(type,"fsae-3dof");
    delete_variable("fsae-test");
#endif
}

TEST(Fsae2026_electric_vehicle_test, constructs_and_evaluates_motor_and_regeneration)
{
#ifndef FRUCD_TIRE_TEST_FILE
    GTEST_SKIP() << "FRUCD_TIRE_TEST_FILE was not configured";
#else
    Xml_document database(
        "./database/vehicles/fsae/fsae-2026-electric-frucd.xml",true);
    configure_fsae_database_test_fit(database);
    fsae2026_electric<double>::cartesian car(database);

    EXPECT_TRUE(car.is_ready());
    EXPECT_EQ(decltype(car)::number_of_inputs,14u);
    EXPECT_EQ(decltype(car)::number_of_controls,4u);
    EXPECT_NEAR(car.get_chassis().get_mass(),300.0,1.0e-12);

    std::array<double,decltype(car)::number_of_inputs> q{};
    const auto defaults = car.get_state_and_control_upper_lower_and_default_values();
    auto u = defaults.controls_def;

    using Chassis = typename decltype(car)::Chassis_type;
    q[Chassis::front_axle_type::input_names::KAPPA_LEFT] = 0.0;
    q[Chassis::front_axle_type::input_names::KAPPA_RIGHT] = 0.0;
    q[Chassis::rear_axle_type::input_names::KAPPA_LEFT] = 0.0;
    q[Chassis::rear_axle_type::input_names::KAPPA_RIGHT] = 0.0;
    q[Chassis::input_names::force_z_fl_g] = -0.1125;
    q[Chassis::input_names::force_z_fr_g] = -0.1125;
    q[Chassis::input_names::force_z_rl_g] = -0.1375;
    q[Chassis::input_names::force_z_rr_g] = -0.1375;
    q[Chassis::input_names::velocity_x_mps] = 20.0;
    q[Chassis::input_names::velocity_y_mps] = 0.0;
    q[Chassis::input_names::yaw_rate_radps] = 0.0;

    u[Chassis::control_names::throttle] = 1.0;
    const auto [traction_states,traction_derivatives] = car(q,u,0.0);
    for (const auto value : traction_states)
        EXPECT_TRUE(std::isfinite(value));
    for (const auto value : traction_derivatives)
        EXPECT_TRUE(std::isfinite(value));

    const auto& motor = car.get_chassis().get_rear_axle().get_engine();
    EXPECT_GT(motor.get_wheel_torque(),0.0);
    EXPECT_GT(motor.get_power(),0.0);
    EXPECT_LE(motor.get_power(),80.0e3 + 1.0e-9);

    u[Chassis::control_names::throttle] = -0.5;
    const auto [regenerative_states,regenerative_derivatives] = car(q,u,0.0);
    for (const auto value : regenerative_states)
        EXPECT_TRUE(std::isfinite(value));
    for (const auto value : regenerative_derivatives)
        EXPECT_TRUE(std::isfinite(value));
    EXPECT_LT(motor.get_wheel_torque(),0.0);
    EXPECT_LT(motor.get_power(),0.0);
    EXPECT_GE(motor.get_power(),-40.0e3 - 1.0e-9);
#endif
}

TEST(Fsae2026_electric_vehicle_test, is_available_through_c_and_python_api_type_registry)
{
#ifndef FRUCD_TIRE_TEST_FILE
    GTEST_SKIP() << "FRUCD_TIRE_TEST_FILE was not configured";
#else
    Xml_document database(
        "./database/vehicles/fsae/fsae-2026-electric-frucd.xml",true);
    configure_fsae_database_test_fit(database);
    const auto runtime_xml = std::filesystem::temp_directory_path()
        / "fsae-2026-electric-frucd-test.xml";
    database.save(runtime_xml.string());

    int n_inputs = 0;
    int n_controls = 0;
    int n_outputs = 0;
    vehicle_type_get_sizes(
        &n_inputs,&n_controls,&n_outputs,"fsae-electric-3dof");
    EXPECT_EQ(n_inputs,14);
    EXPECT_EQ(n_controls,4);
    EXPECT_GT(n_outputs,0);

    create_vehicle_from_xml(
        "fsae-electric-test",runtime_xml.string().c_str());
    char type[32] = {};
    variable_type(type,static_cast<int>(sizeof(type)),"fsae-electric-test");
    EXPECT_STREQ(type,"fsae-electric-3dof");
    delete_variable("fsae-electric-test");
#endif
}

TEST(Fsae2026_electric_vehicle_test, longitudinal_extrema_converge_with_mf62_mnc)
{
#ifndef FRUCD_TIRE_TEST_FILE
    GTEST_SKIP() << "FRUCD_TIRE_TEST_FILE was not configured";
#else
    Xml_document database(
        "./database/vehicles/fsae/fsae-2026-electric-frucd.xml",true);
    configure_fsae_database_test_fit(database);

    using Vehicle = fsae2026_electric<CppAD::AD<double>>::cartesian;
    Vehicle car(database);
    static_assert(Vehicle::steady_state_prefers_limited_memory_hessian);

    auto [maximum,minimum] =
        Steady_state<Vehicle>(car).solve_max_lon_acc(60.0*KMH,0.0);

    EXPECT_TRUE(maximum.solved);
    EXPECT_TRUE(minimum.solved);
    EXPECT_TRUE(std::isfinite(maximum.ax));
    EXPECT_TRUE(std::isfinite(minimum.ax));
    EXPECT_GT(maximum.ax,0.5*g0);
    EXPECT_LT(minimum.ax,-0.3*g0);
#endif
}

TEST(Fsae2026_electric_vehicle_test, gg_diagram_upper_branch_is_continuous)
{
#ifndef FRUCD_TIRE_TEST_FILE
    GTEST_SKIP() << "FRUCD_TIRE_TEST_FILE was not configured";
#else
    Xml_document database(
        "./database/vehicles/fsae/fsae-2026-electric-frucd.xml",true);
    configure_fsae_database_test_fit(database);

    const auto runtime_xml = std::filesystem::temp_directory_path()
        / "fsae-2026-electric-gg-test.xml";
    database.save(runtime_xml.string());

    constexpr std::size_t n_points = 50;
    std::array<double,n_points> ay{};
    std::array<double,n_points> ax_max{};
    std::array<double,n_points> ax_min{};
    create_vehicle_from_xml("fsae-electric-gg-test",runtime_xml.string().c_str());
    gg_diagram(
        ay.data(),ax_max.data(),ax_min.data(),"fsae-electric-gg-test",
        60.0*KMH,static_cast<int>(n_points));
    delete_variable("fsae-electric-gg-test");

    EXPECT_GT(ax_max.front(),0.5*g0);

    // The final two intervals close the envelope at the maximum lateral
    // acceleration and are physically steep. Everywhere else, continuation
    // should keep the upper branch free of isolated solver drop-outs.
    for (std::size_t i = 1; i + 2 < ax_max.size(); ++i)
    {
        EXPECT_TRUE(std::isfinite(ax_max[i]));
        EXPECT_LT(std::abs(ax_max[i] - ax_max[i-1]),0.15*g0)
            << "upper G-G branch discontinuity at point " << i;
    }
#endif
}
