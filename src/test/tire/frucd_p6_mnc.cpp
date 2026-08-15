#include "gtest/gtest.h"

#include "src/core/tire/frucd_mat_tire_data.h"
#include "src/core/tire/tire_frucd_p6_mnc.h"
#include "src/main/c/fastestlapc.h"

#include <cppad/cppad.hpp>

#include <array>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace
{

std::shared_ptr<const frucd::P6_mnc_parameters> load_target_parameters()
{
#ifdef FRUCD_TIRE_TEST_FILE
    return frucd::Mat_tire_data::load(FRUCD_TIRE_TEST_FILE);
#else
    return {};
#endif
}

std::shared_ptr<const frucd::P6_mnc_parameters> make_synthetic_parameters()
{
    auto p = std::make_shared<frucd::P6_mnc_parameters>();
    p->nominal_vertical_load = 1000.0;
    p->nominal_pressure_kpa = 80.0;
    p->unloaded_radius_m = 0.25;
    p->nominal_velocity_mps = 15.0;

    p->p_Cx = 1.65;
    p->p_Dx = {1.20,-0.05,0.0};
    p->p_Ex = {0.20,0.0,0.0,0.0};
    p->p_Kx = {20.0,0.0,0.0};

    p->p_Cy = 1.30;
    p->p_Dy = {1.10,-0.05,0.0};
    p->p_Ey = {-1.0,0.0,0.0,0.0,0.0};
    p->p_Ky = {20.0,1.5,0.0,1.0,0.0,0.0,0.0};
    return p;
}

#if defined(TEST_LIBFASTESTLAPC) && defined(FRUCD_TIRE_TEST_FILE)
std::string last_c_api_error()
{
    const int required_size = fastestlap_last_error(nullptr,0);
    std::vector<char> buffer(static_cast<std::size_t>(required_size)+1u,'\0');
    fastestlap_last_error(buffer.data(),static_cast<int>(buffer.size()));
    return buffer.data();
}
#endif

class Frucd_p6_mnc_test : public ::testing::Test
{
 protected:
    void SetUp() override
    {
        target_parameters = load_target_parameters();
        parameters = target_parameters ? target_parameters : make_synthetic_parameters();
    }

    std::shared_ptr<const frucd::P6_mnc_parameters> target_parameters;
    std::shared_ptr<const frucd::P6_mnc_parameters> parameters;
};

TEST_F(Frucd_p6_mnc_test, loads_compressed_mat_v5_schema)
{
    if (!target_parameters)
        GTEST_SKIP() << "Configure with -DFRUCD_TIRE_TEST_FILE=<fitted.mat> to run MAT-file regression tests";

    EXPECT_DOUBLE_EQ(target_parameters->nominal_vertical_load,3000.0);
    EXPECT_DOUBLE_EQ(target_parameters->nominal_pressure_kpa,70.0);
    EXPECT_DOUBLE_EQ(target_parameters->unloaded_radius_m,0.2032);
    EXPECT_DOUBLE_EQ(target_parameters->nominal_velocity_mps,15.0);
    EXPECT_NEAR(target_parameters->p_Cx,1.3238448807857677,1.0e-15);
    EXPECT_NEAR(target_parameters->p_Dy[0],1.9849700232188905,1.0e-15);
    EXPECT_NEAR(target_parameters->q_Bz[0],9.8114026422544427,1.0e-14);
    EXPECT_NEAR(target_parameters->q_sx[10],4.0047240306647494,1.0e-14);
    EXPECT_DOUBLE_EQ(target_parameters->p_PMx,5.0);

#ifdef FRUCD_TIRE_TEST_FILE
    EXPECT_EQ(target_parameters.get(),frucd::Mat_tire_data::load(FRUCD_TIRE_TEST_FILE).get());

    Tire_frucd_p6_mnc_left<double,0,0> direct(
        "FRUCD direct",FRUCD_TIRE_TEST_FILE,"");
    EXPECT_TRUE(direct.is_ready());
    EXPECT_DOUBLE_EQ(direct.get_radius(),target_parameters->unloaded_radius_m);
#endif
}

TEST_F(Frucd_p6_mnc_test, xml_constructor_uses_mat_radius_and_defaults)
{
    if (!target_parameters)
        GTEST_SKIP() << "Configure with -DFRUCD_TIRE_TEST_FILE=<fitted.mat> to run MAT-file regression tests";

#ifdef FRUCD_TIRE_TEST_FILE
    const auto target_file = std::filesystem::path(FRUCD_TIRE_TEST_FILE);
    const auto relative_mat_file = target_file.filename().string();
    Xml_document database((target_file.parent_path()/"vehicle.xml").string(),false);
    database.parse(std::string("<vehicle><tire type=\"normal\"><mat-file>")
                   + relative_mat_file
                   + "</mat-file></tire></vehicle>");

    Tire_frucd_p6_mnc_left<double,0,0> tire(
        "FRUCD XML tire",database,"vehicle/tire/");

    EXPECT_TRUE(tire.is_ready());
    EXPECT_DOUBLE_EQ(tire.get_radius(),parameters->unloaded_radius_m);
    EXPECT_DOUBLE_EQ(
        tire.get_model().longitudinal_force_correction_factor(),0.7);
    EXPECT_DOUBLE_EQ(
        tire.get_model().lateral_force_correction_factor(),0.7);

    tire.set_parameter(
        "vehicle/tire/longitudinal-force-correction-factor",0.5);
    tire.set_parameter(
        "vehicle/tire/lateral-force-correction-factor",0.6);
    EXPECT_DOUBLE_EQ(
        tire.get_model().longitudinal_force_correction_factor(),0.5);
    EXPECT_DOUBLE_EQ(
        tire.get_model().lateral_force_correction_factor(),0.6);

    Xml_document output;
    output.parse("<vehicle><tire></tire></vehicle>");
    tire.fill_xml(output);
    EXPECT_DOUBLE_EQ(output.get_element("vehicle/tire/radius").get_value(double()),
                     parameters->unloaded_radius_m);
    EXPECT_EQ(output.get_element("vehicle/tire/mat-file").get_value(std::string()),
              relative_mat_file);
    EXPECT_DOUBLE_EQ(output.get_element(
        "vehicle/tire/longitudinal-force-correction-factor").get_value(double()),0.5);
    EXPECT_DOUBLE_EQ(output.get_element(
        "vehicle/tire/lateral-force-correction-factor").get_value(double()),0.6);
#endif
}

TEST_F(Frucd_p6_mnc_test, matches_contact_patch_loads_matlab_goldens)
{
    if (!target_parameters)
        GTEST_SKIP() << "Configure with -DFRUCD_TIRE_TEST_FILE=<fitted.mat> to run MATLAB golden tests";

    frucd::P6_mnc_model<double> model(parameters);

    struct Golden_case
    {
        double alpha_deg;
        double kappa;
        double Fz;
        double pressure;
        double inclination_deg;
        double velocity;
        frucd::Tire_side side;
        double Fx;
        double Fy;
        double Mz;
        double Mx;
    };

    // Generated with Vehicle Model Resources/ContactPatchLoads.m using
    // Pure='Pacejka', Combined='MNC' and the configured Hoosier fit.
    const std::array<Golden_case,6> cases = {{
        { 0.0,  0.00,1000.0,70.0, 0.0,15.0,frucd::Tire_side::left,
          -39.6081426398609, 34.3059401170719,-1.28453227443846, 0.362432497971413},
        { 5.0,  0.00, 700.0,70.0, 0.0,15.0,frucd::Tire_side::left,
          -11.3394501073762,-1572.31682600527,24.0869193882927,-15.8434556951028},
        {-5.0,  0.00, 700.0,70.0, 0.0,15.0,frucd::Tire_side::left,
          -11.5023356189076,1626.17292117751,-31.6281260478134,16.4764182123826},
        { 5.0,  0.10, 700.0,70.0, 1.0,20.0,frucd::Tire_side::left,
          1283.64394141899,-1157.50487164814,3.05669832418567,-18.6631601881480},
        {-7.0, -0.08,1500.0,80.0,-2.0,12.0,frucd::Tire_side::right,
          -2035.53573719931,2691.11672959066,-31.3477851823841,45.0451085130213},
        { 3.0,  0.20,2200.0,60.0, 2.5,30.0,frucd::Tire_side::right,
          4375.65675122809,-1247.01460185103,1.57884767395003,-20.2221267548555}
    }};

    for (const auto& c : cases)
    {
        const auto loads = model.evaluate(
            c.alpha_deg*DEG,c.kappa,c.Fz,c.pressure,
            c.inclination_deg,c.velocity,c.side);

        EXPECT_NEAR(loads.Fx,0.7*c.Fx,2.0e-8);
        EXPECT_NEAR(loads.Fy,0.7*c.Fy,2.0e-8);
        EXPECT_NEAR(loads.Mz,c.Mz,2.0e-8);
        EXPECT_NEAR(loads.Mx,c.Mx,2.0e-8);
        EXPECT_DOUBLE_EQ(loads.My,0.0);
    }
}

TEST_F(Frucd_p6_mnc_test, c_and_python_api_matches_matlab_golden)
{
    if (!target_parameters)
        GTEST_SKIP() << "Configure with -DFRUCD_TIRE_TEST_FILE=<fitted.mat> to run C API regression tests";

#if defined(TEST_LIBFASTESTLAPC) && defined(FRUCD_TIRE_TEST_FILE)
    constexpr const char* tire_name = "FRUCD C API Hoosier(7)";
    ASSERT_EQ(create_tire_from_mat(tire_name,FRUCD_TIRE_TEST_FILE),0)
        << last_c_api_error();
    struct Tire_cleanup
    {
        const char* name;
        ~Tire_cleanup() { if (name != nullptr) delete_tire(name); }
    } cleanup{tire_name};

    std::array<double,5> loads = {};
    ASSERT_EQ(tire_get_contact_patch_loads(
        loads.data(),static_cast<int>(loads.size()),tire_name,
        5.0*DEG,0.0,700.0,70.0,0.0,15.0,"left"),0)
        << last_c_api_error();

    EXPECT_NEAR(loads[0],0.7*-11.3394501073762,2.0e-8);
    EXPECT_NEAR(loads[1],0.7*-1572.31682600527,2.0e-8);
    EXPECT_NEAR(loads[2],24.0869193882927,2.0e-8);
    EXPECT_NEAR(loads[3],-15.8434556951028,2.0e-8);
    EXPECT_DOUBLE_EQ(loads[4],0.0);

    ASSERT_EQ(tire_set_force_correction_factors(tire_name,0.5,0.6),0)
        << last_c_api_error();
    ASSERT_EQ(tire_get_contact_patch_loads(
        loads.data(),static_cast<int>(loads.size()),tire_name,
        5.0*DEG,0.0,700.0,70.0,0.0,15.0,"left"),0)
        << last_c_api_error();
    EXPECT_NEAR(loads[0],0.5*-11.3394501073762,2.0e-8);
    EXPECT_NEAR(loads[1],0.6*-1572.31682600527,2.0e-8);
    EXPECT_NEAR(loads[2],24.0869193882927,2.0e-8);
    EXPECT_NE(tire_set_force_correction_factors(tire_name,1.1,0.6),0);
    EXPECT_NE(last_c_api_error().find("[0,1]"),std::string::npos);

    std::array<char,32> variable_type_name = {};
    variable_type(variable_type_name.data(),static_cast<int>(variable_type_name.size()),tire_name);
    EXPECT_STREQ(variable_type_name.data(),"tire-frucd-p6-mnc");

    EXPECT_NE(tire_get_contact_patch_loads(
        loads.data(),static_cast<int>(loads.size()),tire_name,
        0.0,0.0,700.0,70.0,0.0,15.0,"invalid"),0);
    EXPECT_NE(last_c_api_error().find("left"),std::string::npos);

    EXPECT_EQ(delete_tire(tire_name),0) << last_c_api_error();
    cleanup.name = nullptr;
    EXPECT_NE(tire_get_contact_patch_loads(
        loads.data(),static_cast<int>(loads.size()),tire_name,
        0.0,0.0,700.0,70.0,0.0,15.0,"left"),0);
#else
    GTEST_SKIP() << "fastestlapc or FRUCD_TIRE_TEST_FILE is not available in this build";
#endif
}

TEST_F(Frucd_p6_mnc_test, regularizes_detachment_and_mnc_null)
{
    frucd::P6_mnc_model<double> model(parameters);

    const auto detached = model.evaluate(0.0,0.0,0.0,70.0,0.0,15.0,frucd::Tire_side::left);
    EXPECT_DOUBLE_EQ(detached.Fx,0.0);
    EXPECT_DOUBLE_EQ(detached.Fy,0.0);
    EXPECT_DOUBLE_EQ(detached.Mz,0.0);
    EXPECT_DOUBLE_EQ(detached.Mx,0.0);

    // ContactPatchLoads.m reaches a 0/0 singularity at the simultaneous
    // longitudinal and lateral null. The native evaluator defines a finite limit.
    const double alpha_null = target_parameters ? 0.036399479695538153*DEG : 0.0;
    const double kappa_null = target_parameters ? 0.00049930456334977926 : 0.0;
    const auto at_null = model.evaluate(
        alpha_null,
        kappa_null,
        700.0,70.0,0.0,15.0,frucd::Tire_side::left);
    EXPECT_TRUE(std::isfinite(at_null.Fx));
    EXPECT_TRUE(std::isfinite(at_null.Fy));
    EXPECT_TRUE(std::isfinite(at_null.Mz));
    EXPECT_TRUE(std::isfinite(at_null.Mx));

    const auto tiny_load = model.evaluate(
        0.1,0.1,1.0e-15,70.0,1.0,15.0,frucd::Tire_side::left);
    EXPECT_TRUE(std::isfinite(tiny_load.Fx));
    EXPECT_TRUE(std::isfinite(tiny_load.Fy));
    EXPECT_TRUE(std::isfinite(tiny_load.Mz));
    EXPECT_TRUE(std::isfinite(tiny_load.Mx));
}

TEST_F(Frucd_p6_mnc_test, supports_cppad_values_and_jacobian)
{
    using AD = CppAD::AD<double>;
    frucd::P6_mnc_model<AD> model(parameters);
    std::vector<AD> x = {5.0*DEG,0.1,700.0};
    CppAD::Independent(x);

    const auto loads = model.evaluate(
        x[0],x[1],x[2],AD(70.0),AD(1.0),AD(20.0),frucd::Tire_side::left);
    std::vector<AD> y = {loads.Fx,loads.Fy,loads.Mz,loads.Mx};

    CppAD::ADFun<double> function;
    function.Dependent(x,y);

    const std::vector<double> point = {5.0*DEG,0.1,700.0};
    const auto value = function.Forward(0,point);
    const auto jacobian = function.Jacobian(point);

    const auto reference = frucd::P6_mnc_model<double>(parameters).evaluate(
        point[0],point[1],point[2],70.0,1.0,20.0,frucd::Tire_side::left);
    EXPECT_NEAR(value[0],reference.Fx,2.0e-8);
    EXPECT_NEAR(value[1],reference.Fy,2.0e-8);
    EXPECT_NEAR(value[2],reference.Mz,2.0e-8);
    EXPECT_NEAR(value[3],reference.Mx,2.0e-8);
    ASSERT_EQ(jacobian.size(),12u);
    for (const auto derivative : jacobian)
        EXPECT_TRUE(std::isfinite(derivative));
}

TEST_F(Frucd_p6_mnc_test, keeps_cppad_null_and_detachment_finite)
{
    using AD = CppAD::AD<double>;
    frucd::P6_mnc_model<AD> model(parameters);
    std::vector<AD> x = {0.0,0.0,700.0};
    CppAD::Independent(x);
    const auto loads = model.evaluate(
        x[0],x[1],x[2],AD(70.0),AD(0.0),AD(15.0),frucd::Tire_side::left);
    std::vector<AD> y = {loads.Fx,loads.Fy,loads.Mz,loads.Mx};
    CppAD::ADFun<double> function;
    function.Dependent(x,y);

    const double alpha_null = target_parameters ? 0.036399479695538153*DEG : 0.0;
    const double kappa_null = target_parameters ? 0.00049930456334977926 : 0.0;
    for (const std::vector<double>& point : {
             std::vector<double>{alpha_null,kappa_null,700.0},
             std::vector<double>{0.0,0.0,0.0}})
    {
        const auto value = function.Forward(0,point);
        const auto jacobian = function.Jacobian(point);
        for (const auto component : value)
            EXPECT_TRUE(std::isfinite(component));
        for (const auto derivative : jacobian)
            EXPECT_TRUE(std::isfinite(derivative));
    }
}

TEST_F(Frucd_p6_mnc_test, implements_fastest_lap_3dof_signature_and_coordinates)
{
    using Tire_t = Tire_frucd_p6_mnc_left<double,0,0>;
    Tire_t tire("FRUCD left",parameters);
    sFrame road_frame;
    tire.get_frame().set_parent(road_frame);
    tire.get_frame().set_origin(
        {0.0,0.0,-parameters->unloaded_radius_m},
        {20.0,-20.0*std::tan(5.0*DEG),0.0},
        sFrame::Frame_velocity_types::parent_frame);
    tire.set_parameter("inclination",1.0);
    tire.set_parameter("pressure",70.0);

    const double Fz = 700.0;
    const double kappa = 0.1;
    tire.update(Fz,kappa/tire.get_model().maximum_kappa(Fz),road_frame);

    const auto reference = frucd::P6_mnc_model<double>(parameters).evaluate(
        5.0*DEG,kappa,Fz,70.0,1.0,20.0,frucd::Tire_side::left);

    EXPECT_NEAR(tire.get_kappa(),kappa,1.0e-14);
    EXPECT_NEAR(tire.get_lambda(),std::tan(5.0*DEG),1.0e-14);
    EXPECT_NEAR(tire.get_force()[X],reference.Fx,2.0e-8);
    EXPECT_NEAR(tire.get_force()[Y],-reference.Fy,2.0e-8);
    EXPECT_DOUBLE_EQ(tire.get_force()[Z],-700.0);

    const double radius = parameters->unloaded_radius_m;
    EXPECT_NEAR(tire.get_torque()[X],radius*reference.Fy+reference.Mx,2.0e-8);
    EXPECT_NEAR(tire.get_torque()[Y],radius*reference.Fx-reference.My,2.0e-8);
    EXPECT_NEAR(tire.get_torque()[Z],-reference.Mz,2.0e-8);
    EXPECT_NEAR(tire.get_longitudinal_torque_at_wheel_center(),
                -radius*reference.Fx+reference.My,2.0e-8);
}

TEST_F(Frucd_p6_mnc_test, preserves_fastest_lap_empty_constructor_signature)
{
    Tire_frucd_p6_mnc_left<double,0,0> tire(
        "FRUCD empty","vehicle/front-tire/");
    EXPECT_EQ(tire.get_path(),"vehicle/front-tire/");
    EXPECT_FALSE(tire.is_ready());
}

TEST_F(Frucd_p6_mnc_test, rejects_radius_override_that_would_desynchronize_the_fit)
{
    Tire_frucd_p6_mnc_left<double,0,0> tire("FRUCD left",parameters);
    EXPECT_THROW(tire.set_parameter("radius",0.3),fastest_lap_exception);
    EXPECT_DOUBLE_EQ(tire.get_radius(),parameters->unloaded_radius_m);
}

TEST_F(Frucd_p6_mnc_test, rejects_6dof_update_without_radial_stiffness)
{
    Tire_frucd_p6_mnc_left<double,0,0> tire("FRUCD left",parameters);
    sFrame inertial_frame;
    tire.get_frame().set_parent(inertial_frame);
    EXPECT_THROW(
        tire.update(
            {0.0,0.0,-parameters->unloaded_radius_m},
            {20.0,0.0,0.0},
            20.0/parameters->unloaded_radius_m),
        fastest_lap_exception);
}

TEST_F(Frucd_p6_mnc_test, implements_fastest_lap_6dof_signature)
{
    using Tire_t = Tire_frucd_p6_mnc_right<double,0,0>;
    Tire_t tire("FRUCD right",parameters);
    sFrame inertial_frame;
    tire.get_frame().set_parent(inertial_frame);
    tire.set_parameter("radial-stiffness",100000.0);
    tire.set_parameter("radial-damping",0.0);
    tire.set_parameter("Fz-max-ref2",0.0);

    const double radius = parameters->unloaded_radius_m;
    const double velocity = 20.0;
    tire.update(
        {0.0,0.0,-radius + 0.01},
        {velocity,-velocity*std::tan(3.0*DEG),0.0},
        (1.0 + 0.08)*velocity/radius);

    EXPECT_NEAR(tire.get_vertical_deformation(),0.01,1.0e-14);
    EXPECT_NEAR(tire.get_kappa(),0.08,1.0e-14);
    EXPECT_NEAR(tire.get_lambda(),std::tan(3.0*DEG),1.0e-14);
    EXPECT_NEAR(tire.get_force()[Z],-1000.0,1.0e-10);
    EXPECT_TRUE(std::isfinite(tire.get_force()[X]));
    EXPECT_TRUE(std::isfinite(tire.get_force()[Y]));
    EXPECT_TRUE(std::isfinite(tire.get_torque()[Z]));
}

TEST_F(Frucd_p6_mnc_test, keeps_zero_speed_6dof_update_finite)
{
    Tire_frucd_p6_mnc_left<double,0,0> tire("FRUCD left",parameters);
    sFrame inertial_frame;
    tire.get_frame().set_parent(inertial_frame);
    tire.set_parameter("radial-stiffness",100000.0);
    tire.set_parameter("radial-damping",0.0);
    tire.set_parameter("Fz-max-ref2",0.0);

    tire.update(
        {0.0,0.0,-parameters->unloaded_radius_m + 0.01},
        {0.0,0.0,0.0},
        0.0);

    EXPECT_DOUBLE_EQ(tire.get_kappa(),0.0);
    EXPECT_DOUBLE_EQ(tire.get_lambda(),0.0);
    EXPECT_TRUE(std::isfinite(tire.get_force()[X]));
    EXPECT_TRUE(std::isfinite(tire.get_force()[Y]));
    EXPECT_TRUE(std::isfinite(tire.get_torque()[Z]));
}

TEST_F(Frucd_p6_mnc_test, keeps_zero_speed_cppad_adapter_finite)
{
    using AD = CppAD::AD<double>;
    Tire_frucd_p6_mnc_left<AD,0,0> tire("FRUCD AD left",parameters);
    Frame<AD> inertial_frame;
    tire.get_frame().set_parent(inertial_frame);
    tire.set_parameter("radial-stiffness",100000.0);
    tire.set_parameter("radial-damping",0.0);
    tire.set_parameter("Fz-max-ref2",0.0);

    const AD radius = parameters->unloaded_radius_m;
    tire.update(
        Vector3d<AD>{AD(0.0),AD(0.0),-radius + AD(0.01)},
        Vector3d<AD>{AD(0.0),AD(0.0),AD(0.0)},
        AD(0.0));

    EXPECT_TRUE(std::isfinite(CppAD::Value(tire.get_kappa())));
    EXPECT_TRUE(std::isfinite(CppAD::Value(tire.get_lambda())));
    EXPECT_TRUE(std::isfinite(CppAD::Value(tire.get_force()[X])));
    EXPECT_TRUE(std::isfinite(CppAD::Value(tire.get_force()[Y])));
    EXPECT_TRUE(std::isfinite(CppAD::Value(tire.get_torque()[Z])));
}

} // namespace
