#ifndef FSAE2026_ELECTRIC_H
#define FSAE2026_ELECTRIC_H

#include "src/core/actuators/electric_motor.h"
#include "src/core/tire/tire_frucd_p6_mnc.h"
#include "src/core/vehicles/limebeer2014f1.h"

// Formula SAE Electric 3-DOF vehicle using the fitted FRUCD MF6.2 + MNC
// tires and a rear electric powertrain.  The state/control signature remains
// identical to the formula-car model; battery discharge and regeneration are
// quasi-static power limits and therefore do not add an SOC state.
template<typename Timeseries_t>
using fsae2026_electric = formula_car_3dof<
    Timeseries_t,Tire_frucd_p6_mnc_left,Tire_frucd_p6_mnc_right,
    Electric_motor>;

struct fsae2026_electric_all
{
    fsae2026_electric_all(Xml_document& database_xml)
        : cartesian_scalar(database_xml),
          curvilinear_scalar(database_xml),
          cartesian_ad(database_xml),
          curvilinear_ad(database_xml)
    {}

    fsae2026_electric_all()
        : cartesian_scalar(),
          curvilinear_scalar(),
          cartesian_ad(),
          curvilinear_ad()
    {}

    using vehicle_scalar_curvilinear =
        fsae2026_electric<scalar>::curvilinear_p;
    using vehicle_ad_curvilinear =
        fsae2026_electric<CppAD::AD<scalar>>::curvilinear_p;

    vehicle_ad_curvilinear& get_curvilinear_ad_car()
    {
        return curvilinear_ad;
    }

    vehicle_scalar_curvilinear& get_curvilinear_scalar_car()
    {
        return curvilinear_scalar;
    }

    template<typename T>
    void set_parameter(const std::string& parameter, const T value)
    {
        cartesian_scalar.set_parameter(parameter,value);
        curvilinear_scalar.set_parameter(parameter,value);
        cartesian_ad.set_parameter(parameter,value);
        curvilinear_ad.set_parameter(parameter,value);
    }

    template<typename ... Args>
    void add_parameter(const std::string& parameter_name, Args&& ... args)
    {
        cartesian_scalar.add_parameter(
            parameter_name,std::forward<Args>(args)...);
        curvilinear_scalar.add_parameter(
            parameter_name,std::forward<Args>(args)...);
        cartesian_ad.add_parameter(
            parameter_name,std::forward<Args>(args)...);
        curvilinear_ad.add_parameter(
            parameter_name,std::forward<Args>(args)...);
    }

    fsae2026_electric<scalar>::cartesian                 cartesian_scalar;
    fsae2026_electric<scalar>::curvilinear_p             curvilinear_scalar;
    fsae2026_electric<CppAD::AD<scalar>>::cartesian      cartesian_ad;
    fsae2026_electric<CppAD::AD<scalar>>::curvilinear_p  curvilinear_ad;
};

#endif
