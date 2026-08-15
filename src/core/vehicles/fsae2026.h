#ifndef FSAE2026_H
#define FSAE2026_H

#include "src/core/tire/tire_frucd_p6_mnc.h"
#include "src/core/vehicles/limebeer2014f1.h"

// Formula SAE 3-DOF vehicle using the fitted FRUCD MF6.2 + MNC tire model.
// The chassis equations are shared with the existing formula-car model; the
// tire types are selected at compile time so left/right moment conventions are
// preserved throughout Fastest-lap.
template<typename Timeseries_t>
using fsae2026 = formula_car_3dof<
    Timeseries_t,Tire_frucd_p6_mnc_left,Tire_frucd_p6_mnc_right>;

struct fsae2026_all
{
    fsae2026_all(Xml_document& database_xml)
    : cartesian_scalar(database_xml),
      curvilinear_scalar(database_xml),
      cartesian_ad(database_xml),
      curvilinear_ad(database_xml)
    {}

    fsae2026_all()
    : cartesian_scalar(),
      curvilinear_scalar(),
      cartesian_ad(),
      curvilinear_ad()
    {}

    using vehicle_scalar_curvilinear = fsae2026<scalar>::curvilinear_p;
    using vehicle_ad_curvilinear = fsae2026<CppAD::AD<scalar>>::curvilinear_p;

    vehicle_ad_curvilinear& get_curvilinear_ad_car() { return curvilinear_ad; }
    vehicle_scalar_curvilinear& get_curvilinear_scalar_car() { return curvilinear_scalar; }

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
        cartesian_scalar.add_parameter(parameter_name,std::forward<Args>(args)...);
        curvilinear_scalar.add_parameter(parameter_name,std::forward<Args>(args)...);
        cartesian_ad.add_parameter(parameter_name,std::forward<Args>(args)...);
        curvilinear_ad.add_parameter(parameter_name,std::forward<Args>(args)...);
    }

    fsae2026<scalar>::cartesian                 cartesian_scalar;
    fsae2026<scalar>::curvilinear_p             curvilinear_scalar;
    fsae2026<CppAD::AD<scalar>>::cartesian      cartesian_ad;
    fsae2026<CppAD::AD<scalar>>::curvilinear_p  curvilinear_ad;
};

#endif
