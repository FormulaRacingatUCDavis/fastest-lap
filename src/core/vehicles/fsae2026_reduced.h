#ifndef FSAE2026_REDUCED_H
#define FSAE2026_REDUCED_H

#include "src/core/tire/tire_pacejka.h"
#include "src/core/vehicles/limebeer2014f1.h"

// Formula SAE 3-DOF variants using the two native reduced Pacejka models.
// Keeping distinct compile-time vehicle types is required because the tire
// implementation is a template parameter of formula_car_3dof.
template<typename Timeseries_t>
using fsae2026_pacejka = formula_car_3dof<
    Timeseries_t,Tire_pacejka_std,Tire_pacejka_std>;

template<typename Timeseries_t>
using fsae2026_pacejka_simple = formula_car_3dof<
    Timeseries_t,Tire_pacejka_simple,Tire_pacejka_simple>;

template<template<typename> class Vehicle_t>
struct fsae2026_reduced_all
{
    fsae2026_reduced_all(Xml_document& database_xml)
    : cartesian_scalar(database_xml),
      curvilinear_scalar(database_xml),
      cartesian_ad(database_xml),
      curvilinear_ad(database_xml)
    {}

    fsae2026_reduced_all()
    : cartesian_scalar(),
      curvilinear_scalar(),
      cartesian_ad(),
      curvilinear_ad()
    {}

    using vehicle_scalar_curvilinear = typename Vehicle_t<scalar>::curvilinear_p;
    using vehicle_ad_curvilinear = typename Vehicle_t<CppAD::AD<scalar>>::curvilinear_p;

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

    typename Vehicle_t<scalar>::cartesian                 cartesian_scalar;
    typename Vehicle_t<scalar>::curvilinear_p             curvilinear_scalar;
    typename Vehicle_t<CppAD::AD<scalar>>::cartesian      cartesian_ad;
    typename Vehicle_t<CppAD::AD<scalar>>::curvilinear_p  curvilinear_ad;
};

using fsae2026_pacejka_all = fsae2026_reduced_all<fsae2026_pacejka>;
using fsae2026_pacejka_simple_all = fsae2026_reduced_all<fsae2026_pacejka_simple>;

#endif
