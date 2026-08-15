#ifndef TIRE_PACEJKA_HPP
#define TIRE_PACEJKA_HPP

#include "lion/math/optimise.h"
#include "lion/thirdparty/include/logger.hpp"
#include "src/core/foundation/fastest_lap_exception.h"

template<typename Timeseries_t, typename Pacejka_model, size_t state_start, size_t control_start>
inline Tire_pacejka<Timeseries_t,Pacejka_model,state_start,control_start>::Tire_pacejka(const std::string& name, Xml_document& database, 
                                  const std::string& path)
: base_type(name, database, path),
 _model(),
 _kt(0.0),
 _ct(0.0),
 _Fz_max_ref2(0.0),
 _Smagic(0.0),
 _Fmagic(0.0)
{
    // Get tire parameters
    read_parameters(database, path, get_parameters(), __used_parameters);

    // Get tire model parameters
    read_parameters(database, path, _model.get_parameters(), _model.__used_parameters);

    // Initialise tire model
    _model.initialise();
}

template<typename Timeseries_t, typename Pacejka_model, size_t state_start, size_t control_start>
template<typename T>
void Tire_pacejka<Timeseries_t,Pacejka_model,state_start,control_start>::set_parameter(const std::string& parameter, const T value)
{
    // Look for the parameter in the class
    if ( parameter.find(base_type::get_path()) == 0 )
    {
        auto found = ::set_parameter(get_parameters(), __used_parameters, parameter, base_type::get_path(), value);

        // Look for the parameter in the model
        if ( !found )
            found = ::set_parameter(_model.get_parameters(), _model.__used_parameters, parameter, base_type::get_path(), value);

        if ( !found )
            base_type::set_parameter(parameter, value);

        _model.initialise();
    }
    else
        throw fastest_lap_exception(std::string("Parameter \"") + parameter + "\" was not found in Tire_pacejka");
}


template<typename Timeseries_t, typename Pacejka_model, size_t state_start, size_t control_start>
void Tire_pacejka<Timeseries_t,Pacejka_model,state_start,control_start>::fill_xml(Xml_document& doc) const
{
    // Write the parameters of the parent 
    base_type::fill_xml(doc);

    // Write the owned parameters
    ::write_parameters(doc, base_type::get_path(), get_parameters());

    // Write the parameters of the model
    ::write_parameters(doc, base_type::get_path(), _model.get_parameters());
}


template<typename Timeseries_t, typename Pacejka_model, size_t state_start, size_t control_start>
inline void Tire_pacejka<Timeseries_t,Pacejka_model,state_start,control_start>::update(Timeseries_t Fz, Timeseries_t kappa_dimensionless, const Frame<Timeseries_t>& road_frame)
{
    // Compute omega
    base_type::update_from_kappa(kappa_dimensionless*_model.maximum_kappa(Fz), road_frame);

    update_self(Fz);
}


template<typename Timeseries_t, typename Pacejka_model, size_t state_start, size_t control_start>
inline void Tire_pacejka<Timeseries_t,Pacejka_model,state_start,control_start>::update(const Vector3d<Timeseries_t>& x0, const Vector3d<Timeseries_t>& v0, Timeseries_t omega)
{
    base_type::update(x0, v0, omega);

    update_self();
}


template<typename Timeseries_t, typename Pacejka_model, size_t state_start, size_t control_start>
inline void Tire_pacejka<Timeseries_t,Pacejka_model,state_start,control_start>::update(Timeseries_t omega)
{
    base_type::update(omega);

    update_self();
}


template<typename Timeseries_t, typename Pacejka_model, size_t state_start, size_t control_start>
inline void Tire_pacejka<Timeseries_t,Pacejka_model,state_start,control_start>::update_self()
{
    assert(_kt > 1.0e-12);

    // Compute the normal load based on tire carcass stiffness
    const Timeseries_t Fz = smooth_pos<Timeseries_t>(_kt*base_type::_w + _ct*base_type::_dw, _Fz_max_ref2);

    update_self(Fz);
}


template<typename Timeseries_t, typename Pacejka_model, size_t state_start, size_t control_start>
inline void Tire_pacejka<Timeseries_t,Pacejka_model,state_start,control_start>::update_self(const Timeseries_t Fz)
{
    // Compute the magic formula forces
    // For now, lets use a steady-state version
    _Smagic = _model.force_combined_longitudinal_magic(base_type::_kappa,base_type::_lambda,Fz);
    _Fmagic = _model.force_combined_lateral_magic(base_type::_kappa, base_type::_lambda, Fz);

    base_type::_F[X] = _Smagic; 
    base_type::_F[Y] = _Fmagic;
    base_type::_F[Z] = -Fz;

    // Compute the torque at the wheel centre
    base_type::_T = cross(base_type::get_contact_point(), base_type::_F);
}


inline void Pacejka_standard_model::initialise()
{
    _Fz0prime = _lambdaFz0*_Fz0;

    // At the force peak:
    // C*atan((1-E)*x + E*atan(x)) = pi/2, with x = B*slip.
    // The left-hand side is monotone for the supported E <= 1. A finite peak
    // exists only for C > 1. Store its dimensionless x once so the run-time
    // maximum-slip functions remain AD-compatible.
    const auto peak_argument = [](const scalar C, const scalar E)
    {
        if (C <= 1.0)
            return scalar(0.0);

        const scalar target = tan(pi/(2.0*C));
        const auto residual = [E,target](const scalar x)
        {
            return (1.0-E)*x + E*atan(x) - target;
        };

        scalar lower = 0.0;
        scalar upper = 1.0;
        while (residual(upper) < 0.0 && upper < 1.0e6)
            upper *= 2.0;
        if (residual(upper) < 0.0)
            return scalar(0.0);

        for (std::size_t i=0; i<80; ++i)
        {
            const scalar middle = 0.5*(lower+upper);
            if (residual(middle) < 0.0)
                lower = middle;
            else
                upper = middle;
        }
        return 0.5*(lower+upper);
    };

    _x_peak_longitudinal = peak_argument(_pCx1,_pEx1);
    _x_peak_lateral = peak_argument(_pCy1,_pEy1);
}


template<typename Timeseries_t>
inline Timeseries_t Pacejka_standard_model::force_pure_longitudinal_magic(Timeseries_t kappa, Timeseries_t Fz) const
{
    if ( _pDx1 == 0 )
        return 0.0; 

    const Timeseries_t& Cx = _pCx1;
    const Timeseries_t& mu_x = _pDx1;
    const Timeseries_t& Ex = _pEx1;
    const Timeseries_t  dfz = (Fz - _Fz0prime)/_Fz0prime;
    const Timeseries_t  Bx = (_pKx1 + _pKx2*dfz)*exp(_pKx3*dfz)/(_pCx1*_pDx1);

    return mu_x*Fz*sin(Cx*atan(Bx*kappa - Ex*(Bx*kappa - atan(Bx*kappa))));
}   


template<typename Timeseries_t>
inline Timeseries_t Pacejka_standard_model::force_pure_lateral_magic(Timeseries_t lambda, Timeseries_t Fz) const
{
    const Timeseries_t& Cy = _pCy1;
    const Timeseries_t& mu_y = _pDy1;
    const Timeseries_t& Ey = _pEy1;
    Timeseries_t By(0.0);

    if ( abs(Fz) > eps ) 
        By = (_pKy1*_Fz0prime*sin(_pKy4*atan(Fz/(_Fz0prime*_pKy2)))) / (_pCy1*_pDy1*Fz);
    else
        By = _pKy1*_pKy4 / (_pKy2*_pCy1*_pDy1);

    return mu_y*Fz*sin(Cy*atan(By*lambda - Ey*(By*lambda - atan(By*lambda))));
}


template<typename Timeseries_t>
inline Timeseries_t Pacejka_standard_model::force_combined_longitudinal_magic(Timeseries_t kappa, Timeseries_t lambda, Timeseries_t Fz) const
{
    const Timeseries_t Fx0 = force_pure_longitudinal_magic(kappa, Fz);
    const Timeseries_t Gxlambda = cos(_rCx1*atan(_rBx1*lambda));

    return Gxlambda*Fx0;
}


template<typename Timeseries_t>
inline Timeseries_t Pacejka_standard_model::force_combined_lateral_magic(Timeseries_t kappa, Timeseries_t lambda, Timeseries_t Fz) const
{
    const Timeseries_t Fy0 = force_pure_lateral_magic(lambda,Fz);
    const Timeseries_t Gykappa = cos(_rCy1*atan(_rBy1*kappa));

    return Gykappa*Fy0;
}


template<typename Timeseries_t>
inline Timeseries_t Pacejka_standard_model::maximum_kappa(Timeseries_t Fz) const
{
    using std::abs;
    // The reduced standard fit uses |kappa| <= 0.25. For Cx <= 1 the curve is
    // monotone and has no finite mathematical maximum, so its domain boundary
    // is the meaningful normalization value.
    if (_x_peak_longitudinal <= 0.0)
        return Timeseries_t(0.25);

    const Timeseries_t dfz = (Fz-_Fz0prime)/_Fz0prime;
    const Timeseries_t Bx = (_pKx1+_pKx2*dfz)*exp(_pKx3*dfz)/(_pCx1*_pDx1);
    return abs(Timeseries_t(_x_peak_longitudinal)/Bx);
}


template<typename Timeseries_t>
inline Timeseries_t Pacejka_standard_model::maximum_lambda(Timeseries_t Fz) const
{
    using std::abs;
    if (_x_peak_lateral <= 0.0)
        return Timeseries_t(0.32491969623290634); // tan(18 deg), reduction domain boundary

    Timeseries_t By(0.0);
    if (abs(Fz) > eps)
        By = (_pKy1*_Fz0prime*sin(_pKy4*atan(Fz/(_Fz0prime*_pKy2))))
             /(_pCy1*_pDy1*Fz);
    else
        By = _pKy1*_pKy4/(_pKy2*_pCy1*_pDy1);
    return abs(Timeseries_t(_x_peak_lateral)/By);
}


template<typename Timeseries_t>
inline void Pacejka_simple_model<Timeseries_t>::initialise()
{
    // Transform lambda_max_{1,2} to rad
    _lambda_max1 = _lambda_max1_deg*DEG; 
    _lambda_max2 = _lambda_max2_deg*DEG; 

    // Compute Sx and Sy
    _Sx = pi/(2.0*atan(_Qx));
    _Sy = pi/(2.0*atan(_Qy));
}


template<typename Timeseries_t>
Timeseries_t Pacejka_simple_model<Timeseries_t>::force_combined_longitudinal_magic(Timeseries_t kappa, Timeseries_t lambda, Timeseries_t Fz) const
{
    const Timeseries_t mu_x_max  = smooth_pos((Fz - _Fz1)*(_mu_x_max2  - _mu_x_max1 )/(_Fz2 - _Fz1) + _mu_x_max1-_mu_min,1.0e-5)+_mu_min;
    const Timeseries_t kappa_max = maximum_kappa(Fz);
    const Timeseries_t lambda_max = maximum_lambda(Fz);;

    const Timeseries_t kappa_n = kappa/kappa_max;
    const Timeseries_t lambda_n = lambda/lambda_max;
    const Timeseries_t rho = sqrt(kappa_n*kappa_n + lambda_n*lambda_n + 1.0e-12);

    const Timeseries_t mu_x = mu_x_max*sin(_Qx*atan(_Sx*rho));
    
    return mu_x*Fz*kappa_n/(rho);
}


template<typename Timeseries_t>
Timeseries_t Pacejka_simple_model<Timeseries_t>::force_combined_lateral_magic(Timeseries_t kappa, Timeseries_t lambda, Timeseries_t Fz) const
{
    const Timeseries_t mu_y_max  = smooth_pos((Fz - _Fz1)*(_mu_y_max2  - _mu_y_max1 )/(_Fz2 - _Fz1) + _mu_y_max1-_mu_min,1.0e-5)+_mu_min;
    const Timeseries_t kappa_max = maximum_kappa(Fz);
    const Timeseries_t lambda_max = maximum_lambda(Fz);;

    const Timeseries_t kappa_n = kappa/kappa_max;
    const Timeseries_t lambda_n = lambda/lambda_max;
    const Timeseries_t rho = sqrt(kappa_n*kappa_n + lambda_n*lambda_n + 1.0e-12);

    const Timeseries_t mu_y = mu_y_max*sin(_Qy*atan(_Sy*rho));
    
    return mu_y*Fz*lambda_n/(rho);
}


template<typename Timeseries_t, typename Pacejka_model, size_t state_start, size_t control_start>
inline std::ostream& Tire_pacejka<Timeseries_t,Pacejka_model,state_start,control_start>::print(std::ostream& os) const
{
    base_type::print(os);
    out(2) << std::left << std::setw(16) << "   * kt: "  << std::right << std::setw(5) << _kt << std::endl;
    out(2) << std::left << std::setw(16) << "   * ct: "  << std::right << std::setw(5) << _ct << std::endl;
    _model.print(os);

    return os;
}


inline std::ostream& Pacejka_standard_model::print(std::ostream& os) const
{
    out(2) << std::left << std::setw(16) << "   * Fz0: "  << std::right << std::setw(5) << _Fz0 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pCx1: "  << std::right << std::setw(5) << _pCx1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pDx1: "  << std::right << std::setw(5) << _pDx1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pEx1: "  << std::right << std::setw(5) << _pEx1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pKx1: "  << std::right << std::setw(5) << _pKx1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pKx2: "  << std::right << std::setw(5) << _pKx2 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pKx3: "  << std::right << std::setw(5) << _pKx3 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pCy1: "  << std::right << std::setw(5) << _pCy1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pDy1: "  << std::right << std::setw(5) << _pDy1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pEy1: "  << std::right << std::setw(5) << _pEy1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pKy1: "  << std::right << std::setw(5) << _pKy1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pKy2: "  << std::right << std::setw(5) << _pKy2 << std::endl;
    out(2) << std::left << std::setw(16) << "   * pKy4: "  << std::right << std::setw(5) << _pKy4 << std::endl;
    out(2) << std::left << std::setw(16) << "   * rBx1: "  << std::right << std::setw(5) << _rBx1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * rCx1: "  << std::right << std::setw(5) << _rCx1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * rBy1: "  << std::right << std::setw(5) << _rBy1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * rCy1: "  << std::right << std::setw(5) << _rCy1 << std::endl;
    out(2) << std::left << std::setw(16) << "   * lambdaFz0: " <<  std::right << std::setw(5) << _lambdaFz0 << std::endl;

    return os;
}

#endif
