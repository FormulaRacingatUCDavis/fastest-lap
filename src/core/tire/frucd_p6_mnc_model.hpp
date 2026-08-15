#ifndef FRUCD_P6_MNC_MODEL_HPP
#define FRUCD_P6_MNC_MODEL_HPP

#if __has_include(<cppad/cppad.hpp>)
# include <cppad/cppad.hpp>
# define FRUCD_P6_MNC_HAS_CPPAD 1
#else
# define FRUCD_P6_MNC_HAS_CPPAD 0
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace frucd
{
namespace detail
{

template<typename T>
T sign(const T& value)
{
    if constexpr (std::is_arithmetic<T>::value)
        return value > T(0) ? T(1) : (value < T(0) ? T(-1) : T(0));
#if FRUCD_P6_MNC_HAS_CPPAD
    else
        return CppAD::CondExpGt(value,T(0),T(1),
               CppAD::CondExpLt(value,T(0),T(-1),T(0)));
#endif
}

template<typename T>
T positive_load(const T& value)
{
    if constexpr (std::is_arithmetic<T>::value)
        return value;
#if FRUCD_P6_MNC_HAS_CPPAD
    else
        return CppAD::CondExpGt(value,T(0),value,T(1.0e-9));
#endif
}

template<typename T>
T load_activity(const T& value)
{
    if constexpr (std::is_arithmetic<T>::value)
        return value > T(0) ? T(1) : T(0);
#if FRUCD_P6_MNC_HAS_CPPAD
    else
        return CppAD::CondExpGt(value,T(0),T(1),T(0));
#endif
}

template<typename T>
T nonzero_denominator(const T& value)
{
    using std::abs;
    constexpr double epsilon = 1.0e-12;
    if constexpr (std::is_arithmetic<T>::value)
    {
        if (abs(value) >= epsilon)
            return value;
        return value < T(0) ? T(-epsilon) : T(epsilon);
    }
#if FRUCD_P6_MNC_HAS_CPPAD
    else
    {
        const T magnitude = CppAD::CondExpGt(
            abs(value),T(epsilon),abs(value),T(epsilon));
        const T direction = CppAD::CondExpGe(value,T(0),T(1),T(-1));
        return direction*magnitude;
    }
#endif
}

template<typename T>
struct Pure_longitudinal
{
    T force = 0.0;
    T stiffness = 0.0;
    T null_slip = 0.0;
};

template<typename T>
Pure_longitudinal<T> pure_longitudinal(const P6_mnc_parameters& p,
                                       const T& kappa,
                                       const T& Fz,
                                       const T& dFz,
                                       const T& dPi,
                                       const T& inclination_deg)
{
    using std::atan;
    using std::exp;
    using std::sin;

    const T Cx = p.p_Cx*p.scaling.Cx;
    const T Dx = (p.p_Dx[0] + p.p_Dx[1]*dFz)
        *(T(1) + p.p_Px[2]*dPi + p.p_Px[3]*dPi*dPi)
        *(T(1) - p.p_Dx[2]*inclination_deg*inclination_deg)
        *Fz*p.scaling.mux;
    const T Ex = (p.p_Ex[0] + p.p_Ex[1]*dFz + p.p_Ex[2]*dFz*dFz)
        *(T(1) - p.p_Ex[3]*sign(kappa))*p.scaling.Ex;
    const T Kxk = Fz*(p.p_Kx[0] + p.p_Kx[1]*dFz)
        *exp(p.p_Kx[2]*dFz)
        *(T(1) + p.p_Px[0]*dPi + p.p_Px[1]*dPi*dPi)
        *p.scaling.Kxk;
    const T Vx = Fz*(p.p_Vx[0] + p.p_Vx[1]*dFz)*p.scaling.Vx;
    const T Hx = (p.p_Hx[0] + p.p_Hx[1]*dFz)*p.scaling.Hx;
    const T Bx = Kxk/nonzero_denominator(Cx*Dx);
    const T shifted_kappa = kappa + Hx;

    Pure_longitudinal<T> result;
    result.force = Dx*sin(Cx*atan((T(1)-Ex)*Bx*shifted_kappa
                                 + Ex*atan(Bx*shifted_kappa))) + Vx;
    result.stiffness = Kxk;
    result.null_slip = -Vx/nonzero_denominator(Kxk) - Hx;
    return result;
}

template<typename T>
struct Pure_lateral
{
    T force = 0.0;
    T B = 0.0;
    T C = 0.0;
    T stiffness = 0.0;
    T H = 0.0;
    T V = 0.0;
    T null_slip = 0.0;
};

template<typename T>
Pure_lateral<T> pure_lateral(const P6_mnc_parameters& p,
                             const T& alpha,
                             const T& Fz,
                             const T& dFz,
                             const T& dPi,
                             const T& inclination_deg,
                             const T& side_sign)
{
    using std::abs;
    using std::atan;
    using std::sin;

    const T Cy = p.p_Cy;
    const T Dy = (p.p_Dy[0] + p.p_Dy[1]*dFz)
        *(T(1) + p.p_Py[2]*dPi + p.p_Py[3]*dPi*dPi)
        *(T(1) - p.p_Dy[2]*inclination_deg*inclination_deg)*Fz;
    const T pressure_denominator =
        (p.p_Ky[1] + p.p_Ky[4]*inclination_deg*inclination_deg)
        *(T(1) + p.p_Py[1]*dPi);
    const T Kya = p.p_Ky[0]*p.nominal_vertical_load
        *(T(1) + p.p_Py[0]*dPi)
        *(T(1) - p.p_Ky[2]*abs(inclination_deg))
        *sin(p.p_Ky[3]*atan((Fz/p.nominal_vertical_load)
                            /nonzero_denominator(pressure_denominator)));
    const T Kyg0 = Fz*(p.p_Ky[5] + p.p_Ky[6]*dFz)
        *(T(1) + p.p_Py[4]*dPi);
    const T By = Kya/nonzero_denominator(Cy*Dy);
    const T Vyg = Fz*(p.p_Vy[2] + p.p_Vy[3]*dFz)*inclination_deg;
    const T Vy = Fz*(p.p_Vy[0] + p.p_Vy[1]*dFz) + Vyg;
    const T Hy = (p.p_Hy[0] + p.p_Hy[1]*dFz)
        *(Kyg0*inclination_deg - Vyg)/nonzero_denominator(Kya);
    const T Ey = (p.p_Ey[0] + p.p_Ey[1]*dFz)
        *(T(1) + p.p_Ey[4]*inclination_deg*inclination_deg
          - (p.p_Ey[2] + p.p_Ey[3]*inclination_deg)*sign(alpha + Hy));
    const T shifted_alpha = alpha + Hy;

    Pure_lateral<T> result;
    result.force = (Dy*sin(Cy*atan((T(1)-Ey)*By*shifted_alpha
                                  + Ey*atan(By*shifted_alpha))) + Vy)*side_sign;
    result.B = By;
    result.C = Cy;
    result.stiffness = Kya;
    result.H = Hy;
    result.V = Vy;
    result.null_slip = -Vy/nonzero_denominator(Kya) - Hy;
    return result;
}

template<typename T>
struct Combined_forces
{
    T Fx = 0.0;
    T Fy = 0.0;
};

template<typename T>
Combined_forces<T> mnc(const T& alpha,
                       const T& kappa,
                       const T& Fx0,
                       const T& Fy0,
                       const T& Kxk,
                       const T& Kya,
                       const T& kappa0,
                       const T& alpha0)
{
    using std::abs;
    using std::cos;
    using std::sin;
    using std::sqrt;
    using std::tan;

    const T delta_kappa = kappa - kappa0;
    const T delta_alpha = alpha - alpha0;
    const T sin_alpha = sin(delta_alpha);
    const T cos_alpha = cos(delta_alpha);
    const T tan_alpha = tan(delta_alpha);
    const T denominator = sqrt(
        delta_kappa*delta_kappa*Fy0*Fy0
        + Fx0*Fx0*tan_alpha*tan_alpha + T(1.0e-24));
    const T shared = Fx0*Fy0/denominator;
    const T one_minus_abs_kappa = T(1) - abs(delta_kappa);

    const T fx_unsigned = abs(shared
        *sqrt(delta_kappa*delta_kappa*Kya*Kya
              + one_minus_abs_kappa*one_minus_abs_kappa*cos_alpha*cos_alpha*Fx0*Fx0
              + T(1.0e-24))
        /nonzero_denominator(Kya));
    const T fy_unsigned = abs(shared
        *sqrt(one_minus_abs_kappa*one_minus_abs_kappa*cos_alpha*cos_alpha*Fy0*Fy0
              + sin_alpha*sin_alpha*Kxk*Kxk + T(1.0e-24))
        /nonzero_denominator(Kxk*cos_alpha));

    return {fx_unsigned*sign(Fx0),fy_unsigned*sign(Fy0)};
}

template<typename T>
struct Moment_terms
{
    T Bt = 0.0;
    T Ct = 0.0;
    T Dt = 0.0;
    T Et = 0.0;
    T Ht = 0.0;
    T Br = 0.0;
    T Cr = 1.0;
    T Dr = 0.0;
    T Hf = 0.0;
};

template<typename T>
Moment_terms<T> moment_terms(const P6_mnc_parameters& p,
                             const T& alpha,
                             const T& Fz,
                             const T& dFz,
                             const T& dPi,
                             const T& inclination_deg,
                             const Pure_lateral<T>& lateral)
{
    using std::abs;
    using std::atan;
    using std::cos;

    Moment_terms<T> result;
    result.Bt = (p.q_Bz[0] + p.q_Bz[1]*dFz + p.q_Bz[2]*dFz*dFz)
        *(T(1) + p.q_Bz[4]*abs(inclination_deg)
          + p.q_Bz[5]*inclination_deg*inclination_deg);
    result.Ct = p.q_Cz;
    result.Dt = (p.unloaded_radius_m/p.nominal_vertical_load)
        *(p.q_Dz[4]*(p.nominal_vertical_load/p.unloaded_radius_m)
          + p.q_Dz[0]*Fz + p.q_Dz[1]*Fz*dFz)
        *(T(1) - p.p_Pz[0]*dPi)
        *(T(1) + p.q_Dz[2]*abs(inclination_deg)
          + p.q_Dz[3]*inclination_deg*inclination_deg);
    result.Ht = p.q_Hz[0] + p.q_Hz[1]*dFz
        + (p.q_Hz[2] + p.q_Hz[3]*dFz)*inclination_deg;
    result.Et = (p.q_Ez[0] + p.q_Ez[1]*dFz + p.q_Ez[2]*dFz*dFz)
        *(T(1) + (p.q_Ez[3] + p.q_Ez[4]*inclination_deg)
          *(T(2)/T(3.141592653589793238462643383279502884))
          *atan(result.Bt*result.Ct*(alpha + result.Ht)));
    result.Br = p.q_Bz[9]*lateral.B*lateral.C;
    result.Cr = T(1);
    result.Dr = p.unloaded_radius_m*Fz
        *((p.q_Dz[5] + p.q_Dz[6]*dFz)
          + ((p.q_Dz[7] + p.q_Dz[8]*dFz)*(T(1) + p.p_Pz[1]*dPi)
             + (p.q_Dz[9] + p.q_Dz[10]*dFz)*abs(inclination_deg))*inclination_deg)
        *cos(alpha);
    result.Hf = lateral.H + lateral.V/nonzero_denominator(lateral.stiffness);
    return result;
}

template<typename T>
T combined_aligning_moment(const P6_mnc_parameters& p,
                           const T& alpha,
                           const T& kappa,
                           const T& dFz,
                           const T& inclination_deg,
                           const T& side_sign,
                           const T& Fx,
                           const T& Fy,
                           const T& Fyp,
                           const T& Kxk,
                           const T& Kya,
                           const Moment_terms<T>& m)
{
    using std::atan;
    using std::cos;
    using std::sqrt;

    const T alpha_t = alpha + side_sign*m.Ht;
    const T alpha_r = alpha + side_sign*m.Hf;
    const T stiffness_ratio = Kxk/nonzero_denominator(Kya);
    const T alpha_t_equivalent =
        sqrt(alpha_t*alpha_t + stiffness_ratio*stiffness_ratio*kappa*kappa
             + T(1.0e-24))*sign(alpha_t);
    const T alpha_r_equivalent =
        sqrt(alpha_r*alpha_r + stiffness_ratio*stiffness_ratio*kappa*kappa
             + T(1.0e-24))*sign(alpha_r);
    const T trail = m.Dt*cos(m.Ct*atan((T(1)-m.Et)*m.Bt*alpha_t_equivalent
                                      + m.Et*atan(m.Bt*alpha_t_equivalent)))*cos(alpha);
    const T scrub = p.unloaded_radius_m
        *(p.s_sz[0] + p.s_sz[1]*(Fy/p.nominal_vertical_load)
          + (p.s_sz[2] + p.s_sz[3]*dFz)*inclination_deg);
    const T residual = m.Dr*cos(m.Cr*atan(m.Br*alpha_r_equivalent))*cos(alpha);
    return -trail*Fyp + residual + scrub*Fx;
}

template<typename T>
T overturning_moment(const P6_mnc_parameters& p,
                     const T& Fz,
                     const T& dPi,
                     const T& inclination_deg,
                     const T& Fy)
{
    using std::atan;
    using std::cos;
    using std::sin;

    const T load_atan = atan(p.q_sx[5]*Fz/p.nominal_vertical_load);
    return p.unloaded_radius_m*Fz
        *(p.q_sx[0]
          - p.q_sx[1]*inclination_deg*(T(1) + p.p_PMx*dPi)
          + p.q_sx[2]*Fy/p.nominal_vertical_load
          + p.q_sx[3]*cos(p.q_sx[4]*load_atan*load_atan)
            *sin(p.q_sx[6]*inclination_deg
                 + p.q_sx[7]*atan(p.q_sx[8]*Fy/p.nominal_vertical_load))
          + p.q_sx[9]*atan(p.q_sx[10]*Fz/p.nominal_vertical_load)*inclination_deg);
}

} // namespace detail

template<typename Timeseries_t>
P6_mnc_model<Timeseries_t>::P6_mnc_model()
: _parameters(std::make_shared<P6_mnc_parameters>())
{}

template<typename Timeseries_t>
P6_mnc_model<Timeseries_t>::P6_mnc_model(
    std::shared_ptr<const P6_mnc_parameters> parameters)
: _parameters(parameters ? std::move(parameters) : std::make_shared<P6_mnc_parameters>())
{
    initialise_peak_slips();
}

template<typename Timeseries_t>
P6_mnc_model<Timeseries_t>::P6_mnc_model(const P6_mnc_parameters& parameters)
: P6_mnc_model(std::make_shared<P6_mnc_parameters>(parameters))
{}

template<typename Timeseries_t>
bool P6_mnc_model<Timeseries_t>::is_ready() const
{
    return _parameters
        && _parameters->nominal_vertical_load > 0.0
        && _parameters->nominal_pressure_kpa > 0.0
        && _parameters->unloaded_radius_m > 0.0
        && _parameters->p_Cx != 0.0
        && _parameters->p_Cy != 0.0;
}

template<typename Timeseries_t>
void P6_mnc_model<Timeseries_t>::set_force_correction_factors(
    const Timeseries_t& longitudinal_factor,
    const Timeseries_t& lateral_factor)
{
    if constexpr (std::is_arithmetic<Timeseries_t>::value)
    {
        if (longitudinal_factor < Timeseries_t(0)
            || longitudinal_factor > Timeseries_t(1)
            || lateral_factor < Timeseries_t(0)
            || lateral_factor > Timeseries_t(1))
            throw std::invalid_argument(
                "FRUCD force correction factors must be in [0,1]");
    }

    _longitudinal_force_correction_factor = longitudinal_factor;
    _lateral_force_correction_factor = lateral_factor;
}

template<typename Timeseries_t>
Contact_patch_loads<Timeseries_t> P6_mnc_model<Timeseries_t>::evaluate(
    const Timeseries_t& slip_angle_rad,
    const Timeseries_t& slip_ratio,
    const Timeseries_t& normal_load_N,
    const Timeseries_t& pressure_kpa,
    const Timeseries_t& inclination_deg,
    const Timeseries_t& velocity_mps,
    Tire_side side) const
{
    (void)velocity_mps;

    if (!is_ready())
        return {};
    if constexpr (std::is_arithmetic<Timeseries_t>::value)
        if (normal_load_N <= Timeseries_t(0))
            return {};

    const auto& p = *_parameters;
    const Timeseries_t activity = detail::load_activity(normal_load_N);
    const Timeseries_t Fz = detail::positive_load(normal_load_N);
    const Timeseries_t side_sign = side == Tire_side::left ? Timeseries_t(1) : Timeseries_t(-1);
    const Timeseries_t alpha = slip_angle_rad*side_sign;
    const Timeseries_t inclination = inclination_deg*side_sign;
    const Timeseries_t dFz = (Fz - p.nominal_vertical_load)/p.nominal_vertical_load;
    const Timeseries_t dPi = (pressure_kpa - p.nominal_pressure_kpa)/p.nominal_pressure_kpa;

    const auto longitudinal = detail::pure_longitudinal(
        p,slip_ratio,Fz,dFz,dPi,inclination);
    const auto lateral = detail::pure_lateral(
        p,alpha,Fz,dFz,dPi,inclination,side_sign);
    const auto lateral_zero_inclination = detail::pure_lateral(
        p,alpha,Fz,dFz,dPi,Timeseries_t(0),side_sign);
    const auto moments = detail::moment_terms(
        p,alpha,Fz,dFz,dPi,inclination,lateral);
    const auto combined = detail::mnc(
        alpha,slip_ratio,longitudinal.force,lateral.force,
        longitudinal.stiffness,lateral.stiffness,
        longitudinal.null_slip,lateral.null_slip);
    const auto combined_zero_inclination = detail::mnc(
        alpha,slip_ratio,longitudinal.force,lateral_zero_inclination.force,
        longitudinal.stiffness,lateral.stiffness,
        longitudinal.null_slip,lateral_zero_inclination.null_slip);

    Contact_patch_loads<Timeseries_t> result;
    result.Fx = activity*_longitudinal_force_correction_factor*combined.Fx;
    result.Fy = activity*_lateral_force_correction_factor*combined.Fy;
    result.Mz = activity*detail::combined_aligning_moment(
        p,alpha,slip_ratio,dFz,inclination,side_sign,
        combined.Fx,combined.Fy,combined_zero_inclination.Fy,
        longitudinal.stiffness,lateral.stiffness,moments);
    result.Mx = activity*detail::overturning_moment(
        p,Fz,dPi,inclination,combined.Fy);
    result.My = Timeseries_t(0);
    return result;
}

template<typename Timeseries_t>
void P6_mnc_model<Timeseries_t>::initialise_peak_slips()
{
    if (!is_ready())
        return;

    const auto& p = *_parameters;
    _peak_reference_load_1 = 0.5*p.nominal_vertical_load;
    _peak_reference_load_2 = 1.5*p.nominal_vertical_load;

    const auto find_peaks = [&p](double Fz)
    {
        const double dFz = (Fz-p.nominal_vertical_load)/p.nominal_vertical_load;
        constexpr std::size_t samples = 4000;
        double best_kappa = 0.1;
        double best_fx = -1.0;
        for (std::size_t i=0; i<=samples; ++i)
        {
            const double kappa = 0.5*static_cast<double>(i)/samples;
            const auto pure = detail::pure_longitudinal(
                p,kappa,Fz,dFz,0.0,0.0);
            const double magnitude = std::abs(pure.force);
            if (magnitude > best_fx)
            {
                best_fx = magnitude;
                best_kappa = kappa;
            }
        }

        double best_alpha = 0.1;
        double best_fy = -1.0;
        for (std::size_t i=0; i<=2*samples; ++i)
        {
            const double alpha = -0.5 + static_cast<double>(i)/samples*0.5;
            const auto pure = detail::pure_lateral(
                p,alpha,Fz,dFz,0.0,0.0,1.0);
            const double magnitude = std::abs(pure.force);
            if (magnitude > best_fy)
            {
                best_fy = magnitude;
                best_alpha = std::abs(alpha);
            }
        }
        return std::array<double,2>{std::max(best_kappa,1.0e-6),
                                    std::max(std::tan(best_alpha),1.0e-6)};
    };

    const auto peaks_1 = find_peaks(_peak_reference_load_1);
    const auto peaks_2 = find_peaks(_peak_reference_load_2);
    _maximum_kappa_1 = peaks_1[0];
    _maximum_kappa_2 = peaks_2[0];
    _maximum_lambda_1 = peaks_1[1];
    _maximum_lambda_2 = peaks_2[1];
}

template<typename Timeseries_t>
Timeseries_t P6_mnc_model<Timeseries_t>::maximum_kappa(
    const Timeseries_t& normal_load_N) const
{
    return _maximum_kappa_1
        + (normal_load_N-_peak_reference_load_1)
          *(_maximum_kappa_2-_maximum_kappa_1)
          /(_peak_reference_load_2-_peak_reference_load_1);
}

template<typename Timeseries_t>
Timeseries_t P6_mnc_model<Timeseries_t>::maximum_lambda(
    const Timeseries_t& normal_load_N) const
{
    return _maximum_lambda_1
        + (normal_load_N-_peak_reference_load_1)
          *(_maximum_lambda_2-_maximum_lambda_1)
          /(_peak_reference_load_2-_peak_reference_load_1);
}

} // namespace frucd

#undef FRUCD_P6_MNC_HAS_CPPAD

#endif
