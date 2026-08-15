#ifndef TIRE_FRUCD_P6_MNC_HPP
#define TIRE_FRUCD_P6_MNC_HPP

#include "lion/foundation/utils.h"
#include "src/core/foundation/fastest_lap_exception.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <type_traits>

namespace frucd
{
namespace detail
{

inline Xml_document& prepare_tire_database(Xml_document& database,
                                           const std::string& path)
{
    if (!database.has_element(path + "mat-file"))
        throw fastest_lap_exception("Tire_frucd_p6_mnc: missing XML element \"" + path + "mat-file\"");

    // Tire's base constructor requires radius immediately. Materialize it
    // from the fit before entering that constructor, so users do not have to
    // duplicate Ro in XML.
    if (!database.has_element(path + "radius"))
    {
        auto filename = std::filesystem::path(
            database.get_element(path + "mat-file").get_value(std::string()));
        if (filename.is_relative() && !database.get_file_name().empty())
            filename = std::filesystem::path(database.get_file_name()).parent_path()/filename;
        const auto parameters = Mat_tire_data::load(filename.lexically_normal().string());
        database.add_element(path + "radius").set_value(
            std::to_string(parameters->unloaded_radius_m));
    }
    return database;
}

inline std::string resolve_mat_filename(Xml_document& database,
                                        const std::string& filename)
{
    auto resolved = std::filesystem::path(filename);
    if (resolved.is_relative() && !database.get_file_name().empty())
        resolved = std::filesystem::path(database.get_file_name()).parent_path()/resolved;
    return resolved.lexically_normal().string();
}

template<typename T>
T regularized_longitudinal_velocity(const T& velocity)
{
    using std::abs;
    constexpr double minimum_velocity = 1.0e-9;
    if constexpr (std::is_arithmetic<T>::value)
    {
        if (abs(velocity) >= minimum_velocity)
            return velocity;
        return velocity < T(0) ? T(-minimum_velocity) : T(minimum_velocity);
    }
    else
    {
        const T magnitude = CppAD::CondExpGt(
            abs(velocity),T(minimum_velocity),abs(velocity),T(minimum_velocity));
        const T direction = CppAD::CondExpGe(velocity,T(0),T(1),T(-1));
        return direction*magnitude;
    }
}

} // namespace detail
} // namespace frucd

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::Tire_frucd_p6_mnc(
    const std::string& name, const std::string& path)
: base_type(name,path),
  _model(),
  _mat_filename()
{}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::Tire_frucd_p6_mnc(
    const std::string& name, const std::string& mat_filename, const std::string& path)
: base_type(name,path),
  _model(),
  _mat_filename(mat_filename)
{
    std::fill(__used_parameters.begin(),__used_parameters.end(),true);
    configure(frucd::Mat_tire_data::load(mat_filename));
}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::Tire_frucd_p6_mnc(
    const std::string& name,
    std::shared_ptr<const frucd::P6_mnc_parameters> parameters,
    const std::string& path)
: base_type(name,path),
  _model(),
  _mat_filename()
{
    std::fill(__used_parameters.begin(),__used_parameters.end(),true);
    configure(std::move(parameters));
}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::Tire_frucd_p6_mnc(
    const std::string& name, Xml_document& database, const std::string& path)
: base_type(name,frucd::detail::prepare_tire_database(database,path),path),
  _model(),
  _mat_filename()
{
    if (!database.has_element(path + "mat-file"))
        throw fastest_lap_exception("Tire_frucd_p6_mnc: missing XML element \"" + path + "mat-file\"");

    _mat_filename = database.get_element(path + "mat-file").get_value(std::string());
    database.get_element(path + "mat-file").set_attribute<bool>("__unused__",false);
    configure(frucd::Mat_tire_data::load(
        frucd::detail::resolve_mat_filename(database,_mat_filename)));

    // The fitted file owns the static operating point. XML values, when
    // present, override these defaults.
    _pressure_kpa = _model.parameters().nominal_pressure_kpa;
    _inclination_deg = 0.0;
    _kt = 0.0;
    _ct = 0.0;
    _Fz_max_ref2 = 1.0;
    _longitudinal_force_correction_factor = 0.7;
    _lateral_force_correction_factor = 0.7;

    if (database.has_element(path + "radial-stiffness"))
    {
        _kt = database.get_element(path + "radial-stiffness").get_value(scalar());
        database.get_element(path + "radial-stiffness").set_attribute<bool>("__unused__",false);
    }
    if (database.has_element(path + "radial-damping"))
    {
        _ct = database.get_element(path + "radial-damping").get_value(scalar());
        database.get_element(path + "radial-damping").set_attribute<bool>("__unused__",false);
    }
    if (database.has_element(path + "Fz-max-ref2"))
    {
        _Fz_max_ref2 = database.get_element(path + "Fz-max-ref2").get_value(scalar());
        database.get_element(path + "Fz-max-ref2").set_attribute<bool>("__unused__",false);
    }
    if (database.has_element(path + "pressure"))
    {
        _pressure_kpa = database.get_element(path + "pressure").get_value(scalar());
        database.get_element(path + "pressure").set_attribute<bool>("__unused__",false);
    }
    if (database.has_element(path + "inclination"))
    {
        _inclination_deg = database.get_element(path + "inclination").get_value(scalar());
        database.get_element(path + "inclination").set_attribute<bool>("__unused__",false);
    }
    if (database.has_element(path + "longitudinal-force-correction-factor"))
    {
        _longitudinal_force_correction_factor = database
            .get_element(path + "longitudinal-force-correction-factor")
            .get_value(scalar());
        database.get_element(path + "longitudinal-force-correction-factor")
            .set_attribute<bool>("__unused__",false);
    }
    if (database.has_element(path + "lateral-force-correction-factor"))
    {
        _lateral_force_correction_factor = database
            .get_element(path + "lateral-force-correction-factor")
            .get_value(scalar());
        database.get_element(path + "lateral-force-correction-factor")
            .set_attribute<bool>("__unused__",false);
    }

    _model.set_force_correction_factors(
        _longitudinal_force_correction_factor,
        _lateral_force_correction_factor);

    std::fill(__used_parameters.begin(),__used_parameters.end(),true);
}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
void Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::configure(
    std::shared_ptr<const frucd::P6_mnc_parameters> parameters)
{
    if (!parameters)
        throw fastest_lap_exception("Tire_frucd_p6_mnc: null parameter bundle");

    _model = model_type(parameters);
    _model.set_force_correction_factors(
        _longitudinal_force_correction_factor,
        _lateral_force_correction_factor);
    _pressure_kpa = parameters->nominal_pressure_kpa;
    _inclination_deg = 0.0;

    // Radius is part of Tire's existing public contract, so expose the value
    // loaded from Tire.Pacejka.Ro through the base class parameter machinery.
    base_type::set_parameter(base_type::get_path() + "radius", parameters->unloaded_radius_m);
}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
template<typename T>
void Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::set_parameter(
    const std::string& parameter, const T value)
{
    if (parameter.find(base_type::get_path()) != 0)
        throw fastest_lap_exception("Parameter \"" + parameter + "\" was not found in Tire_frucd_p6_mnc");
    if (parameter == base_type::get_path() + "radius")
        throw fastest_lap_exception(
            "Tire_frucd_p6_mnc: radius is owned by the fitted MAT file and cannot be overridden");

    const auto found = ::set_parameter(get_parameters(), __used_parameters,
                                       parameter, base_type::get_path(), value);
    if (!found)
        base_type::set_parameter(parameter,value);
    else
        _model.set_force_correction_factors(
            _longitudinal_force_correction_factor,
            _lateral_force_correction_factor);
}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
void Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::fill_xml(Xml_document& doc) const
{
    base_type::fill_xml(doc);
    ::write_parameters(doc,base_type::get_path(),get_parameters());

    if (!_mat_filename.empty())
    {
        if (doc.has_element(base_type::get_path() + "mat-file"))
            doc.get_element(base_type::get_path() + "mat-file").set_value(_mat_filename);
        else
            doc.add_element(base_type::get_path() + "mat-file").set_value(_mat_filename);
    }
}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
void Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::update(
    const Vector3d<Timeseries_t>& x0,
    const Vector3d<Timeseries_t>& v0,
    Timeseries_t omega)
{
    base_type::get_frame().set_origin(
        x0,v0,Frame<Timeseries_t>::Frame_velocity_types::parent_frame);
    update(omega);
}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
void Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::update(
    Timeseries_t normal_load_N,
    Timeseries_t kappa_dimensionless,
    const Frame<Timeseries_t>& road_frame)
{
    update_kinematics_from_kappa(
        kappa_dimensionless*_model.maximum_kappa(normal_load_N),road_frame);
    update_self(normal_load_N);
}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
void Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::update(Timeseries_t omega)
{
    update_kinematics(omega);
    update_self();
}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
void Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::update_kinematics(
    Timeseries_t omega)
{
    base_type::_omega = omega;
    base_type::_w = base_type::get_frame()
        .get_absolute_position({0.0,0.0,base_type::get_radius()}).at(Z);
    base_type::_dw = base_type::get_frame()
        .get_absolute_velocity_in_inertial({0.0,0.0,base_type::get_radius()}).at(Z);
    base_type::_v = base_type::get_frame()
        .get_absolute_velocity_in_body(base_type::get_contact_point());

    const Timeseries_t longitudinal_velocity =
        frucd::detail::regularized_longitudinal_velocity(base_type::_v[X]);
    base_type::_kappa =
        (base_type::_omega*base_type::get_radius()-base_type::_v[X])/longitudinal_velocity;
    base_type::_lambda = -base_type::_v[Y]/longitudinal_velocity;
}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
void Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::update_kinematics_from_kappa(
    Timeseries_t kappa, const Frame<Timeseries_t>& road_frame)
{
    base_type::_kappa = kappa;
    const auto position_and_velocity = base_type::get_frame()
        .get_position_and_velocity_in_target(
            road_frame,{0.0,0.0,base_type::get_radius()});
    base_type::_w = position_and_velocity.first.z();
    base_type::_dw = position_and_velocity.second.z();
    base_type::_v = base_type::get_frame()
        .get_absolute_velocity_in_body(base_type::get_contact_point());

    const Timeseries_t longitudinal_velocity =
        frucd::detail::regularized_longitudinal_velocity(base_type::_v[X]);
    base_type::_lambda = -base_type::_v[Y]/longitudinal_velocity;
    base_type::_omega =
        (Timeseries_t(1)+kappa)*base_type::_v[X]/base_type::get_radius();
}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
void Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::update_self()
{
    if (_kt <= 1.0e-12)
        throw fastest_lap_exception(
            "Tire_frucd_p6_mnc: radial-stiffness must be positive for the 6DOF update signature");
    const Timeseries_t normal_load_N =
        smooth_pos<Timeseries_t>(_kt*base_type::_w + _ct*base_type::_dw,_Fz_max_ref2);
    update_self(normal_load_N);
}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
void Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::update_self(
    const Timeseries_t& normal_load_N)
{
    using std::atan;
    using std::abs;

    const Timeseries_t alpha_frucd = atan(base_type::_lambda);
    const auto loads = _model.evaluate(
        alpha_frucd,
        base_type::_kappa,
        normal_load_N,
        Timeseries_t(_pressure_kpa),
        Timeseries_t(_inclination_deg),
        abs(base_type::_v[X]),
        side);

    // FRUCD is Z-up/y-left; Fastest-lap is Z-down/y-right.
    base_type::_F = Vector3d<Timeseries_t>{loads.Fx,-loads.Fy,-normal_load_N};
    const Vector3d<Timeseries_t> contact_patch_moment =
        {loads.Mx,-loads.My,-loads.Mz};
    base_type::_T = cross(base_type::get_contact_point(),base_type::_F) + contact_patch_moment;
}

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
std::unordered_map<std::string,Timeseries_t>
Tire_frucd_p6_mnc<Timeseries_t,side,state_start,control_start>::get_outputs_map() const
{
    return base_type::get_outputs_map();
}

#endif
