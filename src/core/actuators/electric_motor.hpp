#ifndef ELECTRIC_MOTOR_HPP
#define ELECTRIC_MOTOR_HPP

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace electric_motor_detail
{

template<typename T>
T minimum(const T& lhs, const T& rhs)
{
    if constexpr (std::is_arithmetic_v<T>)
        return std::min(lhs,rhs);
    else
        return CppAD::CondExpLt(lhs,rhs,lhs,rhs);
}

template<typename T>
T maximum(const T& lhs, const T& rhs)
{
    if constexpr (std::is_arithmetic_v<T>)
        return std::max(lhs,rhs);
    else
        return CppAD::CondExpGt(lhs,rhs,lhs,rhs);
}

template<typename T>
T enabled_below(const T& value, const T& limit)
{
    if constexpr (std::is_arithmetic_v<T>)
        return value <= limit ? T(1) : T(0);
    else
        return CppAD::CondExpLe(value,limit,T(1),T(0));
}

template<typename T>
T absolute(const T& value)
{
    using std::abs;
    return abs(value);
}

} // namespace electric_motor_detail

template<typename Timeseries_t>
Electric_motor<Timeseries_t>::Electric_motor(
    Xml_document& database, const std::string& path,
    const bool /*only_max_power*/)
    : _path(path)
{
    read_parameters(database,path,get_parameters(),__used_parameters);
}

template<typename Timeseries_t>
Timeseries_t Electric_motor<Timeseries_t>::operator()(
    Timeseries_t command, Timeseries_t axle_speed)
{
    using electric_motor_detail::absolute;
    using electric_motor_detail::enabled_below;
    using electric_motor_detail::maximum;
    using electric_motor_detail::minimum;

    const Timeseries_t zero = 0.0;
    const Timeseries_t positive_command = minimum(maximum(command,zero),Timeseries_t(1.0));
    const Timeseries_t regenerative_command = minimum(maximum(-command,zero),Timeseries_t(1.0));

    _motor_speed = absolute(axle_speed)*_gear_ratio;
    const Timeseries_t speed_for_power = maximum(_motor_speed,Timeseries_t(1.0e-3));
    const Timeseries_t speed_enabled = enabled_below(_motor_speed,_maximum_speed*RPM);
    const Timeseries_t regeneration_speed_enabled =
        enabled_below(_regenerative_minimum_speed*RPM,_motor_speed)
        *speed_enabled;

    // Battery power is measured before the inverter/motor.  Convert the
    // electrical discharge ceiling into the mechanical power available at the
    // shaft, then apply the motor's own mechanical power ceiling.
    const Timeseries_t traction_shaft_power = minimum(
        _maximum_power*1.0e3,
        _battery_maximum_discharge_power*1.0e3*_drive_efficiency);
    const Timeseries_t traction_torque_limit = speed_enabled*minimum(
        _maximum_torque,traction_shaft_power/speed_for_power);

    // During regeneration, the charge limit is electrical.  Divide by the
    // regeneration efficiency to obtain the permitted mechanical shaft power.
    const Timeseries_t regeneration_efficiency = maximum(
        _regenerative_efficiency,Timeseries_t(1.0e-6));
    const Timeseries_t regenerative_shaft_power = minimum(
        _maximum_power*1.0e3,
        _battery_maximum_charge_power*1.0e3/regeneration_efficiency);
    const Timeseries_t regenerative_torque_limit = regeneration_speed_enabled*minimum(
        _regenerative_maximum_torque,
        regenerative_shaft_power/speed_for_power);

    _motor_torque = positive_command*traction_torque_limit
                  - regenerative_command*regenerative_torque_limit;

    const Timeseries_t transmission_efficiency = maximum(
        _transmission_efficiency,Timeseries_t(1.0e-6));
    _wheel_torque = positive_command*traction_torque_limit
                      *_gear_ratio*transmission_efficiency
                  - regenerative_command*regenerative_torque_limit
                      *_gear_ratio/transmission_efficiency;

    const Timeseries_t drive_efficiency = maximum(
        _drive_efficiency,Timeseries_t(1.0e-6));
    _battery_power = positive_command*traction_torque_limit
                       *_motor_speed/drive_efficiency
                   - regenerative_command*regenerative_torque_limit
                       *_motor_speed*regeneration_efficiency;

    return _wheel_torque;
}

#endif
