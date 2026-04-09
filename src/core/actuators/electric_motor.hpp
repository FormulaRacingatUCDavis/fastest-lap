#ifndef __ELECTRIC_MOTOR_HPP__
#define __ELECTRIC_MOTOR_HPP__

template<typename Timeseries_t>
inline Electric_motor<Timeseries_t>::Electric_motor(Xml_document& database, const std::string& path)
: _path(path),
  _maximum_torque(0.0),
  _maximum_power(0.0),
  _gear_ratio(1.0)
{
    read_parameters(database, path, get_parameters(), __used_parameters);
}


template<typename Timeseries_t>
inline Timeseries_t Electric_motor<Timeseries_t>::operator()(const Timeseries_t throttle_percentage, const Timeseries_t angular_speed) const
{
    // Motor shaft speed = axle speed * gear_ratio
    // Use a small epsilon to avoid division by zero at standstill
    const scalar eps = 1.0e-6;
    const Timeseries_t shaft_speed = _gear_ratio * angular_speed + eps;

    // Maximum power available [W] = maximum-power [kW] * 1000
    const scalar P_max_W = _maximum_power * 1.0e3;

    // We compute torque AT THE AXLE limited by P_max
    // Power = T_axle * omega_axle => T_axle_limit = Power / omega_axle
    const Timeseries_t torque_power_limit = P_max_W / (angular_speed + eps);

    // Clamp to T_max in constant-torque region
    // Note: _maximum_torque is defined as maximum torque AT THE AXLE in our XML
    Timeseries_t T_limit;
    if constexpr (std::is_same_v<Timeseries_t, double>)
    {
        T_limit = std::min(_maximum_torque, Value(torque_power_limit));
    }
    else
    {
        // CppAD-friendly: CondExpLt(left, right, true_val, false_val)
        T_limit = CppAD::CondExpLt(torque_power_limit, Timeseries_t(_maximum_torque),
                                   torque_power_limit, Timeseries_t(_maximum_torque));
    }

    return throttle_percentage * T_limit;
}

#endif
