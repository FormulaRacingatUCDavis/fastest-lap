#ifndef ELECTRIC_MOTOR_H
#define ELECTRIC_MOTOR_H

#include <string>

#include "lion/foundation/constants.h"
#include "lion/foundation/types.h"
#include "lion/io/Xml_document.h"
#include "lion/io/database_parameters.h"
#include "lion/thirdparty/include/cppad/cppad.hpp"
#include "src/core/foundation/fastest_lap_exception.h"

//! Quasi-static electric powertrain model.
//!
//! The motor follows a constant-torque/constant-power envelope at the motor
//! shaft.  A fixed reduction ratio and transmission efficiency convert shaft
//! torque into axle torque.  Battery discharge and charge power limits are
//! applied before the torque reaches the axle, while regenerative braking is
//! represented by a separate torque and efficiency limit.
template<typename Timeseries_t>
class Electric_motor
{
 public:
    using Timeseries_type = Timeseries_t;

    static constexpr bool supports_regeneration = true;
    static constexpr const char* parameter_path = "motor/";
    static constexpr const char* energy_integral_name = "battery-energy";

    Electric_motor() = default;

    Electric_motor(Xml_document& database, const std::string& path,
                   const bool /*only_max_power*/);

    Electric_motor(const std::string& path, const bool /*only_max_power*/)
        : _path(path)
    {}

    template<typename T>
    void set_parameter(const std::string& parameter, const T value)
    {
        const auto found = ::set_parameter(
            get_parameters(),__used_parameters,parameter,_path,value);
        if (!found)
            throw fastest_lap_exception(
                "Parameter \"" + parameter + "\" was not found in electric motor");
    }

    void fill_xml(Xml_document& doc) const
    {
        ::write_parameters(doc,_path,get_parameters());
    }

    //! Return axle torque for a command in [-1,1] and axle speed [rad/s].
    Timeseries_t operator()(Timeseries_t command, Timeseries_t axle_speed);

    constexpr const bool& direct_torque() const { return _direct_torque; }
    constexpr       bool& direct_torque()       { return _direct_torque; }

    const Timeseries_t& get_power() const { return _battery_power; }
    const Timeseries_t& get_motor_speed() const { return _motor_speed; }
    const Timeseries_t& get_motor_torque() const { return _motor_torque; }
    const Timeseries_t& get_wheel_torque() const { return _wheel_torque; }

 private:
    std::string _path;
    bool _direct_torque = false;

    // Motor and inverter parameters
    Timeseries_t _maximum_torque = 0.0;                 //! [N.m], motor shaft
    Timeseries_t _maximum_power = 0.0;                  //! [kW], motor shaft
    Timeseries_t _maximum_speed = 0.0;                  //! [rpm]
    Timeseries_t _gear_ratio = 1.0;                     //! motor/wheel speed
    Timeseries_t _transmission_efficiency = 1.0;        //! shaft-to-wheel
    Timeseries_t _drive_efficiency = 1.0;               //! battery-to-shaft
    Timeseries_t _battery_maximum_discharge_power = 0.0;//! [kW], electrical

    // Regenerative braking parameters
    Timeseries_t _regenerative_maximum_torque = 0.0;    //! [N.m], motor shaft
    Timeseries_t _regenerative_minimum_speed = 0.0;     //! [rpm], motor shaft
    Timeseries_t _battery_maximum_charge_power = 0.0;   //! [kW], electrical
    Timeseries_t _regenerative_efficiency = 1.0;        //! shaft-to-battery

    // Last evaluation
    Timeseries_t _battery_power = 0.0;                  //! [W], + discharge
    Timeseries_t _motor_speed = 0.0;                    //! [rad/s]
    Timeseries_t _motor_torque = 0.0;                   //! [N.m]
    Timeseries_t _wheel_torque = 0.0;                   //! [N.m]

    DECLARE_PARAMS(
        { "maximum-torque", _maximum_torque },
        { "maximum-power", _maximum_power },
        { "maximum-speed", _maximum_speed },
        { "gear-ratio", _gear_ratio },
        { "transmission-efficiency", _transmission_efficiency },
        { "drive-efficiency", _drive_efficiency },
        { "battery-maximum-discharge-power", _battery_maximum_discharge_power },
        { "regenerative-maximum-torque", _regenerative_maximum_torque },
        { "regenerative-minimum-speed", _regenerative_minimum_speed },
        { "battery-maximum-charge-power", _battery_maximum_charge_power },
        { "regenerative-efficiency", _regenerative_efficiency }
    );
};

#include "electric_motor.hpp"

#endif
