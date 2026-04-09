#ifndef __ELECTRIC_MOTOR_H__
#define __ELECTRIC_MOTOR_H__

#include <string>
#include "lion/io/Xml_document.h"
#include "lion/io/database_parameters.h"
#include "src/core/foundation/fastest_lap_exception.h"

//! Electric motor model
//!
//! Models a single electric motor with a simplified torque-speed characteristic:
//!   - Constant torque region: T = throttle * T_max,  for omega*gear_ratio <= P_max*1000 / T_max
//!   - Constant power region: T = throttle * P_max*1000 / (omega*gear_ratio), otherwise
//!
//! The motor is also capable of regenerative braking when throttle < 0 (treated as negative torque).
template<typename Timeseries_t>
class Electric_motor
{
 public:
    using Timeseries_type = Timeseries_t;

    //! Default constructor (no motor)
    Electric_motor() = default;

    //! Construct from XML database
    Electric_motor(Xml_document& database, const std::string& path);

    //! Construct from path only (parameters set later)
    Electric_motor(const std::string& path)
        : _path(path), _maximum_torque(0.0), _maximum_power(0.0), _gear_ratio(1.0) {}

    //! Set a named parameter
    template<typename T>
    void set_parameter(const std::string& parameter, const T value)
    {
        const auto found = ::set_parameter(get_parameters(), __used_parameters, parameter, _path, value);
        if (!found)
            throw fastest_lap_exception("Parameter \"" + parameter + "\" was not found in Electric_motor");
    }

    //! Write parameters into XML document
    void fill_xml(Xml_document& doc) const
    {
        ::write_parameters(doc, _path, get_parameters());
    }

    //! Compute torque output
    //! @param[in] throttle_percentage  [-1,1] throttle command (negative = regen)
    //! @param[in] angular_speed        axle angular speed [rad/s]
    //! @return torque [N.m] (positive = driving, negative = regen)
    Timeseries_t operator()(const Timeseries_t throttle_percentage, const Timeseries_t angular_speed) const;

    //! Getters
    const scalar& get_maximum_torque() const { return _maximum_torque; }
    const scalar& get_maximum_power()  const { return _maximum_power; }
    const scalar& get_gear_ratio()     const { return _gear_ratio; }

 private:
    std::string _path;

    scalar _maximum_torque = 0.0;    //! Maximum motor torque [N.m]
    scalar _maximum_power  = 0.0;    //! Maximum motor power [kW]
    scalar _gear_ratio     = 1.0;    //! Gear ratio between motor shaft and wheel axle [-]

    DECLARE_PARAMS(
        { "maximum-torque", _maximum_torque },
        { "maximum-power",  _maximum_power  },
        { "gear-ratio",     _gear_ratio     }
    );

};

#include "electric_motor.hpp"

#endif
