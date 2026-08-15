#ifndef TIRE_FRUCD_P6_MNC_H
#define TIRE_FRUCD_P6_MNC_H

#include "frucd_mat_tire_data.h"
#include "frucd_p6_mnc_model.h"
#include "tire.h"

#include <memory>
#include <string>
#include <unordered_map>

template<typename Timeseries_t, frucd::Tire_side side, size_t state_start, size_t control_start>
class Tire_frucd_p6_mnc : public Tire<Timeseries_t,state_start,control_start>
{
 public:
    using base_type = Tire<Timeseries_t,state_start,control_start>;
    using model_type = frucd::P6_mnc_model<Timeseries_t>;

    // MF6.2 + MNC contains abs/sign branches. A quasi-Newton Hessian is more
    // robust than differentiating those branches twice in steady-state NLPs.
    static constexpr bool steady_state_prefers_limited_memory_hessian = true;

    struct input_names { enum { end = base_type::input_names::end }; };
    struct state_names { enum { end = base_type::state_names::end }; };
    struct control_names { enum { end = base_type::control_names::end }; };

    static_assert(static_cast<size_t>(input_names::end) == static_cast<size_t>(state_names::end));

    Tire_frucd_p6_mnc() = default;

    //! Fastest-lap empty constructor used while assembling vehicle types.
    Tire_frucd_p6_mnc(const std::string& name,
                      const std::string& path);

    //! Construct directly from a fitted MATLAB file. The explicit third
    //! argument keeps this distinct from Fastest-lap's (name,path) signature.
    Tire_frucd_p6_mnc(const std::string& name,
                      const std::string& mat_filename,
                      const std::string& path);

    //! Construct from already loaded parameters (useful when sharing one fit across tires).
    Tire_frucd_p6_mnc(const std::string& name,
                      std::shared_ptr<const frucd::P6_mnc_parameters> parameters,
                      const std::string& path = "");

    //! Fastest-lap XML constructor. The tire node must contain <mat-file>.
    Tire_frucd_p6_mnc(const std::string& name,
                      Xml_document& database,
                      const std::string& path = "");

    template<typename T>
    void set_parameter(const std::string& parameter, const T value);

    void fill_xml(Xml_document& doc) const;

    void update(const Vector3d<Timeseries_t>& x0,
                const Vector3d<Timeseries_t>& v0,
                Timeseries_t omega);

    void update(Timeseries_t normal_load_N,
                Timeseries_t kappa_dimensionless,
                const Frame<Timeseries_t>& road_frame);

    void update(Timeseries_t omega);

    constexpr const scalar& get_radial_stiffness() const { return _kt; }
    const model_type& get_model() const { return _model; }

    //! Torque about Fastest-lap's positive wheel-spin direction.
    Timeseries_t get_longitudinal_torque_at_wheel_center() const
    {
        return -base_type::_T[Y];
    }

    template<size_t number_of_states>
    void get_state_and_state_derivative(std::array<Timeseries_t,number_of_states>&,
                                        std::array<Timeseries_t,number_of_states>&) const {}

    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_controls(const std::array<Timeseries_t,number_of_inputs>&,
                                const std::array<Timeseries_t,number_of_controls>&) {}

    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_control_upper_lower_and_default_values(
        const std::array<scalar,number_of_inputs>&,
        const std::array<scalar,number_of_inputs>&,
        const std::array<scalar,number_of_inputs>&,
        const std::array<scalar,number_of_controls>&,
        const std::array<scalar,number_of_controls>&,
        const std::array<scalar,number_of_controls>&) const {}

    template<size_t number_of_inputs, size_t number_of_controls>
    void set_state_and_control_names(std::array<std::string,number_of_inputs>&,
                                     std::array<std::string,number_of_controls>&) const {}

    bool is_ready() const
    {
        return base_type::is_ready() && _model.is_ready() &&
            std::all_of(__used_parameters.begin(), __used_parameters.end(), [](bool value) { return value; });
    }

    static std::string type() { return "tire_frucd_p6_mnc"; }

    std::unordered_map<std::string,Timeseries_t> get_outputs_map() const;

 private:
    void configure(std::shared_ptr<const frucd::P6_mnc_parameters> parameters);
    void update_kinematics(Timeseries_t omega);
    void update_kinematics_from_kappa(Timeseries_t kappa,
                                      const Frame<Timeseries_t>& road_frame);
    void update_self();
    void update_self(const Timeseries_t& normal_load_N);

    model_type _model;
    std::string _mat_filename;

    scalar _kt = 0.0;
    scalar _ct = 0.0;
    scalar _Fz_max_ref2 = 1.0;
    scalar _pressure_kpa = 0.0;
    scalar _inclination_deg = 0.0;
    Timeseries_t _longitudinal_force_correction_factor = 0.7;
    Timeseries_t _lateral_force_correction_factor = 0.7;

    DECLARE_PARAMS(
        { "radial-stiffness", _kt },
        { "radial-damping", _ct },
        { "Fz-max-ref2", _Fz_max_ref2 },
        { "pressure", _pressure_kpa },
        { "inclination", _inclination_deg },
        { "longitudinal-force-correction-factor", _longitudinal_force_correction_factor },
        { "lateral-force-correction-factor", _lateral_force_correction_factor }
    )
};

template<typename Timeseries_t, size_t state_start, size_t control_start>
using Tire_frucd_p6_mnc_left =
    Tire_frucd_p6_mnc<Timeseries_t,frucd::Tire_side::left,state_start,control_start>;

template<typename Timeseries_t, size_t state_start, size_t control_start>
using Tire_frucd_p6_mnc_right =
    Tire_frucd_p6_mnc<Timeseries_t,frucd::Tire_side::right,state_start,control_start>;

#include "tire_frucd_p6_mnc.hpp"

#endif
