#ifndef FRUCD_P6_MNC_MODEL_H
#define FRUCD_P6_MNC_MODEL_H

#include "frucd_p6_mnc_parameters.h"

#include <cstddef>
#include <memory>

namespace frucd
{

template<typename Timeseries_t>
struct Contact_patch_loads
{
    Timeseries_t Fx = 0.0;
    Timeseries_t Fy = 0.0;
    Timeseries_t Mz = 0.0;
    Timeseries_t Mx = 0.0;
    // Radius function handles embedded in the MATLAB files are deliberately
    // not evaluated in the real-time model. Rolling resistance is therefore
    // reported as zero until a numeric radius schema is exported.
    Timeseries_t My = 0.0;
};

template<typename Timeseries_t>
class P6_mnc_model
{
 public:
    P6_mnc_model();
    explicit P6_mnc_model(std::shared_ptr<const P6_mnc_parameters> parameters);
    explicit P6_mnc_model(const P6_mnc_parameters& parameters);

    Contact_patch_loads<Timeseries_t> evaluate(
        const Timeseries_t& slip_angle_rad,
        const Timeseries_t& slip_ratio,
        const Timeseries_t& normal_load_N,
        const Timeseries_t& pressure_kpa,
        const Timeseries_t& inclination_deg,
        const Timeseries_t& velocity_mps,
        Tire_side side) const;

    Timeseries_t maximum_kappa(const Timeseries_t& normal_load_N) const;
    Timeseries_t maximum_lambda(const Timeseries_t& normal_load_N) const;

    const P6_mnc_parameters& parameters() const { return *_parameters; }
    const Timeseries_t& longitudinal_force_correction_factor() const
    {
        return _longitudinal_force_correction_factor;
    }
    const Timeseries_t& lateral_force_correction_factor() const
    {
        return _lateral_force_correction_factor;
    }
    void set_force_correction_factors(
        const Timeseries_t& longitudinal_factor,
        const Timeseries_t& lateral_factor);
    bool is_ready() const;

 private:
    void initialise_peak_slips();

    std::shared_ptr<const P6_mnc_parameters> _parameters;
    Timeseries_t _longitudinal_force_correction_factor = 0.7;
    Timeseries_t _lateral_force_correction_factor = 0.7;
    double _peak_reference_load_1 = 1.0;
    double _peak_reference_load_2 = 2.0;
    double _maximum_kappa_1 = 0.1;
    double _maximum_kappa_2 = 0.1;
    double _maximum_lambda_1 = 0.1;
    double _maximum_lambda_2 = 0.1;
};

} // namespace frucd

#include "frucd_p6_mnc_model.hpp"

#endif
