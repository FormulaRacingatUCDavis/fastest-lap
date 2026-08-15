#!/usr/bin/env python3
"""Fit fastest-lap's 12-parameter simple Pacejka model to a FRUCD MAT fit."""

from __future__ import annotations

import argparse
import sys
from typing import Sequence

import numpy as np
from scipy.optimize import least_squares

from frucd_reduction_common import (
    FrucdParameters,
    add_common_arguments,
    base_report,
    convention_report,
    force_grid,
    make_teacher,
    normalized_metrics,
    tire_xml,
    validate_common_arguments,
    validate_convention,
    write_outputs,
)


LOWER = np.array([1.0001, 1.0001, 1.0001, 1.0001, 0.005, 0.005, 0.005, 0.005, 0.2, 0.2])
UPPER = np.array([5.0, 5.0, 5.0, 5.0, 0.6, 0.6, 1.0, 1.0, 3.0, 3.0])


def _smooth_pos(value: np.ndarray, epsilon: float = 1.0e-5) -> np.ndarray:
    return 0.5 * (value + np.sqrt(value * value + epsilon * epsilon))


def evaluate_simple(
    x: np.ndarray,
    kappa: np.ndarray,
    lamb: np.ndarray,
    load: np.ndarray,
    reference_load_1: float,
    reference_load_2: float,
) -> tuple[np.ndarray, np.ndarray]:
    mux1, mux2, muy1, muy2, kappa1, kappa2, lambda1, lambda2, qx, qy = x
    fraction = (load - reference_load_1) / (reference_load_2 - reference_load_1)
    mux = _smooth_pos(mux1 + fraction * (mux2 - mux1) - 1.0) + 1.0
    muy = _smooth_pos(muy1 + fraction * (muy2 - muy1) - 1.0) + 1.0
    kappa_max = kappa1 + fraction * (kappa2 - kappa1)
    lambda_max = lambda1 + fraction * (lambda2 - lambda1)
    kappa_n = kappa / kappa_max
    lambda_n = lamb / lambda_max
    rho = np.sqrt(kappa_n * kappa_n + lambda_n * lambda_n + 1.0e-12)
    sx = np.pi / (2.0 * np.arctan(qx))
    sy = np.pi / (2.0 * np.arctan(qy))
    return (
        mux * np.sin(qx * np.arctan(sx * rho)) * load * kappa_n / rho,
        muy * np.sin(qy * np.arctan(sy * rho)) * load * lambda_n / rho,
    )


def _peak_initial(teacher, load: float, kappa_limit: float, alpha_limit_deg: float) -> tuple[float, float, float, float]:
    kappa = np.linspace(0.0, kappa_limit, 2001)
    loads = np.full_like(kappa, load)
    fx, _ = teacher.evaluate(kappa, np.zeros_like(kappa), loads)
    ix = int(np.argmax(np.abs(fx)))
    alpha = np.linspace(0.0, alpha_limit_deg, 2001)
    lamb = np.tan(np.deg2rad(alpha))
    _, fy = teacher.evaluate(np.zeros_like(lamb), lamb, np.full_like(lamb, load))
    iy = int(np.argmax(np.abs(fy)))
    return (
        max(abs(float(fx[ix])) / load, 1.0002),
        max(float(kappa[ix]), 0.0051),
        max(float(lamb[iy]), 0.0051),
        max(abs(float(fy[iy])) / load, 1.0002),
    )


def _fit(args: argparse.Namespace) -> tuple[dict[str, float], dict]:
    p = FrucdParameters.from_mat(args.mat_file)
    pressure = validate_common_arguments(args, p)
    teacher = make_teacher(args, p, pressure)
    convention = convention_report(teacher, min(max(p.nominal_load, args.fz_min), args.fz_max))
    warnings = validate_convention(convention, args.allow_convention_warning)
    reference_load_1 = args.reference_load_1 or args.fz_min
    reference_load_2 = args.reference_load_2 or args.fz_max
    if not args.fz_min <= reference_load_1 < reference_load_2 <= args.fz_max:
        raise ValueError("reference loads must satisfy fz-min <= load-1 < load-2 <= fz-max")

    peak1 = _peak_initial(teacher, reference_load_1, args.kappa_limit, args.alpha_limit_deg)
    peak2 = _peak_initial(teacher, reference_load_2, args.kappa_limit, args.alpha_limit_deg)
    initial = np.array(
        [peak1[0], peak2[0], peak1[3], peak2[3], peak1[1], peak2[1], peak1[2], peak2[2], 1.9, 1.9]
    )
    initial = np.minimum(np.maximum(initial, LOWER + 1.0e-8), UPPER - 1.0e-8)
    grid = force_grid(
        teacher,
        args.fz_min,
        args.fz_max,
        args.load_count,
        args.kappa_limit,
        args.alpha_limit_deg,
        args.slip_count,
    )

    def residual(x: np.ndarray) -> np.ndarray:
        fx, fy = evaluate_simple(
            x, grid["kappa"], grid["lambda"], grid["load"], reference_load_1, reference_load_2
        )
        return np.concatenate(
            (((fx - grid["Fx"]) / grid["load"]).ravel(), ((fy - grid["Fy"]) / grid["load"]).ravel())
        )

    fit = least_squares(
        residual,
        initial,
        bounds=(LOWER, UPPER),
        max_nfev=args.max_nfev,
        loss="soft_l1",
        f_scale=0.04,
    )
    predicted_fx, predicted_fy = evaluate_simple(
        fit.x, grid["kappa"], grid["lambda"], grid["load"], reference_load_1, reference_load_2
    )
    names = (
        "mu-x-max-1", "mu-x-max-2", "mu-y-max-1", "mu-y-max-2",
        "kappa-max-1", "kappa-max-2", "lambda-max-1-rad", "lambda-max-2-rad", "Qx", "Qy",
    )
    values = dict(zip(names, map(float, fit.x)))
    bound_tolerance = 1.0e-4 * np.maximum(UPPER - LOWER, 1.0)
    active_bounds = [
        name
        for name, value, lower, upper, tolerance in zip(
            names, fit.x, LOWER, UPPER, bound_tolerance
        )
        if value <= lower + tolerance or value >= upper - tolerance
    ]
    if active_bounds:
        warnings.append(
            "reduced parameters reached fit bounds: " + ", ".join(active_bounds)
        )
    report = base_report(args, p, pressure, convention, warnings)
    report.update(
        {
            "reduced_model": "tire-pacejka-simple",
            "parameters": {
                "reference-load-1": reference_load_1,
                "reference-load-2": reference_load_2,
                **values,
                "lambda-max-1-deg": float(np.rad2deg(fit.x[6])),
                "lambda-max-2-deg": float(np.rad2deg(fit.x[7])),
            },
            "fit_status": {
                "success": fit.success,
                "message": fit.message,
                "nfev": fit.nfev,
                "parameters_at_bounds": active_bounds,
            },
            "metrics": {
                "combined_grid_Fx": normalized_metrics(grid["Fx"], predicted_fx, grid["load"]),
                "combined_grid_Fy": normalized_metrics(grid["Fy"], predicted_fy, grid["load"]),
            },
        }
    )
    return {
        "radius": p.radius_m,
        "reference-load-1": reference_load_1,
        "reference-load-2": reference_load_2,
        **values,
    }, report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_arguments(parser)
    parser.add_argument("--reference-load-1", type=float)
    parser.add_argument("--reference-load-2", type=float)
    parser.add_argument("--max-nfev", type=int, default=1000)
    args = parser.parse_args(argv)
    try:
        fit, report = _fit(args)
        xml_parameters = (
            ("reference-load-1", fit["reference-load-1"]),
            ("reference-load-2", fit["reference-load-2"]),
            ("mu-x-max-1", fit["mu-x-max-1"]),
            ("mu-x-max-2", fit["mu-x-max-2"]),
            ("mu-y-max-1", fit["mu-y-max-1"]),
            ("mu-y-max-2", fit["mu-y-max-2"]),
            ("kappa-max-1", fit["kappa-max-1"]),
            ("kappa-max-2", fit["kappa-max-2"]),
            ("lambda-max-1", np.rad2deg(fit["lambda-max-1-rad"])),
            ("lambda-max-2", np.rad2deg(fit["lambda-max-2-rad"])),
            ("Qx", fit["Qx"]),
            ("Qy", fit["Qy"]),
        )
        root = tire_xml(
            args.xml_element,
            "tire-pacejka-simple",
            fit["radius"],
            args.radial_stiffness,
            args.radial_damping,
            args.fz_max_ref2,
            xml_parameters,
        )
        output, report_path = write_outputs(args, root, report)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(f"Wrote {output}")
    print(f"Wrote {report_path}")
    print(f"Combined RMS/Fz: Fx={report['metrics']['combined_grid_Fx']['rms_over_Fz']:.4f}, "
          f"Fy={report['metrics']['combined_grid_Fy']['rms_over_Fz']:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
