#!/usr/bin/env python3
"""Fit fastest-lap's reduced 18-parameter Pacejka model to a FRUCD MAT fit."""

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


LONG_LOWER = np.array([0.5, 0.1, -5.0, 0.01, -100.0, -5.0])
LONG_UPPER = np.array([3.0, 5.0, 1.0, 100.0, 100.0, 5.0])
LAT_LOWER = np.array([0.5, 0.1, -5.0, 0.01, 0.02, 0.2])
LAT_UPPER = np.array([3.0, 5.0, 1.0, 100.0, 10.0, 5.0])
COMBINED_LOWER = np.array([0.01, 0.05, 0.01, 0.05])
COMBINED_UPPER = np.array([100.0, 3.0, 100.0, 3.0])


def _clip_initial(value: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(value, lower + 1.0e-8), upper - 1.0e-8)


def pure_longitudinal(x: np.ndarray, kappa: np.ndarray, load: np.ndarray, fz0: float) -> np.ndarray:
    cx, dx, ex, kx1, kx2, kx3 = x
    dfz = (load - fz0) / fz0
    bx = (kx1 + kx2 * dfz) * np.exp(kx3 * dfz) / (cx * dx)
    bk = bx * kappa
    return dx * load * np.sin(cx * np.arctan(bk - ex * (bk - np.arctan(bk))))


def pure_lateral(x: np.ndarray, lamb: np.ndarray, load: np.ndarray, fz0: float) -> np.ndarray:
    cy, dy, ey, ky1, ky2, ky4 = x
    by = ky1 * fz0 * np.sin(ky4 * np.arctan(load / (fz0 * ky2))) / (cy * dy * load)
    bl = by * lamb
    return dy * load * np.sin(cy * np.arctan(bl - ey * (bl - np.arctan(bl))))


def evaluate_standard(
    x: np.ndarray, kappa: np.ndarray, lamb: np.ndarray, load: np.ndarray, fz0: float
) -> tuple[np.ndarray, np.ndarray]:
    fx0 = pure_longitudinal(x[:6], kappa, load, fz0)
    fy0 = pure_lateral(x[6:12], lamb, load, fz0)
    rbx, rcx, rby, rcy = x[12:]
    return (
        np.cos(rcx * np.arctan(rbx * lamb)) * fx0,
        np.cos(rcy * np.arctan(rby * kappa)) * fy0,
    )


def _fit(args: argparse.Namespace) -> tuple[dict[str, float], dict]:
    p = FrucdParameters.from_mat(args.mat_file)
    pressure = validate_common_arguments(args, p)
    teacher = make_teacher(args, p, pressure)
    convention = convention_report(teacher, min(max(p.nominal_load, args.fz_min), args.fz_max))
    warnings = validate_convention(convention, args.allow_convention_warning)

    pure_loads, pure_slips = np.meshgrid(
        np.linspace(args.fz_min, args.fz_max, max(args.load_count, 7)),
        np.linspace(-args.kappa_limit, args.kappa_limit, max(args.slip_count * 3, 51)),
        indexing="ij",
    )
    zeros = np.zeros_like(pure_slips)
    target_fx, _ = teacher.evaluate(pure_slips, zeros, pure_loads)

    long_initial = _clip_initial(
        np.array(
            [
                p.p_Cx * p.L_Cx,
                args.longitudinal_correction * p.p_Dx[0] * p.L_mux,
                p.p_Ex[0] * p.L_Ex,
                args.longitudinal_correction * p.p_Kx[0] * p.L_Kxk,
                args.longitudinal_correction * p.p_Kx[1] * p.L_Kxk,
                p.p_Kx[2],
            ]
        ),
        LONG_LOWER,
        LONG_UPPER,
    )
    long_fit = least_squares(
        lambda x: ((pure_longitudinal(x, pure_slips, pure_loads, p.nominal_load) - target_fx) / pure_loads).ravel(),
        long_initial,
        bounds=(LONG_LOWER, LONG_UPPER),
        max_nfev=args.max_nfev,
        loss="soft_l1",
        f_scale=0.03,
    )

    alpha_slips = np.linspace(-args.alpha_limit_deg, args.alpha_limit_deg, max(args.slip_count * 3, 51))
    lambdas = np.tan(np.deg2rad(alpha_slips))
    lateral_loads, lateral_lambdas = np.meshgrid(
        np.linspace(args.fz_min, args.fz_max, max(args.load_count, 7)), lambdas, indexing="ij"
    )
    lateral_zeros = np.zeros_like(lateral_lambdas)
    _, target_fy = teacher.evaluate(lateral_zeros, lateral_lambdas, lateral_loads)
    lat_initial = _clip_initial(
        np.array(
            [
                p.p_Cy,
                args.lateral_correction * p.p_Dy[0],
                p.p_Ey[0],
                -args.lateral_correction * p.p_Ky[0],
                abs(p.p_Ky[1]),
                p.p_Ky[3],
            ]
        ),
        LAT_LOWER,
        LAT_UPPER,
    )
    lat_fit = least_squares(
        lambda x: ((pure_lateral(x, lateral_lambdas, lateral_loads, p.nominal_load) - target_fy) / lateral_loads).ravel(),
        lat_initial,
        bounds=(LAT_LOWER, LAT_UPPER),
        max_nfev=args.max_nfev,
        loss="soft_l1",
        f_scale=0.03,
    )

    grid = force_grid(
        teacher,
        args.fz_min,
        args.fz_max,
        args.load_count,
        args.kappa_limit,
        args.alpha_limit_deg,
        args.slip_count,
    )
    pure_x = np.concatenate((long_fit.x, lat_fit.x))

    def combined_residual(combined: np.ndarray) -> np.ndarray:
        x = np.concatenate((pure_x, combined))
        fx, fy = evaluate_standard(x, grid["kappa"], grid["lambda"], grid["load"], p.nominal_load)
        return np.concatenate(
            (((fx - grid["Fx"]) / grid["load"]).ravel(), ((fy - grid["Fy"]) / grid["load"]).ravel())
        )

    combined_fit = least_squares(
        combined_residual,
        np.array([5.0, 1.0, 5.0, 1.0]),
        bounds=(COMBINED_LOWER, COMBINED_UPPER),
        max_nfev=args.max_nfev,
        loss="soft_l1",
        f_scale=0.04,
    )
    fitted = np.concatenate((pure_x, combined_fit.x))

    if not args.no_polish:
        lower = np.concatenate((LONG_LOWER, LAT_LOWER, COMBINED_LOWER))
        upper = np.concatenate((LONG_UPPER, LAT_UPPER, COMBINED_UPPER))

        def global_residual(x: np.ndarray) -> np.ndarray:
            fx, fy = evaluate_standard(x, grid["kappa"], grid["lambda"], grid["load"], p.nominal_load)
            combined = np.concatenate(
                (((fx - grid["Fx"]) / grid["load"]).ravel(), ((fy - grid["Fy"]) / grid["load"]).ravel())
            )
            pure_fx = (pure_longitudinal(x[:6], pure_slips, pure_loads, p.nominal_load) - target_fx) / pure_loads
            pure_fy = (pure_lateral(x[6:12], lateral_lambdas, lateral_loads, p.nominal_load) - target_fy) / lateral_loads
            return np.concatenate(
                (combined, args.pure_weight * pure_fx.ravel(), args.pure_weight * pure_fy.ravel())
            )

        polished = least_squares(
            global_residual,
            fitted,
            bounds=(lower, upper),
            max_nfev=args.max_nfev,
            loss="soft_l1",
            f_scale=0.04,
        )
        fitted = polished.x
        polish_status = {"success": polished.success, "message": polished.message, "nfev": polished.nfev}
    else:
        polish_status = {"skipped": True}

    names = (
        "pCx1", "pDx1", "pEx1", "pKx1", "pKx2", "pKx3",
        "pCy1", "pDy1", "pEy1", "pKy1", "pKy2", "pKy4",
        "rBx1", "rCx1", "rBy1", "rCy1",
    )
    values = dict(zip(names, map(float, fitted)))
    final_lower = np.concatenate((LONG_LOWER, LAT_LOWER, COMBINED_LOWER))
    final_upper = np.concatenate((LONG_UPPER, LAT_UPPER, COMBINED_UPPER))
    bound_tolerance = 1.0e-4 * np.maximum(final_upper - final_lower, 1.0)
    active_bounds = [
        name
        for name, value, lower, upper, tolerance in zip(
            names, fitted, final_lower, final_upper, bound_tolerance
        )
        if value <= lower + tolerance or value >= upper - tolerance
    ]
    if active_bounds:
        warnings.append(
            "reduced parameters reached fit bounds: " + ", ".join(active_bounds)
        )
    predicted_fx, predicted_fy = evaluate_standard(
        fitted, grid["kappa"], grid["lambda"], grid["load"], p.nominal_load
    )
    report = base_report(args, p, pressure, convention, warnings)
    report.update(
        {
            "reduced_model": "tire-pacejka",
            "pure_slice_weight_in_global_polish": args.pure_weight,
            "parameters": {"nominal-vertical-load": p.nominal_load, "lambdaFz0": 1.0, **values},
            "fit_status": {
                "pure_longitudinal": {"success": long_fit.success, "message": long_fit.message, "nfev": long_fit.nfev},
                "pure_lateral": {"success": lat_fit.success, "message": lat_fit.message, "nfev": lat_fit.nfev},
                "combined": {"success": combined_fit.success, "message": combined_fit.message, "nfev": combined_fit.nfev},
                "global_polish": polish_status,
                "parameters_at_bounds": active_bounds,
            },
            "metrics": {
                "combined_grid_Fx": normalized_metrics(grid["Fx"], predicted_fx, grid["load"]),
                "combined_grid_Fy": normalized_metrics(grid["Fy"], predicted_fy, grid["load"]),
                "pure_Fx": normalized_metrics(
                    target_fx, pure_longitudinal(fitted[:6], pure_slips, pure_loads, p.nominal_load), pure_loads
                ),
                "pure_Fy": normalized_metrics(
                    target_fy,
                    pure_lateral(fitted[6:12], lateral_lambdas, lateral_loads, p.nominal_load),
                    lateral_loads,
                ),
            },
        }
    )
    return {"radius": p.radius_m, "fz0": p.nominal_load, **values}, report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_arguments(parser)
    parser.add_argument("--max-nfev", type=int, default=500, help="maximum evaluations per fit stage")
    parser.add_argument(
        "--pure-weight",
        type=float,
        default=2.0,
        help="weight of pure-slip slices during final global refinement",
    )
    parser.add_argument("--no-polish", action="store_true", help="skip final all-parameter refinement")
    args = parser.parse_args(argv)
    if args.pure_weight <= 0.0:
        parser.error("--pure-weight must be positive")
    try:
        fit, report = _fit(args)
        xml_parameters = (
            ("nominal-vertical-load", fit["fz0"]),
            ("lambdaFz0", 1.0),
            ("longitudinal/pure/pCx1", fit["pCx1"]),
            ("longitudinal/pure/pDx1", fit["pDx1"]),
            ("longitudinal/pure/pEx1", fit["pEx1"]),
            ("longitudinal/pure/pKx1", fit["pKx1"]),
            ("longitudinal/pure/pKx2", fit["pKx2"]),
            ("longitudinal/pure/pKx3", fit["pKx3"]),
            ("lateral/pure/pCy1", fit["pCy1"]),
            ("lateral/pure/pDy1", fit["pDy1"]),
            ("lateral/pure/pEy1", fit["pEy1"]),
            ("lateral/pure/pKy1", fit["pKy1"]),
            ("lateral/pure/pKy2", fit["pKy2"]),
            ("lateral/pure/pKy4", fit["pKy4"]),
            ("longitudinal/combined/rBx1", fit["rBx1"]),
            ("longitudinal/combined/rCx1", fit["rCx1"]),
            ("lateral/combined/rBy1", fit["rBy1"]),
            ("lateral/combined/rCy1", fit["rCy1"]),
        )
        root = tire_xml(
            args.xml_element,
            "tire-pacejka",
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
