"""Shared FRUCD MF6.2 + MNC sampling and reduced-tire utilities.

The canonical convention used by this module is fastest-lap's convention:

* normal load is a positive magnitude;
* positive kappa produces positive longitudinal force;
* lambda = -Vy/Vx = tan(alpha_FRUCD);
* positive lambda produces positive fastest-lap lateral force;
* angles passed to the FRUCD force equations are radians, while inclination
  remains in degrees to match the fitted ``Tire.Pacejka`` schema.

Only force terms are reproduced because fastest-lap's reduced Pacejka models
do not expose the FRUCD contact-patch moments.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET

try:
    import numpy as np
    from scipy.io import loadmat
except ImportError as exc:  # pragma: no cover - exercised by the CLI
    raise SystemExit(
        "FRUCD tire reduction requires NumPy and SciPy. Install them with "
        "`python -m pip install -r tools/tire_reduction/requirements.txt`."
    ) from exc


Array = np.ndarray


def _array(value: Any, size: int, name: str) -> Array:
    result = np.asarray(value, dtype=float).reshape(-1)
    if result.size < size:
        raise ValueError(f"Tire.Pacejka.{name} needs {size} coefficients; found {result.size}")
    result = result[:size]
    if not np.all(np.isfinite(result)):
        raise ValueError(f"Tire.Pacejka.{name} contains non-finite coefficients")
    return result


def _scalar(value: Any, name: str) -> float:
    result = float(np.asarray(value, dtype=float).reshape(-1)[0])
    if not math.isfinite(result):
        raise ValueError(f"Tire.Pacejka.{name} is not finite")
    return result


def _field(node: Mapping[str, Any], path: Sequence[str], default: Any = None) -> Any:
    current: Any = node
    for part in path:
        if not isinstance(current, Mapping) or part not in current:
            if default is not None:
                return default
            raise ValueError("MAT file is missing Tire.Pacejka." + ".".join(path))
        current = current[part]
    return current


@dataclass(frozen=True)
class FrucdParameters:
    nominal_load: float
    nominal_pressure_kpa: float
    radius_m: float
    nominal_velocity_mps: float
    L_Cx: float
    L_mux: float
    L_Ex: float
    L_Kxk: float
    L_Hx: float
    L_Vx: float
    p_Cx: float
    p_Dx: Array
    p_Ex: Array
    p_Kx: Array
    p_Hx: Array
    p_Vx: Array
    p_Px: Array
    p_Cy: float
    p_Dy: Array
    p_Ey: Array
    p_Ky: Array
    p_Hy: Array
    p_Vy: Array
    p_Py: Array

    @classmethod
    def from_mat(cls, filename: Path) -> "FrucdParameters":
        try:
            raw = loadmat(filename, simplify_cells=True)
        except NotImplementedError as exc:
            raise ValueError(
                f"{filename} is probably a MATLAB v7.3/HDF5 file; export Tire as a v5 MAT file"
            ) from exc

        tire: Any = raw.get("Tire")
        if not isinstance(tire, Mapping) or "Pacejka" not in tire:
            tire = next(
                (
                    value
                    for key, value in raw.items()
                    if not key.startswith("__")
                    and isinstance(value, Mapping)
                    and "Pacejka" in value
                ),
                None,
            )
        if not isinstance(tire, Mapping) or not isinstance(tire.get("Pacejka"), Mapping):
            raise ValueError(f"{filename} has no Tire.Pacejka structure")
        p = tire["Pacejka"]

        return cls(
            nominal_load=_scalar(_field(p, ("Fzo",)), "Fzo"),
            nominal_pressure_kpa=_scalar(_field(p, ("Pio",)), "Pio"),
            radius_m=_scalar(_field(p, ("Ro",)), "Ro"),
            nominal_velocity_mps=_scalar(_field(p, ("Vo",), 15.0), "Vo"),
            L_Cx=_scalar(_field(p, ("L", "C", "x"), 1.0), "L.C.x"),
            L_mux=_scalar(_field(p, ("L", "mu", "x"), 1.0), "L.mu.x"),
            L_Ex=_scalar(_field(p, ("L", "E", "x"), 1.0), "L.E.x"),
            L_Kxk=_scalar(_field(p, ("L", "K", "x", "k"), 1.0), "L.K.x.k"),
            L_Hx=_scalar(_field(p, ("L", "H", "x"), 1.0), "L.H.x"),
            L_Vx=_scalar(_field(p, ("L", "V", "x"), 1.0), "L.V.x"),
            p_Cx=_scalar(_field(p, ("p", "C", "x")), "p.C.x"),
            p_Dx=_array(_field(p, ("p", "D", "x")), 3, "p.D.x"),
            p_Ex=_array(_field(p, ("p", "E", "x")), 4, "p.E.x"),
            p_Kx=_array(_field(p, ("p", "K", "x")), 3, "p.K.x"),
            p_Hx=_array(_field(p, ("p", "H", "x")), 2, "p.H.x"),
            p_Vx=_array(_field(p, ("p", "V", "x")), 2, "p.V.x"),
            p_Px=_array(_field(p, ("p", "P", "x")), 4, "p.P.x"),
            p_Cy=_scalar(_field(p, ("p", "C", "y")), "p.C.y"),
            p_Dy=_array(_field(p, ("p", "D", "y")), 3, "p.D.y"),
            p_Ey=_array(_field(p, ("p", "E", "y")), 5, "p.E.y"),
            p_Ky=_array(_field(p, ("p", "K", "y")), 7, "p.K.y"),
            p_Hy=_array(_field(p, ("p", "H", "y")), 2, "p.H.y"),
            p_Vy=_array(_field(p, ("p", "V", "y")), 4, "p.V.y"),
            p_Py=_array(_field(p, ("p", "P", "y")), 5, "p.P.y"),
        )


def _nz(value: Array | float) -> Array:
    value = np.asarray(value, dtype=float)
    return np.where(np.abs(value) >= 1.0e-12, value, np.where(value < 0.0, -1.0e-12, 1.0e-12))


@dataclass(frozen=True)
class PureLongitudinal:
    force: Array
    stiffness: Array
    null_slip: Array


@dataclass(frozen=True)
class PureLateral:
    force: Array
    stiffness: Array
    null_slip: Array


def pure_longitudinal(
    p: FrucdParameters,
    kappa: Array,
    load: Array,
    pressure_kpa: float,
    inclination_deg: float,
) -> PureLongitudinal:
    kappa, load = np.broadcast_arrays(np.asarray(kappa, dtype=float), np.asarray(load, dtype=float))
    dfz = (load - p.nominal_load) / p.nominal_load
    dpi = (pressure_kpa - p.nominal_pressure_kpa) / p.nominal_pressure_kpa
    cx = p.p_Cx * p.L_Cx
    dx = (
        (p.p_Dx[0] + p.p_Dx[1] * dfz)
        * (1.0 + p.p_Px[2] * dpi + p.p_Px[3] * dpi * dpi)
        * (1.0 - p.p_Dx[2] * inclination_deg * inclination_deg)
        * load
        * p.L_mux
    )
    ex = (
        (p.p_Ex[0] + p.p_Ex[1] * dfz + p.p_Ex[2] * dfz * dfz)
        * (1.0 - p.p_Ex[3] * np.sign(kappa))
        * p.L_Ex
    )
    kxk = (
        load
        * (p.p_Kx[0] + p.p_Kx[1] * dfz)
        * np.exp(p.p_Kx[2] * dfz)
        * (1.0 + p.p_Px[0] * dpi + p.p_Px[1] * dpi * dpi)
        * p.L_Kxk
    )
    vx = load * (p.p_Vx[0] + p.p_Vx[1] * dfz) * p.L_Vx
    hx = (p.p_Hx[0] + p.p_Hx[1] * dfz) * p.L_Hx
    bx = kxk / _nz(cx * dx)
    shifted = kappa + hx
    force = dx * np.sin(cx * np.arctan((1.0 - ex) * bx * shifted + ex * np.arctan(bx * shifted))) + vx
    return PureLongitudinal(force, kxk, -vx / _nz(kxk) - hx)


def pure_lateral(
    p: FrucdParameters,
    alpha_rad: Array,
    load: Array,
    pressure_kpa: float,
    inclination_deg: float,
    side_sign: float,
) -> PureLateral:
    alpha_rad, load = np.broadcast_arrays(
        np.asarray(alpha_rad, dtype=float), np.asarray(load, dtype=float)
    )
    dfz = (load - p.nominal_load) / p.nominal_load
    dpi = (pressure_kpa - p.nominal_pressure_kpa) / p.nominal_pressure_kpa
    cy = p.p_Cy
    dy = (
        (p.p_Dy[0] + p.p_Dy[1] * dfz)
        * (1.0 + p.p_Py[2] * dpi + p.p_Py[3] * dpi * dpi)
        * (1.0 - p.p_Dy[2] * inclination_deg * inclination_deg)
        * load
    )
    pressure_den = (
        (p.p_Ky[1] + p.p_Ky[4] * inclination_deg * inclination_deg)
        * (1.0 + p.p_Py[1] * dpi)
    )
    kya = (
        p.p_Ky[0]
        * p.nominal_load
        * (1.0 + p.p_Py[0] * dpi)
        * (1.0 - p.p_Ky[2] * abs(inclination_deg))
        * np.sin(p.p_Ky[3] * np.arctan((load / p.nominal_load) / _nz(pressure_den)))
    )
    kyg0 = load * (p.p_Ky[5] + p.p_Ky[6] * dfz) * (1.0 + p.p_Py[4] * dpi)
    by = kya / _nz(cy * dy)
    vyg = load * (p.p_Vy[2] + p.p_Vy[3] * dfz) * inclination_deg
    vy = load * (p.p_Vy[0] + p.p_Vy[1] * dfz) + vyg
    hy = (p.p_Hy[0] + p.p_Hy[1] * dfz) * (kyg0 * inclination_deg - vyg) / _nz(kya)
    ey = (
        (p.p_Ey[0] + p.p_Ey[1] * dfz)
        * (
            1.0
            + p.p_Ey[4] * inclination_deg * inclination_deg
            - (p.p_Ey[2] + p.p_Ey[3] * inclination_deg) * np.sign(alpha_rad + hy)
        )
    )
    shifted = alpha_rad + hy
    force = (
        dy * np.sin(cy * np.arctan((1.0 - ey) * by * shifted + ey * np.arctan(by * shifted)))
        + vy
    ) * side_sign
    return PureLateral(force, kya, -vy / _nz(kya) - hy)


def mnc_forces(
    alpha_rad: Array,
    kappa: Array,
    fx0: Array,
    fy0: Array,
    kxk: Array,
    kya: Array,
    kappa0: Array,
    alpha0: Array,
) -> tuple[Array, Array]:
    delta_kappa = kappa - kappa0
    delta_alpha = alpha_rad - alpha0
    sin_alpha = np.sin(delta_alpha)
    cos_alpha = np.cos(delta_alpha)
    denominator = np.sqrt(
        delta_kappa * delta_kappa * fy0 * fy0
        + fx0 * fx0 * np.tan(delta_alpha) ** 2
        + 1.0e-24
    )
    shared = fx0 * fy0 / denominator
    remaining = 1.0 - np.abs(delta_kappa)
    fx = (
        np.abs(
            shared
            * np.sqrt(
                delta_kappa * delta_kappa * kya * kya
                + remaining * remaining * cos_alpha * cos_alpha * fx0 * fx0
                + 1.0e-24
            )
            / _nz(kya)
        )
        * np.sign(fx0)
    )
    fy = (
        np.abs(
            shared
            * np.sqrt(
                remaining * remaining * cos_alpha * cos_alpha * fy0 * fy0
                + sin_alpha * sin_alpha * kxk * kxk
                + 1.0e-24
            )
            / _nz(kxk * cos_alpha)
        )
        * np.sign(fy0)
    )
    return fx, fy


class FrucdTeacher:
    """Vectorized copy of the fastest-lap FRUCD force path."""

    def __init__(
        self,
        parameters: FrucdParameters,
        pressure_kpa: float,
        camber_deg: float,
        longitudinal_correction: float,
        lateral_correction: float,
        side: str,
    ) -> None:
        self.p = parameters
        self.pressure_kpa = pressure_kpa
        self.camber_deg = camber_deg
        self.longitudinal_correction = longitudinal_correction
        self.lateral_correction = lateral_correction
        self.side = side

    def _evaluate_side(
        self, kappa: Array, lambda_dimensionless: Array, load: Array, side_sign: float
    ) -> tuple[Array, Array]:
        kappa, lambda_dimensionless, load = np.broadcast_arrays(
            np.asarray(kappa, dtype=float),
            np.asarray(lambda_dimensionless, dtype=float),
            np.asarray(load, dtype=float),
        )
        # Tire_frucd_p6_mnc performs atan(lambda) before calling P6_mnc_model.
        alpha = np.arctan(lambda_dimensionless) * side_sign
        inclination = self.camber_deg * side_sign
        longitudinal = pure_longitudinal(self.p, kappa, load, self.pressure_kpa, inclination)
        lateral = pure_lateral(
            self.p, alpha, load, self.pressure_kpa, inclination, side_sign
        )
        fx, fy_frucd = mnc_forces(
            alpha,
            kappa,
            longitudinal.force,
            lateral.force,
            longitudinal.stiffness,
            lateral.stiffness,
            longitudinal.null_slip,
            lateral.null_slip,
        )
        activity = (load > 0.0).astype(float)
        # FRUCD is y-left/z-up; Tire_frucd_p6_mnc maps Fy to fastest-lap y-right.
        return (
            activity * self.longitudinal_correction * fx,
            -activity * self.lateral_correction * fy_frucd,
        )

    def evaluate(
        self, kappa: Array, lambda_dimensionless: Array, load: Array
    ) -> tuple[Array, Array]:
        if self.side == "left":
            return self._evaluate_side(kappa, lambda_dimensionless, load, 1.0)
        if self.side == "right":
            return self._evaluate_side(kappa, lambda_dimensionless, load, -1.0)
        left = self._evaluate_side(kappa, lambda_dimensionless, load, 1.0)
        right = self._evaluate_side(kappa, lambda_dimensionless, load, -1.0)
        return 0.5 * (left[0] + right[0]), 0.5 * (left[1] + right[1])


def convention_report(teacher: FrucdTeacher, load: float) -> dict[str, Any]:
    h = 1.0e-5
    fx_plus, _ = teacher.evaluate(np.array(h), np.array(0.0), np.array(load))
    fx_minus, _ = teacher.evaluate(np.array(-h), np.array(0.0), np.array(load))
    _, fy_plus = teacher.evaluate(np.array(0.0), np.array(h), np.array(load))
    _, fy_minus = teacher.evaluate(np.array(0.0), np.array(-h), np.array(load))
    fx0, fy0 = teacher.evaluate(np.array(0.0), np.array(0.0), np.array(load))
    left = teacher._evaluate_side(np.array(0.08), np.array(0.12), np.array(load), 1.0)
    right = teacher._evaluate_side(np.array(0.08), np.array(0.12), np.array(load), -1.0)
    scale = max(load, 1.0)
    return {
        "canonical_convention": {
            "kappa": "(wheel_speed_radius - Vx) / Vx; positive is driving",
            "lambda": "-Vy / Vx = tan(alpha_FRUCD)",
            "normal_load": "positive magnitude in N",
            "force_axes": "fastest-lap: x-forward, y-right, z-down",
            "frucd_slip_angle_unit": "rad",
            "frucd_inclination_unit": "deg",
        },
        "probe_load_N": load,
        "dFx_dKappa_N": float((fx_plus - fx_minus) / (2.0 * h)),
        "dFy_dLambda_N": float((fy_plus - fy_minus) / (2.0 * h)),
        "zero_slip_Fx_N": float(fx0),
        "zero_slip_Fy_N": float(fy0),
        "left_right_force_difference_over_Fz": {
            "Fx": float(abs(left[0] - right[0]) / scale),
            "Fy": float(abs(left[1] - right[1]) / scale),
        },
    }


def validate_convention(report: Mapping[str, Any], allow_warning: bool) -> list[str]:
    warnings: list[str] = []
    fatal: list[str] = []
    if report["dFx_dKappa_N"] <= 0.0:
        fatal.append("canonical dFx/dkappa is not positive")
    if report["dFy_dLambda_N"] <= 0.0:
        fatal.append("canonical dFy/dlambda is not positive")
    asymmetry = report["left_right_force_difference_over_Fz"]
    if max(asymmetry.values()) > 0.05:
        warnings.append("left/right force mismatch exceeds 5% of normal load")
    warnings = fatal + warnings
    if fatal and not allow_warning:
        raise ValueError(
            "convention probes failed: " + "; ".join(fatal)
            + ". Inspect the MAT convention or pass --allow-convention-warning deliberately."
        )
    return warnings


def normalized_metrics(target: Array, predicted: Array, load: Array) -> dict[str, float]:
    target, predicted, load = np.broadcast_arrays(target, predicted, load)
    normalized = (predicted - target) / np.maximum(np.abs(load), 1.0)
    absolute = predicted - target
    return {
        "rms_over_Fz": float(np.sqrt(np.mean(normalized * normalized))),
        "p95_abs_over_Fz": float(np.percentile(np.abs(normalized), 95.0)),
        "max_abs_over_Fz": float(np.max(np.abs(normalized))),
        "rmse_N": float(np.sqrt(np.mean(absolute * absolute))),
        "max_abs_N": float(np.max(np.abs(absolute))),
    }


def force_grid(
    teacher: FrucdTeacher,
    fz_min: float,
    fz_max: float,
    load_count: int,
    kappa_limit: float,
    alpha_limit_deg: float,
    slip_count: int,
) -> dict[str, Array]:
    loads = np.linspace(fz_min, fz_max, load_count)
    kappas = np.linspace(-kappa_limit, kappa_limit, slip_count)
    lambdas = np.tan(np.deg2rad(np.linspace(-alpha_limit_deg, alpha_limit_deg, slip_count)))
    load, kappa, lamb = np.meshgrid(loads, kappas, lambdas, indexing="ij")
    fx, fy = teacher.evaluate(kappa, lamb, load)
    return {"load": load, "kappa": kappa, "lambda": lamb, "Fx": fx, "Fy": fy}


def add_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("mat_file", type=Path, help="fitted MATLAB v5 file containing Tire.Pacejka")
    parser.add_argument("--output", type=Path, required=True, help="output fastest-lap tire XML")
    parser.add_argument("--report", type=Path, help="JSON report (default: OUTPUT.report.json)")
    parser.add_argument("--xml-element", default="front-tire", help="XML element name")
    parser.add_argument("--pressure-kpa", type=float, help="fixed pressure; default is Tire.Pacejka.Pio")
    parser.add_argument("--camber-deg", type=float, default=0.0, help="fixed inclination/camber")
    parser.add_argument("--fz-min", type=float, default=200.0)
    parser.add_argument("--fz-max", type=float, default=1800.0)
    parser.add_argument("--load-count", type=int, default=7)
    parser.add_argument("--slip-count", type=int, default=17)
    parser.add_argument("--kappa-limit", type=float, default=0.25)
    parser.add_argument("--alpha-limit-deg", type=float, default=18.0)
    parser.add_argument("--side", choices=("average", "left", "right"), default="average")
    parser.add_argument("--longitudinal-correction", type=float, default=0.7)
    parser.add_argument("--lateral-correction", type=float, default=0.7)
    parser.add_argument("--radial-stiffness", type=float, default=100000.0)
    parser.add_argument("--radial-damping", type=float, default=0.0)
    parser.add_argument("--fz-max-ref2", type=float, default=1.0)
    parser.add_argument(
        "--allow-convention-warning",
        action="store_true",
        help="continue despite a failed sign or left/right convention probe",
    )


def validate_common_arguments(args: argparse.Namespace, p: FrucdParameters) -> float:
    if not args.mat_file.is_file():
        raise ValueError(f"MAT file does not exist: {args.mat_file}")
    if not 0.0 < args.fz_min < args.fz_max:
        raise ValueError("expected 0 < --fz-min < --fz-max")
    if args.load_count < 2 or args.slip_count < 5:
        raise ValueError("--load-count must be >= 2 and --slip-count must be >= 5")
    if args.slip_count % 2 == 0:
        raise ValueError("--slip-count must be odd so zero slip is sampled")
    if args.kappa_limit <= 0.0 or not 0.0 < args.alpha_limit_deg < 80.0:
        raise ValueError("slip limits must be positive and alpha must be below 80 degrees")
    if not 0.0 <= args.longitudinal_correction <= 1.0:
        raise ValueError("--longitudinal-correction must be in [0,1]")
    if not 0.0 <= args.lateral_correction <= 1.0:
        raise ValueError("--lateral-correction must be in [0,1]")
    return p.nominal_pressure_kpa if args.pressure_kpa is None else args.pressure_kpa


def make_teacher(args: argparse.Namespace, p: FrucdParameters, pressure: float) -> FrucdTeacher:
    return FrucdTeacher(
        p,
        pressure,
        args.camber_deg,
        args.longitudinal_correction,
        args.lateral_correction,
        args.side,
    )


def _format_float(value: float) -> str:
    return format(float(value), ".17g")


def tire_xml(
    element_name: str,
    model: str,
    radius_m: float,
    radial_stiffness: float,
    radial_damping: float,
    fz_max_ref2: float,
    parameters: Iterable[tuple[str, float]],
) -> ET.Element:
    root = ET.Element(element_name, {"model": model, "type": "normal"})
    base = (
        ("radius", radius_m),
        ("radial-stiffness", radial_stiffness),
        ("radial-damping", radial_damping),
        ("Fz-max-ref2", fz_max_ref2),
    )
    for path, value in (*base, *tuple(parameters)):
        node = root
        for part in path.split("/"):
            child = node.find(part)
            node = child if child is not None else ET.SubElement(node, part)
        node.text = _format_float(value)
    ET.indent(root, space="    ")
    return root


def write_outputs(
    args: argparse.Namespace,
    root: ET.Element,
    report: Mapping[str, Any],
) -> tuple[Path, Path]:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(args.output, encoding="utf-8", xml_declaration=True)
    report_path = args.report or args.output.with_suffix(args.output.suffix + ".report.json")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return args.output.resolve(), report_path.resolve()


def base_report(
    args: argparse.Namespace,
    p: FrucdParameters,
    pressure: float,
    convention: Mapping[str, Any],
    warnings: Sequence[str],
) -> dict[str, Any]:
    return {
        "source_mat": str(args.mat_file.resolve()),
        "source_schema": "Tire.Pacejka MF6.2 pure force + MNC combined slip",
        "operating_point": {
            "pressure_kpa": pressure,
            "camber_deg": args.camber_deg,
            "side": args.side,
            "longitudinal_correction": args.longitudinal_correction,
            "lateral_correction": args.lateral_correction,
            "fz_range_N": [args.fz_min, args.fz_max],
            "kappa_range": [-args.kappa_limit, args.kappa_limit],
            "alpha_range_deg": [-args.alpha_limit_deg, args.alpha_limit_deg],
        },
        "source_nominal": {
            "load_N": p.nominal_load,
            "pressure_kpa": p.nominal_pressure_kpa,
            "radius_m": p.radius_m,
        },
        "convention": convention,
        "warnings": list(warnings),
        "known_reduction_limits": [
            "contact-patch Mx, My and Mz are not represented",
            "pressure and camber are frozen at the selected operating point",
            "horizontal/vertical MF shifts cannot be represented exactly",
        ],
    }
