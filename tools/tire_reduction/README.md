# FRUCD tire model reduction

These offline tools fit the two native fastest-lap Pacejka models to a fitted
FRUCD `Tire.Pacejka` MATLAB v5 file. NumPy and SciPy are required only while
converting the fit; generated XML has no Python, SciPy, MATLAB, or MAT-file
runtime dependency.

Install the converter dependencies:

```powershell
python -m pip install -r tools/tire_reduction/requirements.txt
```

Fit the reduced standard Pacejka model:

```powershell
python tools/tire_reduction/reduce_to_pacejka_standard.py `
  "D:\FRUCD\Tire-Data\Models\Hoosier_R20_16(18)x75(60)-10x8(7).mat" `
  --output reduced-standard.xml `
  --pressure-kpa 70 --camber-deg 0 --fz-min 200 --fz-max 1800
```

Fit the fastest simple model:

```powershell
python tools/tire_reduction/reduce_to_pacejka_simple.py `
  "D:\FRUCD\Tire-Data\Models\Hoosier_R20_16(18)x75(60)-10x8(7).mat" `
  --output reduced-simple.xml `
  --pressure-kpa 70 --camber-deg 0 --fz-min 200 --fz-max 1800
```

Each command writes a complete tire XML element and an adjacent
`*.report.json`. The report records the operating point, convention probes,
parameters, active parameter bounds, optimizer status, and force errors
normalized by vertical load. The standard converter's final global refinement
balances the full combined-slip grid against pure-slip slices; use
`--pure-weight` to change that balance or `--no-polish` to preserve the two
independent pure-slip fits.

## Conventions

The converters reproduce the convention bridge used by
`Tire_frucd_p6_mnc` before fitting:

- `kappa = (omega*radius - Vx)/Vx`, positive while driving;
- fastest-lap `lambda = -Vy/Vx`, and FRUCD receives `alpha = atan(lambda)`;
- FRUCD y-left force is negated to fastest-lap y-right;
- normal load is passed as a positive magnitude;
- FRUCD slip angle is in radians, but fitted inclination is in degrees;
- left/right tire mirroring is evaluated explicitly.

By default a failed force-gradient sign probe stops conversion. A left/right
discrepancy over 5% of normal load is recorded as a warning because fitted
horizontal and vertical shifts can create real asymmetry. The
`--allow-convention-warning` option overrides a gradient failure deliberately;
all warnings remain in the JSON report.

The default `--side average` produces a symmetric equivalent tire. Use
`--side left` or `--side right` when fitting a fixed non-zero camber operating
point and the vehicle integration can represent side-specific parameters.

## Included Formula SAE vehicles

The repository includes two combustion-vehicle databases generated from the
bundled R20 fit with the default operating domain:

- `database/vehicles/fsae/fsae-2026-pacejka.xml`, vehicle type
  `fsae-pacejka-3dof`;
- `database/vehicles/fsae/fsae-2026-pacejka-simple.xml`, vehicle type
  `fsae-pacejka-simple-3dof`.

Both are registered by the C and Python APIs and can be passed directly to
`create_vehicle_from_xml`.

## Reduction limits

Both native models freeze pressure and camber at the values supplied to the
converter. Neither represents FRUCD contact-patch `Mx`, `My`, or `Mz`. The
standard model also has no horizontal or vertical Magic Formula shifts, so its
zero-slip residual force is irreducible and is reported by the convention
probe.
