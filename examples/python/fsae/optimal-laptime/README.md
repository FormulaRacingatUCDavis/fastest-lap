# Formula SAE optimal lap-time examples

This directory contains Jupyter and command-line minimum-lap-time examples for
the two reduced Hoosier R20 tire models:

- `Optimal_laptime_pacejka_standard.ipynb` loads
  `fsae-2026-pacejka.xml`;
- `Optimal_laptime_pacejka_simple.ipynb` loads
  `fsae-2026-pacejka-simple.xml`.

Both run the combustion FSAE vehicle on the 1.25 km Vendrell circuit. They use
the same chassis, powertrain, track, optimization mesh, output channels, and
plots so their results can be compared directly. The reduced vehicles contain
all tire parameters in XML and do not need a MATLAB file at run time.

## Runtime

Use a built or installed `fastest_lap` Python runtime containing the
`fsae-pacejka-3dof` and `fsae-pacejka-simple-3dof` types. If it is not installed,
point the examples at its Python directory:

```powershell
$env:FASTESTLAP_PYTHON_DIR = "D:\projects\fastest-lap\build\artifacts\fastestlap-python-windows-mingw64-x86_64\python"
```

NumPy is used to inspect the result arrays. Matplotlib is optional: without it
the solve still runs and the two plotting cells print an installation hint.
Install both for the complete notebook:

```powershell
python -m pip install numpy matplotlib
```

## Run the notebooks

The recommended kernel is the existing Conda environment named
`fastest-lap`. Register it with Jupyter once if it is not already listed:

```powershell
conda activate fastest-lap
python -m ipykernel install --user --name fastest-lap --display-name "Python (fastest-lap)"
```

Then start Jupyter from the repository and open either notebook:

```powershell
conda activate fastest-lap
jupyter lab examples/python/fsae/optimal-laptime
```

Each notebook is self-contained and checks the loaded vehicle type before
solving. They intentionally share the same Vendrell track, mesh, solver
options, output channels, and plots so their lap results can be compared
directly.

## Command-line counterparts

Standard Pacejka:

```powershell
python examples/python/fsae/optimal-laptime/pacejka_standard.py
```

Simple Pacejka:

```powershell
python examples/python/fsae/optimal-laptime/pacejka_simple.py
```

The default mesh uses every second Vendrell point, approximately 250
optimization nodes. Use `--mesh-stride 1` for the full mesh, or a larger value
for a quicker exploratory solve. `--no-plot` suppresses Matplotlib windows.

Before committing time to a nonlinear solve, validate the installed runtime
and vehicle registration with:

```powershell
python examples/python/fsae/optimal-laptime/pacejka_standard.py --validate-only
python examples/python/fsae/optimal-laptime/pacejka_simple.py --validate-only
```

The existing `Optimal_laptime.ipynb` documents the older FRUCD electric/MAT
workflow. The two model-specific notebooks above are the primary examples for
the reduced combustion vehicles; the Python scripts provide the same workflow
for automated and headless runs.
