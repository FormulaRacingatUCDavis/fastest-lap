# Formula SAE vehicle and tire model

This example loads the combustion and electric Formula SAE vehicle models and
evaluates the Hoosier MF6.2 + MNC tire through the Python API.

The notebook demonstrates how to:

- load `fsae-3dof` and `fsae-electric-3dof` vehicles from XML;
- use the fitted `Hoosier_R20_16(18)x75(60)-10x8(7).mat` tire data;
- set the independent longitudinal and lateral force correction factors; and
- plot pure-longitudinal and pure-lateral tire-force sweeps.

## Prerequisites

Use a built or installed fastest-lap Python package that contains the FSAE and
MF6.2 + MNC APIs. The fitted MAT file is not stored in this repository. Point
the notebook to it with `FASTESTLAP_FSAE_TIRE_FILE`.

The notebook also requires NumPy, Matplotlib, and Jupyter:

```bash
python -m pip install numpy matplotlib notebook
```

Make sure that Jupyter imports the current FSAE-enabled build. When using an
uninstalled build artifact, add its `python` directory to `PYTHONPATH` before
starting Jupyter. For example:

```powershell
$env:PYTHONPATH = "D:\path\to\fastestlap-package\python"
```

The first notebook cell prints `fastest_lap.__file__` and checks for the
required MF6.2 + MNC API. If it reports a stale build, restart the kernel after
correcting `PYTHONPATH`; Python otherwise keeps the previously imported module
in memory.

PowerShell:

```powershell
$env:FASTESTLAP_FSAE_TIRE_FILE = "D:\path\to\Hoosier_R20_16(18)x75(60)-10x8(7).mat"
jupyter notebook examples/python/fsae/vehicle-model/fsae_vehicle_model.ipynb
```

Linux or macOS:

```bash
export FASTESTLAP_FSAE_TIRE_FILE=/path/to/Hoosier_R20_16\(18\)x75\(60\)-10x8\(7\).mat
jupyter notebook examples/python/fsae/vehicle-model/fsae_vehicle_model.ipynb
```

Alternatively, place the MAT file beside the FSAE vehicle XML files in
`database/vehicles/fsae`. The notebook writes temporary XML copies containing
the resolved MAT path; the database XML files are not modified.
