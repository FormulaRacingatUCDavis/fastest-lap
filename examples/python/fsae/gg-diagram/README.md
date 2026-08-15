# Formula SAE G-G diagram

This notebook computes a G-G diagram for the electric Formula SAE vehicle with
the fitted Hoosier MF6.2 + MNC tire model.

The example uses 60 km/h. The 250 km/h value in the Formula 1 example is
outside the useful operating range of this vehicle: aerodynamic drag exceeds
the available tractive force and the longitudinal maximum becomes negative.

Set `FASTESTLAP_FSAE_TIRE_FILE` when the fitted MAT file is not stored beside
the FSAE vehicle XML. Set `FASTESTLAP_PYTHON_DIR` to the `python` folder of a
built fastest-lap runtime when it is not installed in the active environment.

PowerShell example:

```powershell
$env:FASTESTLAP_PYTHON_DIR = "D:\projects\fastest-lap\build\artifacts\fastestlap-python-windows-mingw64-x86_64-fsae-gg\python"
$env:FASTESTLAP_FSAE_TIRE_FILE = "D:\path\to\Hoosier_R20_16(18)x75(60)-10x8(7).mat"
jupyter notebook examples/python/fsae/gg-diagram/gg-diagram.ipynb
```

Restart the Jupyter kernel after changing the Python runtime path. A loaded
Windows DLL cannot be replaced by changing `sys.path` in the same process.

