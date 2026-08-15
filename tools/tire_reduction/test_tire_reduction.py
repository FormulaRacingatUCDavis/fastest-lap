"""Focused tests for the offline FRUCD tire converters."""

from __future__ import annotations

import os
from pathlib import Path
import unittest

import numpy as np

from frucd_reduction_common import FrucdParameters, FrucdTeacher, convention_report
from reduce_to_pacejka_simple import evaluate_simple
from reduce_to_pacejka_standard import evaluate_standard


class ReducedModelTests(unittest.TestCase):
    def test_standard_model_is_zero_at_zero_slip(self) -> None:
        parameters = np.array(
            [1.5, 1.8, 0.2, 20.0, 0.0, 0.0, 1.4, 1.7, 0.1, 15.0, 1.0, 2.0, 5.0, 1.0, 5.0, 1.0]
        )
        fx, fy = evaluate_standard(parameters, np.array(0.0), np.array(0.0), np.array(1000.0), 1000.0)
        self.assertEqual(float(fx), 0.0)
        self.assertEqual(float(fy), 0.0)

    def test_simple_model_is_zero_at_zero_slip(self) -> None:
        parameters = np.array([1.5, 1.4, 1.6, 1.5, 0.1, 0.11, 0.12, 0.13, 1.9, 1.9])
        fx, fy = evaluate_simple(
            parameters, np.array(0.0), np.array(0.0), np.array(1000.0), 500.0, 1500.0
        )
        self.assertAlmostEqual(float(fx), 0.0, places=12)
        self.assertAlmostEqual(float(fy), 0.0, places=12)


@unittest.skipUnless(os.environ.get("FRUCD_TIRE_TEST_FILE"), "set FRUCD_TIRE_TEST_FILE for MAT regression")
class RealMatRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.parameters = FrucdParameters.from_mat(Path(os.environ["FRUCD_TIRE_TEST_FILE"]))

    def test_mat_schema(self) -> None:
        self.assertEqual(self.parameters.nominal_load, 3000.0)
        self.assertEqual(self.parameters.nominal_pressure_kpa, 70.0)
        self.assertEqual(self.parameters.radius_m, 0.2032)
        self.assertAlmostEqual(self.parameters.p_Cx, 1.3238448807857677, places=14)

    def test_python_teacher_matches_cpp_matlab_golden(self) -> None:
        teacher = FrucdTeacher(self.parameters, 70.0, 0.0, 0.7, 0.7, "left")
        lamb = np.tan(np.deg2rad(5.0))
        fx, fy = teacher.evaluate(np.array(0.0), np.array(lamb), np.array(700.0))
        # C++/MATLAB golden is in FRUCD y-left. Canonical teacher output is y-right.
        self.assertAlmostEqual(float(fx), 0.7 * -11.3394501073762, places=9)
        self.assertAlmostEqual(float(fy), -0.7 * -1572.31682600527, places=9)

    def test_right_tire_pressure_and_camber_match_cpp_matlab_golden(self) -> None:
        teacher = FrucdTeacher(self.parameters, 80.0, -2.0, 0.7, 0.7, "right")
        lamb = np.tan(np.deg2rad(-7.0))
        fx, fy = teacher.evaluate(np.array(-0.08), np.array(lamb), np.array(1500.0))
        self.assertAlmostEqual(float(fx), 0.7 * -2035.53573719931, places=8)
        self.assertAlmostEqual(float(fy), -0.7 * 2691.11672959066, places=8)

    def test_canonical_gradients_are_positive(self) -> None:
        teacher = FrucdTeacher(self.parameters, 70.0, 0.0, 0.7, 0.7, "average")
        report = convention_report(teacher, 1800.0)
        self.assertGreater(report["dFx_dKappa_N"], 0.0)
        self.assertGreater(report["dFy_dLambda_N"], 0.0)


if __name__ == "__main__":
    unittest.main()
