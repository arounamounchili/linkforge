"""Tests for mathematical utility functions."""

from __future__ import annotations

from linkforge.core._utils.math_utils import (
    clean_float,
    format_float,
    is_positive_semi_definite_3x3,
    normalize_vector,
    sylvester_minors_3x3,
    symmetric_matrix_eigenvalues_3x3,
)
from linkforge.core.constants import MIN_REASONABLE_INERTIA


def test_clean_float() -> None:
    """Test cleaning floating point values."""
    assert clean_float(1.0) == 1.0
    assert clean_float(1e-11) == 0.0
    assert clean_float(-1e-11) == 0.0
    assert clean_float(1e-9, epsilon=1e-8) == 0.0


def test_format_float() -> None:
    """Test formatting floating point values."""
    assert format_float(1.1234567) == "1.123457"
    assert format_float(1.0) == "1"
    assert format_float(1.100) == "1.1"
    assert format_float(1e-11) == "0"
    assert format_float(-0.0) == "0"
    assert format_float(MIN_REASONABLE_INERTIA) == "1e-09"
    assert format_float(-MIN_REASONABLE_INERTIA) == "-1e-09"


def test_normalize_vector() -> None:
    """Test normalizing 3D vectors."""
    # Unit vectors
    assert normalize_vector(1, 0, 0) == (1.0, 0.0, 0.0)
    assert normalize_vector(0, 5, 0) == (0.0, 1.0, 0.0)

    # Arbitrary vector
    x, y, z = normalize_vector(1, 1, 1)
    mag = (x**2 + y**2 + z**2) ** 0.5
    assert abs(mag - 1.0) < 1e-10

    # Zero vector
    assert normalize_vector(0, 0, 0) == (0.0, 0.0, 0.0)
    assert normalize_vector(1e-11, 0, 0) == (0.0, 0.0, 0.0)


def test_symmetric_matrix_eigenvalues_3x3() -> None:
    """Test analytical 3x3 symmetric matrix eigenvalue calculation."""
    # Identity matrix
    e1, e2, e3 = symmetric_matrix_eigenvalues_3x3(1.0, 1.0, 1.0, 0.0, 0.0, 0.0)
    assert (abs(e1 - 1.0), abs(e2 - 1.0), abs(e3 - 1.0)) < (1e-9, 1e-9, 1e-9)

    # Distinct diagonal matrix
    e1, e2, e3 = symmetric_matrix_eigenvalues_3x3(5.0, 2.0, 9.0, 0.0, 0.0, 0.0)
    assert (abs(e1 - 9.0), abs(e2 - 5.0), abs(e3 - 2.0)) < (1e-9, 1e-9, 1e-9)

    # Known matrix with off-diagonals: [[2, 1, 0], [1, 2, 0], [0, 0, 3]] -> eigenvalues 3, 3, 1
    e1, e2, e3 = symmetric_matrix_eigenvalues_3x3(2.0, 2.0, 3.0, 1.0, 0.0, 0.0)
    assert abs(e1 - 3.0) < 1e-6
    assert abs(e2 - 3.0) < 1e-6
    assert abs(e3 - 1.0) < 1e-6

    # Unphysical matrix with negative eigenvalue: [[2, 3, 0], [3, 2, 0], [0, 0, 2]] -> eigenvalues 5, 2, -1
    e1, e2, e3 = symmetric_matrix_eigenvalues_3x3(2.0, 2.0, 2.0, 3.0, 0.0, 0.0)
    assert abs(e1 - 5.0) < 1e-6
    assert abs(e2 - 2.0) < 1e-6
    assert abs(e3 - (-1.0)) < 1e-6


def test_sylvester_minors_and_psd() -> None:
    """Test Sylvester's principal minors and positive semi-definiteness checks."""
    # Positive definite matrix: [[2, 0.5, 0.3], [0.5, 2, 0.4], [0.3, 0.4, 2]]
    d1, d2, d3 = sylvester_minors_3x3(2.0, 2.0, 2.0, 0.5, 0.3, 0.4)
    assert d1 > 0
    assert d2 > 0
    assert d3 > 0
    assert is_positive_semi_definite_3x3(2.0, 2.0, 2.0, 0.5, 0.3, 0.4)

    # Non-positive definite matrix: [[2, 3, 0], [3, 2, 0], [0, 0, 2]] -> D2 = 4 - 9 = -5
    d1, d2, d3 = sylvester_minors_3x3(2.0, 2.0, 2.0, 3.0, 0.0, 0.0)
    assert d1 == 2.0
    assert d2 == -5.0
    assert not is_positive_semi_definite_3x3(2.0, 2.0, 2.0, 3.0, 0.0, 0.0)
