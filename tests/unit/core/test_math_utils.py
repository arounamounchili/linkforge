"""Tests for mathematical utility functions."""

from __future__ import annotations

from linkforge.core.utils.math_utils import clean_float, format_float, normalize_vector


def test_clean_float():
    """Test cleaning floating point values."""
    assert clean_float(1.0) == 1.0
    assert clean_float(1e-11) == 0.0
    assert clean_float(-1e-11) == 0.0
    assert clean_float(1e-9, epsilon=1e-8) == 0.0


def test_format_float():
    """Test formatting floating point values."""
    assert format_float(1.1234567) == "1.123457"
    assert format_float(1.0) == "1"
    assert format_float(1.100) == "1.1"
    assert format_float(1e-11) == "0"
    assert format_float(-0.0) == "0"


def test_normalize_vector():
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
