"""Mathematical utility functions."""

from __future__ import annotations

import math

from ..constants import (
    EPSILON,
    FLOAT_CLEAN_EPSILON,
    SCIENTIFIC_NOTATION_THRESHOLD,
    SYLVESTER_TOLERANCE_EPSILON,
)


def clean_float(value: float, epsilon: float = FLOAT_CLEAN_EPSILON) -> float:
    """Clean up floating point values to avoid -0.0 and very small numbers.

    Args:
        value: Float value to clean
        epsilon: Threshold below which values become 0.0

    Returns:
        Cleaned float value
    """
    if abs(value) < epsilon:
        return 0.0
    return value


def format_float(value: float, precision: int = 6) -> str:
    """Format float with reasonable precision, removing trailing zeros.

    Args:
        value: Float value to format
        precision: Maximum number of decimal places

    Returns:
        Formatted string
    """
    # Clean up small values and -0.0 first
    cleaned = clean_float(value)
    if cleaned == 0.0:
        return "0"

    # Use scientific notation for very small numbers to avoid zeroing them out
    if abs(cleaned) < SCIENTIFIC_NOTATION_THRESHOLD:
        formatted = f"{cleaned:.{precision}e}"
        mantissa, exp = formatted.split("e")
        mantissa = mantissa.rstrip("0").rstrip(".")
        return f"{mantissa}e{exp}"

    # Format with specified precision
    formatted = f"{cleaned:.{precision}f}"
    # Remove trailing zeros and decimal point if not needed
    formatted = formatted.rstrip("0").rstrip(".")
    return formatted if formatted != "-0" else "0"


def normalize_vector(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Normalize a 3D vector to unit length.

    Args:
        x, y, z: Vector components

    Returns:
        Normalized components (x, y, z)
    """
    magnitude = math.sqrt(x**2 + y**2 + z**2)
    if magnitude < EPSILON:
        return (0.0, 0.0, 0.0)
    return (x / magnitude, y / magnitude, z / magnitude)


def format_vector(x: float, y: float, z: float, precision: int = 6) -> str:
    """Format 3D vector with reasonable precision.

    Converts three float components into a space-separated string suitable
    for URDF attributes like xyz, rpy, size, etc.

    Args:
        x, y, z: Vector components
        precision: Floating point precision

    Returns:
        Space-separated string (e.g., "1.0 2.0 3.0")
    """
    return f"{format_float(x, precision)} {format_float(y, precision)} {format_float(z, precision)}"


def symmetric_matrix_eigenvalues_3x3(
    a: float,
    b: float,
    c: float,
    d: float = 0.0,
    e: float = 0.0,
    f: float = 0.0,
) -> tuple[float, float, float]:
    """Compute the three real eigenvalues of a 3x3 real symmetric matrix analytically.

    Uses Oliver K. Smith's closed-form trigonometric algorithm (1961) for real
    symmetric 3x3 matrices:
        A = [[a, d, e],
             [d, b, f],
             [e, f, c]]

    Args:
        a: A[0, 0] component (e.g. Ixx)
        b: A[1, 1] component (e.g. Iyy)
        c: A[2, 2] component (e.g. Izz)
        d: A[0, 1] / A[1, 0] component (e.g. Ixy)
        e: A[0, 2] / A[2, 0] component (e.g. Ixz)
        f: A[1, 2] / A[2, 1] component (e.g. Iyz)

    Returns:
        Tuple of three eigenvalues (lambda1, lambda2, lambda3) sorted in descending order.
    """
    q = (a + b + c) / 3.0
    p1 = d**2 + e**2 + f**2
    if p1 < 1e-15:
        # Matrix is diagonal
        eigs = sorted([a, b, c], reverse=True)
        return (eigs[0], eigs[1], eigs[2])

    p2 = (a - q) ** 2 + (b - q) ** 2 + (c - q) ** 2 + 2.0 * p1
    p = math.sqrt(p2 / 6.0)

    # Normalized shifted matrix B = (A - q*I)/p
    b00, b11, b22 = (a - q) / p, (b - q) / p, (c - q) / p
    b01, b02, b12 = d / p, e / p, f / p

    det_b = (
        b00 * (b11 * b22 - b12**2) - b01 * (b01 * b22 - b02 * b12) + b02 * (b01 * b12 - b11 * b02)
    )
    r = max(-1.0, min(1.0, det_b / 2.0))
    phi = math.acos(r) / 3.0

    eig1 = q + 2.0 * p * math.cos(phi)
    eig3 = q + 2.0 * p * math.cos(phi + 2.0 * math.pi / 3.0)
    eig2 = 3.0 * q - eig1 - eig3

    eigs = sorted([eig1, eig2, eig3], reverse=True)
    return (eigs[0], eigs[1], eigs[2])


def sylvester_minors_3x3(
    a: float,
    b: float,
    c: float,
    d: float = 0.0,
    e: float = 0.0,
    f: float = 0.0,
) -> tuple[float, float, float]:
    """Compute the leading principal minors (Delta1, Delta2, Delta3) of a 3x3 symmetric matrix.

        A = [[a, d, e],
             [d, b, f],
             [e, f, c]]

    Args:
        a, b, c: Diagonal elements (e.g. Ixx, Iyy, Izz)
        d, e, f: Off-diagonal elements (e.g. Ixy, Ixz, Iyz)

    Returns:
        Tuple of leading principal minors (Delta1, Delta2, Delta3).
    """
    delta1 = a
    delta2 = a * b - d**2
    delta3 = a * (b * c - f**2) - d * (d * c - e * f) + e * (d * f - b * e)
    return (delta1, delta2, delta3)


def is_positive_semi_definite_3x3(
    a: float,
    b: float,
    c: float,
    d: float = 0.0,
    e: float = 0.0,
    f: float = 0.0,
    eps: float = SYLVESTER_TOLERANCE_EPSILON,
) -> bool:
    """Check if a 3x3 symmetric matrix is positive semi-definite using Sylvester's criterion.

    Args:
        a, b, c: Diagonal elements (Ixx, Iyy, Izz)
        d, e, f: Off-diagonal elements (Ixy, Ixz, Iyz)
        eps: Numerical tolerance factor

    Returns:
        True if all leading principal minors are non-negative within tolerance.
    """
    scale = max(abs(a), abs(b), abs(c), abs(d), abs(e), abs(f), 1.0)
    delta1, delta2, delta3 = sylvester_minors_3x3(a, b, c, d, e, f)
    tol1 = eps * scale
    tol2 = eps * (scale**2)
    tol3 = eps * (scale**3)
    return delta1 >= -tol1 and delta2 >= -tol2 and delta3 >= -tol3
