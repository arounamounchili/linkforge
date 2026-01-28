"""Mathematical utility functions."""

from __future__ import annotations


def clean_float(value: float, epsilon: float = 1e-10) -> float:
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
    import math

    magnitude = math.sqrt(x**2 + y**2 + z**2)
    if magnitude < 1e-10:
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
        Space-separated string (e.g., \"1.0 2.0 3.0\")
    """
    return f"{format_float(x, precision)} {format_float(y, precision)} {format_float(z, precision)}"
