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
