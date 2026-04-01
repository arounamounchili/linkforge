"""String utility functions."""

from __future__ import annotations

from ..exceptions import RobotValidationError


def sanitize_name(name: str, allow_hyphen: bool = True) -> str:
    """Sanitize a name for URDF and Python identifier compatibility.

    Replaces invalid characters with underscores and ensures it doesn't
    start with a digit.

    Args:
        name: Original name
        allow_hyphen: Whether to allow hyphens (valid in URDF, invalid in Python)

    Returns:
        Sanitized name
    """
    if not name:
        return ""

    # Prevent ReDoS: limit input length before processing
    if len(name) > 1000:
        raise RobotValidationError("NameLength", f"Length {len(name)} exceeds 1000")

    # Replace spaces with underscores
    name = name.replace(" ", "_")

    # Character iteration (safer than regex)
    allowed_special = ("_", "-") if allow_hyphen else ("_",)
    sanitized = "".join(c if c.isalnum() or c in allowed_special else "_" for c in name)

    # Ensure it doesn't start with a digit
    if sanitized and sanitized[0].isdigit():
        sanitized = f"_{sanitized}"

    return sanitized


def is_valid_urdf_name(name: str, allow_hyphen: bool = True) -> bool:
    """Check if a name is valid for URDF without modification.

    A valid URDF name:
    - Is not empty
    - Does not start with a digit
    - Contains only alphanumeric characters, underscores, and optionally hyphens

    Args:
        name: Name to validate
        allow_hyphen: Whether to allow hyphens (valid in URDF, invalid in Python)

    Returns:
        True if name is valid, False otherwise

    Examples:
        >>> is_valid_urdf_name("base_link")
        True
        >>> is_valid_urdf_name("base-link")
        True
        >>> is_valid_urdf_name("base-link", allow_hyphen=False)
        False
        >>> is_valid_urdf_name("2nd_link")
        False
        >>> is_valid_urdf_name("base link")
        False
    """
    if not name:
        return False

    # Must not start with a digit
    if name[0].isdigit():
        return False

    # All characters must be alphanumeric or allowed special chars
    allowed_special = ("_", "-") if allow_hyphen else ("_",)
    return all(c.isalnum() or c in allowed_special for c in name)
