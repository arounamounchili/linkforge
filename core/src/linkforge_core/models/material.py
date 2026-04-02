"""Material and color definitions for robot visuals."""

from __future__ import annotations

from dataclasses import dataclass

from ..exceptions import RobotValidationError


@dataclass(frozen=True)
class Color:
    """RGBA color representation."""

    r: float  # Red (0.0 - 1.0)
    g: float  # Green (0.0 - 1.0)
    b: float  # Blue (0.0 - 1.0)
    a: float = 1.0  # Alpha (0.0 - 1.0)

    def __post_init__(self) -> None:
        """Validate color values."""
        for component in (self.r, self.g, self.b, self.a):
            if not 0.0 <= component <= 1.0:
                raise RobotValidationError(
                    "ColorComponent", component, "must be in range [0.0, 1.0]"
                )

    def to_tuple(self) -> tuple[float, float, float, float]:
        """Convert to RGBA tuple."""
        return (self.r, self.g, self.b, self.a)

    def __str__(self) -> str:
        """String representation for URDF."""
        return f"{self.r} {self.g} {self.b} {self.a}"


@dataclass(frozen=True)
class Material:
    """Material properties for visual elements."""

    name: str
    color: Color | None = None
    texture: str | None = None  # Path to texture file

    def __post_init__(self) -> None:
        """Validate material has at least color or texture."""
        if self.color is None and self.texture is None:
            raise RobotValidationError(
                "MaterialDefinition", self.name, "must have either color or texture"
            )
