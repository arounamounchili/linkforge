"""Material and color definitions for visual robot components.

Defines the appearance of robot links, including RGBA colors and
texture references.

Core Components:
    - Material: Container for color and texture associations.
    - Color: RGBA spatial color representation [0, 1].
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from ..constants import (
    DEFAULT_MATERIAL_RGBA,
)
from ..exceptions import RobotValidationError, ValidationErrorCode


@dataclass(frozen=True)
class Color:
    """RGBA color representation with components in range [0.0, 1.0]."""

    r: float  # Red (0.0 - 1.0)
    g: float  # Green (0.0 - 1.0)
    b: float  # Blue (0.0 - 1.0)
    a: float = 1.0  # Alpha (0.0 - 1.0)

    def __post_init__(self) -> None:
        """Validate color values."""
        for component in (self.r, self.g, self.b, self.a):
            if not 0.0 <= component <= 1.0:
                raise RobotValidationError(
                    ValidationErrorCode.OUT_OF_RANGE,
                    "Color component must be in range [0.0, 1.0]",
                    target="ColorComponent",
                    value=component,
                )

    @classmethod
    def white(cls) -> Color:
        """Standard white color (1.0, 1.0, 1.0, 1.0)."""
        return cls(1.0, 1.0, 1.0, 1.0)

    @classmethod
    def black(cls) -> Color:
        """Standard black color (0.0, 0.0, 0.0, 1.0)."""
        return cls(0.0, 0.0, 0.0, 1.0)

    @classmethod
    def grey(cls) -> Color:
        """Standard grey color from constants."""
        return cls(*DEFAULT_MATERIAL_RGBA)

    def to_tuple(self) -> tuple[float, float, float, float]:
        """Convert to RGBA tuple."""
        return (self.r, self.g, self.b, self.a)

    def __str__(self) -> str:
        """String representation formatted as 'R G B A'."""
        return f"{self.r} {self.g} {self.b} {self.a}"


@dataclass(frozen=True)
class Material:
    """Material properties defining the visual surface of a robot link."""

    name: str
    color: Color | None = None
    texture: str | None = None  # Path to texture file

    def __post_init__(self) -> None:
        """Validate material has at least color or texture."""
        if self.color is None and self.texture is None:
            raise RobotValidationError(
                ValidationErrorCode.VALUE_EMPTY,
                f"Material '{self.name}' must have either color or texture",
                target="MaterialDefinition",
                value=self.name,
            )

    def with_prefix(self, prefix: str) -> Material:
        """Create a new material with a prefixed name."""
        return replace(self, name=f"{prefix}{self.name}")
