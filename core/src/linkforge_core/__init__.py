"""LinkForge Core Library.

The platform-independent heart of the LinkForge project, providing a
unified Intermediate Representation (IR) for robotics. This core library
handles the "Robotics Intelligence" isolated from design tools.

Modules:
    composer: High-level API for assembling robots via RobotBuilder.
    generators: URDF, XACRO, and SRDF file generation.
    models: Core data structures for robots, links, joints, and geometry.
    parsers: Lossless URDF, XACRO, and SRDF file parsing.
    physics: Scientifically grounded inertia and mass calculations.
    validation: Multi-phase hardened validation of robot descriptions.
"""

from __future__ import annotations

__version__ = "1.3.0"  # x-release-please-version

from . import composer, generators, models, parsers, physics, validation
from .composer import RobotBuilder
from .exceptions import (
    LinkForgeError,
    RobotGeneratorError,
    RobotModelError,
    RobotParserError,
    XacroDetectedError,
)
from .generators import URDFGenerator, XACROGenerator
from .utils.math_utils import format_float, format_vector
from .validation import RobotValidator

__all__ = [
    "models",
    "physics",
    "parsers",
    "composer",
    "validation",
    "RobotBuilder",
    "RobotValidator",
    "URDFGenerator",
    "XACROGenerator",
    "format_float",
    "format_vector",
    "LinkForgeError",
    "RobotGeneratorError",
    "RobotModelError",
    "RobotParserError",
    "XacroDetectedError",
]
