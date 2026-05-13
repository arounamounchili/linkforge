"""LinkForge Core Library.

This is a multi-platform core library for robot URDF/XACRO generation.
It can be used standalone or integrated into various platforms (Blender, Unity, Web, etc.).

Modules:
    models: Data structures for robots, links, joints, geometry, and SRDF
    physics: Inertia and mass calculations
    parsers: URDF, XACRO, and SRDF file parsing
    generators: URDF, XACRO, and SRDF file generation
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
