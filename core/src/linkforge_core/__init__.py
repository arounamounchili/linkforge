"""LinkForge Core Library.

This is a multi-platform core library for robot URDF/XACRO generation.
It can be used standalone or integrated into various platforms (Blender, Unity, Web, etc.).

Modules:
    models: Data structures for robots, links, joints, geometry
    physics: Inertia and mass calculations
    parsers: URDF and XACRO file parsing
    generators: URDF and XACRO file generation
"""

from __future__ import annotations

__version__ = "1.2.2"  # x-release-please-version

from . import generators, models, parsers, physics
from .base import LinkForgeError, RobotGeneratorError, RobotParserError, XacroDetectedError
from .generators import URDFGenerator, XACROGenerator
from .generators.urdf_generator import format_float, format_vector

__all__ = [
    "models",
    "physics",
    "parsers",
    "URDFGenerator",
    "XACROGenerator",
    "format_float",
    "format_vector",
    "LinkForgeError",
    "RobotGeneratorError",
    "RobotParserError",
    "XacroDetectedError",
]
