"""LinkForge Core Library.

This is the Blender-independent core library for robot URDF/XACRO generation.
It can be used standalone without Blender for testing and CLI tools.

Modules:
    models: Data structures for robots, links, joints, geometry
    physics: Inertia and mass calculations
    parsers: URDF and XACRO file parsing
    generators: URDF and XACRO file generation
"""

from __future__ import annotations

__version__ = "1.2.0"

from . import generators, models, parsers, physics
from .base import LinkForgeError, RobotGeneratorError, RobotParserError
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
]
