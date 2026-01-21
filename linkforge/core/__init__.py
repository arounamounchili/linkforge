"""LinkForge Core Library.

This is the Blender-independent core library for robot URDF/XACRO generation.
It can be used standalone without Blender for testing and CLI tools.

Modules:
    models: Data structures for robots, links, joints, geometry
    physics: Inertia and mass calculations
    parsers: URDF and XACRO file parsing
"""

from __future__ import annotations

from . import models, parsers, physics
from .urdf_generator import URDFGenerator, format_float, format_vector
from .xacro_generator import XACROGenerator

__all__ = [
    "models",
    "physics",
    "parsers",
    "URDFGenerator",
    "XACROGenerator",
    "format_float",
    "format_vector",
]
