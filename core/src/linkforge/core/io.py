"""Functional I/O interface for LinkForge.

This module provides high-level, Pandas-style functional entry points for
reading and writing robot models in various formats (URDF, XACRO, SRDF).
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .models.robot import Robot
    from .models.srdf import SemanticRobotDescription
    from .validation import ValidationResult


def read_urdf(path_or_xml: str | Path) -> Robot:
    """Read a URDF from a file path or XML string.

    Args:
        path_or_xml: Path to .urdf file or raw XML string.

    Returns:
        A populated Robot model.
    """
    from .parsers import URDFParser

    parser = URDFParser()
    if os.path.exists(path_or_xml):
        return parser.parse(Path(path_or_xml))
    return parser.parse_string(str(path_or_xml))


def write_urdf(robot: Robot, path: str | Path) -> None:
    """Write a Robot model to a URDF file.

    Args:
        robot: The Robot model to export.
        path: Destination file path.
    """
    from .generators import URDFGenerator

    URDFGenerator().write(robot, Path(path))


def read_xacro(path: str | Path, **mappings: Any) -> Robot:
    """Parse and resolve a XACRO file into a Robot model.

    Args:
        path: Path to the .xacro file.
        **mappings: $(arg) substitutions for XACRO resolution.

    Returns:
        A populated Robot model.
    """
    from .parsers import URDFParser, XACROParser

    xml = XACROParser().resolve(Path(path), **mappings)
    return URDFParser().parse_string(xml)


def write_xacro(robot: Robot, path: str | Path) -> None:
    """Write a Robot model to a XACRO file.

    Args:
        robot: The Robot model to export.
        path: Destination file path.
    """
    from .generators import XACROGenerator

    XACROGenerator().write(robot, Path(path))


def read_srdf(path_or_xml: str | Path, robot: Robot | None = None) -> SemanticRobotDescription:
    """Read an SRDF from a file path or XML string.

    Args:
        path_or_xml: Path to .srdf file or raw XML string.
        robot: Optional robot model for cross-reference validation.

    Returns:
        A SemanticRobotDescription model.
    """
    from .parsers import SRDFParser

    parser = SRDFParser()
    if os.path.exists(path_or_xml):
        return parser.parse(Path(path_or_xml), robot=robot)
    return parser.parse_string(str(path_or_xml), robot=robot)


def write_srdf(robot_or_srdf: Robot | SemanticRobotDescription, path: str | Path) -> None:
    """Write semantic robot description to an SRDF file.

    Args:
        robot_or_srdf: Either a full Robot model or just the semantic description.
        path: Destination file path.
    """
    from .generators import SRDFGenerator
    from .models.robot import Robot
    from .models.srdf import SemanticRobotDescription

    if isinstance(robot_or_srdf, SemanticRobotDescription):
        # Wrap in a dummy robot for the generator
        robot = Robot(name=robot_or_srdf.robot_name or "robot")
        robot.semantic = robot_or_srdf
    else:
        robot = robot_or_srdf

    SRDFGenerator().write(robot, Path(path))


def validate_robot(robot: Robot) -> ValidationResult:
    """Perform full multi-phase validation on a Robot model."""
    from .validation import RobotValidator

    return RobotValidator().validate(robot)
