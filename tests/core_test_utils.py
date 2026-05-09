"""Shared utilities for Core integration tests."""

from __future__ import annotations

import tempfile
from pathlib import Path
from typing import Any

from linkforge_core.generators.urdf_generator import URDFGenerator
from linkforge_core.models.robot import Robot
from linkforge_core.parsers.urdf_parser import URDFParser


def perform_urdf_roundtrip(robot: Robot, pretty_print: bool = True, **kwargs: Any) -> Robot:
    """Helper to perform a full URDF export-import cycle.

    Args:
        robot: The robot model to roundtrip.
        pretty_print: Whether to use pretty printing in the generator.
        **kwargs: Additional options for URDFGenerator (e.g., use_ros2_control=False)

    Returns:
        The re-imported robot model.
    """
    generator = URDFGenerator(pretty_print=pretty_print, **kwargs)
    urdf_string = generator.generate(robot)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        f.write(urdf_string)

    try:
        return URDFParser().parse(temp_path)
    finally:
        if temp_path.exists():
            temp_path.unlink()


def assert_robots_equal(robot1: Robot, robot2: Robot, context: str = "") -> None:
    """Assert that two robots are equal using comprehensive comparison."""
    # We use direct equality on normalized models to leverage pytest's built-in diffing
    # while remaining order-independent for component lists.
    assert robot1.normalized() == robot2.normalized(), (
        f"{context}: Robots are not structurally equal"
    )
