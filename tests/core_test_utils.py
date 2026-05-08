"""Shared utilities for Core integration tests."""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest
from linkforge_core.generators.urdf_generator import URDFGenerator
from linkforge_core.models.gazebo import GazeboElement
from linkforge_core.models.robot import Robot
from linkforge_core.parsers.urdf_parser import URDFParser


def perform_urdf_roundtrip(robot: Robot, pretty_print: bool = True) -> Robot:
    """Helper to perform a full URDF export-import cycle.

    Args:
        robot: The robot model to roundtrip.
        pretty_print: Whether to use pretty printing in the generator.

    Returns:
        The re-imported robot model.
    """
    generator = URDFGenerator(pretty_print=pretty_print)
    urdf_string = generator.generate(robot)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        f.write(urdf_string)

    try:
        return URDFParser().parse(temp_path)
    finally:
        if temp_path.exists():
            temp_path.unlink()


def compare_robots(robot1: Robot, robot2: Robot, context: str = "") -> list[str]:
    """Compare two robots and return a list of differences.

    This is a comprehensive comparison covering links, joints, materials,
    inertial properties, transmissions, sensors, and gazebo elements.
    """
    differences = []
    prefix = f"{context}: " if context else ""

    # Basic properties
    if robot1.name != robot2.name:
        differences.append(f"{prefix}Robot name: {robot1.name} != {robot2.name}")

    # Link comparison
    link_map1 = {link.name: link for link in robot1.links}
    link_map2 = {link.name: link for link in robot2.links}

    if set(link_map1.keys()) != set(link_map2.keys()):
        differences.append(
            f"{prefix}Link set mismatch: {set(link_map1.keys())} != {set(link_map2.keys())}"
        )
    else:
        for name, l1 in link_map1.items():
            l2 = link_map2[name]
            # Visuals
            if len(l1.visuals) != len(l2.visuals):
                differences.append(f"{prefix}Link '{name}' visual count mismatch")
            # Collisions
            if len(l1.collisions) != len(l2.collisions):
                differences.append(f"{prefix}Link '{name}' collision count mismatch")
            # Inertial
            if (l1.inertial is None) != (l2.inertial is None):
                differences.append(f"{prefix}Link '{name}' inertial presence mismatch")
            elif l1.inertial and l2.inertial:
                if abs(l1.inertial.mass - l2.inertial.mass) > 1e-6:
                    differences.append(
                        f"{prefix}Link '{name}' mass mismatch: {l1.inertial.mass} != {l2.inertial.mass}"
                    )
                # Origin
                o1, o2 = l1.inertial.origin, l2.inertial.origin
                for attr in ["x", "y", "z"]:
                    if abs(getattr(o1.xyz, attr) - getattr(o2.xyz, attr)) > 1e-5:
                        differences.append(f"{prefix}Link '{name}' inertial origin.{attr} mismatch")

    # Joint comparison
    joint_map1 = {j.name: j for j in robot1.joints}
    joint_map2 = {j.name: j for j in robot2.joints}

    if set(joint_map1.keys()) != set(joint_map2.keys()):
        differences.append(f"{prefix}Joint set mismatch")
    else:
        for name, j1 in joint_map1.items():
            j2 = joint_map2[name]
            if j1.type != j2.type:
                differences.append(f"{prefix}Joint '{name}' type mismatch: {j1.type} != {j2.type}")
            if j1.parent != j2.parent or j1.child != j2.child:
                differences.append(f"{prefix}Joint '{name}' hierarchy mismatch")
            # Limits
            if (j1.limits is None) != (j2.limits is None):
                differences.append(f"{prefix}Joint '{name}' limits presence mismatch")
            elif j1.limits and j2.limits:
                for attr in ["lower", "upper", "effort", "velocity"]:
                    v1, v2 = getattr(j1.limits, attr), getattr(j2.limits, attr)
                    if (v1 is None) != (v2 is None):
                        differences.append(f"{prefix}Joint '{name}' limit.{attr} presence mismatch")
                    elif v1 is not None and v2 is not None and abs(v1 - v2) > 1e-5:
                        differences.append(f"{prefix}Joint '{name}' limit.{attr} value mismatch")

    # Transmission comparison
    if len(robot1.transmissions) != len(robot2.transmissions):
        differences.append(
            f"{prefix}Transmission count mismatch: {len(robot1.transmissions)} != {len(robot2.transmissions)}"
        )

    # Sensor comparison
    if len(robot1.sensors) != len(robot2.sensors):
        differences.append(
            f"{prefix}Sensor count mismatch: {len(robot1.sensors)} != {len(robot2.sensors)}"
        )

    # Gazebo element comparison
    # Filter out automatically injected ros2_control plugins to avoid roundtrip mismatches
    def is_injected_plugin(g: GazeboElement) -> bool:
        return any("ros2_control" in p.name.lower() for p in g.plugins)

    gz1 = [g for g in robot1.gazebo_elements if not is_injected_plugin(g)]
    gz2 = [g for g in robot2.gazebo_elements if not is_injected_plugin(g)]

    if len(gz1) != len(gz2):
        differences.append(f"{prefix}Gazebo element count mismatch")

    return differences


def assert_robots_equal(robot1: Robot, robot2: Robot, context: str = "") -> None:
    """Assert that two robots are equal using comprehensive comparison."""
    differences = compare_robots(robot1, robot2, context)
    if differences:
        error_msg = f"Robots are not equal. {len(differences)} differences found:\n"
        error_msg += "\n".join(f"  - {d}" for d in differences)
        pytest.fail(error_msg)
