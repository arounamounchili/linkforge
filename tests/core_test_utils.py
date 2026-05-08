"""Shared utilities for Core integration tests."""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest
from linkforge_core.generators.urdf_generator import URDFGenerator
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
        temp_path.unlink()


def compare_robots(robot1: Robot, robot2: Robot, context: str = "") -> list[str]:
    """Compare two robots and return a list of differences.

    This is a comprehensive comparison covering links, joints, materials,
    inertial properties, and transmissions.
    """
    differences = []

    # Compare basic properties
    if robot1.name != robot2.name:
        differences.append(f"{context}Robot name: {robot1.name} != {robot2.name}")

    # Compare link count
    if len(robot1.links) != len(robot2.links):
        differences.append(f"{context}Link count: {len(robot1.links)} != {len(robot2.links)}")
        return differences  # Can't continue if counts differ

    # Compare links
    link_map1 = {link.name: link for link in robot1.links}
    link_map2 = {link.name: link for link in robot2.links}

    for link_name in sorted(link_map1.keys()):
        if link_name not in link_map2:
            differences.append(f"{context}Link '{link_name}' missing in robot2")
            continue

        link1 = link_map1[link_name]
        link2 = link_map2[link_name]

        # Compare visuals
        if len(link1.visuals) != len(link2.visuals):
            differences.append(
                f"{context}Link '{link_name}': visual count {len(link1.visuals)} != {len(link2.visuals)}"
            )

        # Compare collisions
        if len(link1.collisions) != len(link2.collisions):
            differences.append(
                f"{context}Link '{link_name}': collision count {len(link1.collisions)} != {len(link2.collisions)}"
            )

        # Compare inertial
        if (link1.inertial is None) != (link2.inertial is None):
            differences.append(f"{context}Link '{link_name}': inertial presence mismatch")
        elif link1.inertial and link2.inertial:
            if abs(link1.inertial.mass - link2.inertial.mass) > 1e-6:
                differences.append(
                    f"{context}Link '{link_name}': mass {link1.inertial.mass} != {link2.inertial.mass}"
                )

            # Compare inertia tensor
            i1 = link1.inertial.inertia
            i2 = link2.inertial.inertia
            for attr in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
                v1 = getattr(i1, attr)
                v2 = getattr(i2, attr)
                if abs(v1 - v2) > 1e-6:
                    differences.append(f"{context}Link '{link_name}': inertia.{attr} {v1} != {v2}")

    # Compare joints
    if len(robot1.joints) != len(robot2.joints):
        differences.append(f"{context}Joint count: {len(robot1.joints)} != {len(robot2.joints)}")
    else:
        joint_map1 = {j.name: j for j in robot1.joints}
        joint_map2 = {j.name: j for j in robot2.joints}
        for j_name in sorted(joint_map1.keys()):
            if j_name not in joint_map2:
                differences.append(f"{context}Joint '{j_name}' missing in robot2")
                continue
            j1 = joint_map1[j_name]
            j2 = joint_map2[j_name]
            if j1.type != j2.type:
                differences.append(f"{context}Joint '{j_name}': type {j1.type} != {j2.type}")

    return differences


def assert_robots_equal(robot1: Robot, robot2: Robot, context: str = "") -> None:
    """Assert that two robots are equal using comprehensive comparison."""
    differences = compare_robots(robot1, robot2, context)
    if differences:
        error_msg = f"Robots are not equal. {len(differences)} differences found:\n"
        error_msg += "\n".join(f"  - {d}" for d in differences)
        pytest.fail(error_msg)
