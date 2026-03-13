"""Perfect roundtrip test - Verify exact preservation of all URDF elements.

This test performs a deep comparison between original and round-tripped URDFs
to identify any data loss or transformation issues.
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest
from linkforge_core import URDFGenerator
from linkforge_core.models import (
    Box,
    Collision,
    Color,
    Cylinder,
    Joint,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointType,
    Link,
    Material,
    Robot,
    Sphere,
    Transform,
    Vector3,
    Visual,
)
from linkforge_core.parsers.urdf_parser import URDFParser


def compare_robots(robot1: Robot, robot2: Robot, context: str = "") -> list[str]:
    """Compare two robots and return list of differences."""
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

            # Compare inertial origin
            if (link1.inertial.origin is None) != (link2.inertial.origin is None):
                differences.append(
                    f"{context}Link '{link_name}': inertial origin presence mismatch"
                )
            elif link1.inertial.origin and link2.inertial.origin:
                o1 = link1.inertial.origin
                o2 = link2.inertial.origin
                for coord in ["x", "y", "z"]:
                    v1 = getattr(o1.xyz, coord)
                    v2 = getattr(o2.xyz, coord)
                    if abs(v1 - v2) > 1e-6:
                        differences.append(
                            f"{context}Link '{link_name}': inertial origin.xyz.{coord} {v1} != {v2}"
                        )
                for angle in ["x", "y", "z"]:
                    v1 = getattr(o1.rpy, angle)
                    v2 = getattr(o2.rpy, angle)
                    if abs(v1 - v2) > 1e-6:
                        differences.append(
                            f"{context}Link '{link_name}': inertial origin.rpy.{angle} {v1} != {v2}"
                        )

        # Compare visual origins and materials
        for i, (vis1, vis2) in enumerate(zip(link1.visuals, link2.visuals, strict=False)):
            if (vis1.origin is None) != (vis2.origin is None):
                differences.append(
                    f"{context}Link '{link_name}' visual {i}: origin presence mismatch"
                )
            elif vis1.origin and vis2.origin:
                for coord in ["x", "y", "z"]:
                    v1 = getattr(vis1.origin.xyz, coord)
                    v2 = getattr(vis2.origin.xyz, coord)
                    if abs(v1 - v2) > 1e-6:
                        differences.append(
                            f"{context}Link '{link_name}' visual {i}: origin.xyz.{coord} {v1} != {v2}"
                        )

            # Compare material
            if (vis1.material is None) != (vis2.material is None):
                differences.append(
                    f"{context}Link '{link_name}' visual {i}: material presence mismatch"
                )
            elif vis1.material and vis2.material:
                if vis1.material.name != vis2.material.name:
                    differences.append(
                        f"{context}Link '{link_name}' visual {i}: material name '{vis1.material.name}' != '{vis2.material.name}'"
                    )
                if vis1.material.color and vis2.material.color:
                    for channel in ["r", "g", "b", "a"]:
                        v1 = getattr(vis1.material.color, channel)
                        v2 = getattr(vis2.material.color, channel)
                        if abs(v1 - v2) > 1e-6:
                            differences.append(
                                f"{context}Link '{link_name}' visual {i}: material.color.{channel} {v1} != {v2}"
                            )

    # Compare joints
    if len(robot1.joints) != len(robot2.joints):
        differences.append(f"{context}Joint count: {len(robot1.joints)} != {len(robot2.joints)}")
        return differences

    joint_map1 = {joint.name: joint for joint in robot1.joints}
    joint_map2 = {joint.name: joint for joint in robot2.joints}

    for joint_name in sorted(joint_map1.keys()):
        if joint_name not in joint_map2:
            differences.append(f"{context}Joint '{joint_name}' missing in robot2")
            continue

        joint1 = joint_map1[joint_name]
        joint2 = joint_map2[joint_name]

        # Compare type
        if joint1.type != joint2.type:
            differences.append(
                f"{context}Joint '{joint_name}': type {joint1.type} != {joint2.type}"
            )

        # Compare parent/child
        if joint1.parent != joint2.parent:
            differences.append(
                f"{context}Joint '{joint_name}': parent '{joint1.parent}' != '{joint2.parent}'"
            )
        if joint1.child != joint2.child:
            differences.append(
                f"{context}Joint '{joint_name}': child '{joint1.child}' != '{joint2.child}'"
            )

        # Compare origin
        if (joint1.origin is None) != (joint2.origin is None):
            differences.append(f"{context}Joint '{joint_name}': origin presence mismatch")
        elif joint1.origin and joint2.origin:
            for coord in ["x", "y", "z"]:
                v1 = getattr(joint1.origin.xyz, coord)
                v2 = getattr(joint2.origin.xyz, coord)
                if abs(v1 - v2) > 1e-6:
                    differences.append(
                        f"{context}Joint '{joint_name}': origin.xyz.{coord} {v1} != {v2}"
                    )
            for angle in ["x", "y", "z"]:
                v1 = getattr(joint1.origin.rpy, angle)
                v2 = getattr(joint2.origin.rpy, angle)
                if abs(v1 - v2) > 1e-6:
                    differences.append(
                        f"{context}Joint '{joint_name}': origin.rpy.{angle} {v1} != {v2}"
                    )

        # Compare axis (only for joints that use it)
        if joint1.type in (JointType.REVOLUTE, JointType.CONTINUOUS, JointType.PRISMATIC):
            if (joint1.axis is None) != (joint2.axis is None):
                differences.append(f"{context}Joint '{joint_name}': axis presence mismatch")
            elif joint1.axis and joint2.axis:
                for coord in ["x", "y", "z"]:
                    v1 = getattr(joint1.axis, coord)
                    v2 = getattr(joint2.axis, coord)
                    if abs(v1 - v2) > 1e-6:
                        differences.append(
                            f"{context}Joint '{joint_name}': axis.{coord} {v1} != {v2}"
                        )

        # Compare limits
        if (joint1.limits is None) != (joint2.limits is None):
            differences.append(f"{context}Joint '{joint_name}': limits presence mismatch")
        elif joint1.limits and joint2.limits:
            for attr in ["lower", "upper", "effort", "velocity"]:
                v1 = getattr(joint1.limits, attr)
                v2 = getattr(joint2.limits, attr)
                if abs(v1 - v2) > 1e-6:
                    differences.append(f"{context}Joint '{joint_name}': limits.{attr} {v1} != {v2}")

        # Compare dynamics
        if (joint1.dynamics is None) != (joint2.dynamics is None):
            differences.append(f"{context}Joint '{joint_name}': dynamics presence mismatch")
        elif joint1.dynamics and joint2.dynamics:
            if abs(joint1.dynamics.damping - joint2.dynamics.damping) > 1e-6:
                differences.append(
                    f"{context}Joint '{joint_name}': dynamics.damping {joint1.dynamics.damping} != {joint2.dynamics.damping}"
                )
            if abs(joint1.dynamics.friction - joint2.dynamics.friction) > 1e-6:
                differences.append(
                    f"{context}Joint '{joint_name}': dynamics.friction {joint1.dynamics.friction} != {joint2.dynamics.friction}"
                )

        # Compare mimic
        if (joint1.mimic is None) != (joint2.mimic is None):
            differences.append(f"{context}Joint '{joint_name}': mimic presence mismatch")
        elif joint1.mimic and joint2.mimic:
            if joint1.mimic.joint != joint2.mimic.joint:
                differences.append(
                    f"{context}Joint '{joint_name}': mimic.joint '{joint1.mimic.joint}' != '{joint2.mimic.joint}'"
                )
            if abs(joint1.mimic.multiplier - joint2.mimic.multiplier) > 1e-6:
                differences.append(
                    f"{context}Joint '{joint_name}': mimic.multiplier {joint1.mimic.multiplier} != {joint2.mimic.multiplier}"
                )
            if abs(joint1.mimic.offset - joint2.mimic.offset) > 1e-6:
                differences.append(
                    f"{context}Joint '{joint_name}': mimic.offset {joint1.mimic.offset} != {joint2.mimic.offset}"
                )

    # Compare transmissions
    if len(robot1.transmissions) != len(robot2.transmissions):
        differences.append(
            f"{context}Transmission count: {len(robot1.transmissions)} != {len(robot2.transmissions)}"
        )
    else:
        trans_map1 = {t.name: t for t in robot1.transmissions}
        trans_map2 = {t.name: t for t in robot2.transmissions}

        for trans_name in sorted(trans_map1.keys()):
            if trans_name not in trans_map2:
                differences.append(f"{context}Transmission '{trans_name}' missing in robot2")
                continue

            trans1 = trans_map1[trans_name]
            trans2 = trans_map2[trans_name]

            if trans1.type != trans2.type:
                differences.append(
                    f"{context}Transmission '{trans_name}': type '{trans1.type}' != '{trans2.type}'"
                )

            if len(trans1.joints) != len(trans2.joints):
                differences.append(
                    f"{context}Transmission '{trans_name}': joint count {len(trans1.joints)} != {len(trans2.joints)}"
                )

    # Compare sensors
    if len(robot1.sensors) != len(robot2.sensors):
        differences.append(f"{context}Sensor count: {len(robot1.sensors)} != {len(robot2.sensors)}")
    else:
        sensor_map1 = {s.name: s for s in robot1.sensors}
        sensor_map2 = {s.name: s for s in robot2.sensors}

        for sensor_name in sorted(sensor_map1.keys()):
            if sensor_name not in sensor_map2:
                differences.append(f"{context}Sensor '{sensor_name}' missing in robot2")
                continue

            sensor1 = sensor_map1[sensor_name]
            sensor2 = sensor_map2[sensor_name]

            if sensor1.type != sensor2.type:
                differences.append(
                    f"{context}Sensor '{sensor_name}': type '{sensor1.type}' != '{sensor2.type}'"
                )

            if sensor1.link_name != sensor2.link_name:
                differences.append(
                    f"{context}Sensor '{sensor_name}': link_name '{sensor1.link_name}' != '{sensor2.link_name}'"
                )

    # Compare Gazebo elements
    if len(robot1.gazebo_elements) != len(robot2.gazebo_elements):
        differences.append(
            f"{context}Gazebo element count: {len(robot1.gazebo_elements)} != {len(robot2.gazebo_elements)}"
        )

    return differences


def test_perfect_roundtrip_comprehensive_robot(examples_dir: Path) -> None:
    """Test that comprehensive test robot survives perfect roundtrip."""
    # Load original
    original_path = examples_dir / "urdf" / "roundtrip_test_robot.urdf"
    robot1 = URDFParser().parse(original_path)

    # Export
    generator = URDFGenerator(pretty_print=True)
    urdf_string = generator.generate(robot1)

    # Re-import
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        f.write(urdf_string)

    try:
        robot2 = URDFParser().parse(temp_path)

        # Compare
        differences = compare_robots(robot1, robot2)

        # Report differences
        if differences:
            print("\n=== ROUNDTRIP DIFFERENCES FOUND ===")
            for diff in differences:
                print(f"  - {diff}")
            print(f"\nTotal differences: {len(differences)}")

        # Assert no differences
        assert len(differences) == 0, f"Found {len(differences)} differences in roundtrip"

    finally:
        temp_path.unlink()


def test_geometry_types_roundtrip() -> None:
    """Test that all geometry types survive roundtrip."""
    robot = Robot(name="geometry_test")

    # Add base link to satisfy tree structure
    robot.add_link(Link(name="base"))

    # Box
    robot.add_link(
        Link(
            name="box_link",
            initial_visuals=[
                Visual(
                    geometry=Box(size=Vector3(1.0, 2.0, 3.0)),
                    origin=Transform(xyz=Vector3(0.1, 0.2, 0.3)),
                )
            ],
        )
    )
    robot.add_joint(
        Joint(
            name="base_to_box",
            type=JointType.FIXED,
            parent="base",
            child="box_link",
        )
    )

    # Cylinder
    robot.add_link(
        Link(
            name="cylinder_link",
            initial_visuals=[
                Visual(
                    geometry=Cylinder(radius=0.5, length=2.0),
                    origin=Transform(xyz=Vector3(0.0, 0.0, 1.0)),
                )
            ],
        )
    )
    robot.add_joint(
        Joint(
            name="base_to_cylinder",
            type=JointType.FIXED,
            parent="base",
            child="cylinder_link",
        )
    )

    # Sphere
    robot.add_link(
        Link(
            name="sphere_link",
            initial_visuals=[
                Visual(
                    geometry=Sphere(radius=0.75),
                    origin=Transform(xyz=Vector3(1.0, 1.0, 1.0)),
                )
            ],
        )
    )
    robot.add_joint(
        Joint(
            name="base_to_sphere",
            type=JointType.FIXED,
            parent="base",
            child="sphere_link",
        )
    )

    # Roundtrip
    generator = URDFGenerator()
    urdf_string = generator.generate(robot)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        f.write(urdf_string)

    try:
        robot2 = URDFParser().parse(temp_path)
        differences = compare_robots(robot, robot2)

        assert len(differences) == 0, f"Geometry roundtrip failed: {differences}"

    finally:
        temp_path.unlink()


def test_joint_types_roundtrip() -> None:
    """Test that all joint types survive roundtrip."""
    robot = Robot(name="joint_test")
    robot.add_link(Link(name="base"))
    robot.add_link(Link(name="revolute_link"))
    robot.add_link(Link(name="prismatic_link"))
    robot.add_link(Link(name="continuous_link"))
    robot.add_link(Link(name="fixed_link"))

    # Revolute
    robot.add_joint(
        Joint(
            name="revolute_joint",
            type=JointType.REVOLUTE,
            parent="base",
            child="revolute_link",
            origin=Transform(xyz=Vector3(1.0, 0.0, 0.0), rpy=Vector3(0.0, 0.0, 1.57)),
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=2.0),
            dynamics=JointDynamics(damping=0.5, friction=0.1),
        )
    )

    # Prismatic
    robot.add_joint(
        Joint(
            name="prismatic_joint",
            type=JointType.PRISMATIC,
            parent="base",
            child="prismatic_link",
            origin=Transform(xyz=Vector3(0.0, 1.0, 0.0)),
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=0.0, upper=1.0, effort=5.0, velocity=1.0),
        )
    )

    # Continuous
    robot.add_joint(
        Joint(
            name="continuous_joint",
            type=JointType.CONTINUOUS,
            parent="base",
            child="continuous_link",
            origin=Transform(xyz=Vector3(0.0, 0.0, 1.0)),
            axis=Vector3(1, 0, 0),
            dynamics=JointDynamics(damping=0.2, friction=0.05),
        )
    )

    # Fixed
    robot.add_joint(
        Joint(
            name="fixed_joint",
            type=JointType.FIXED,
            parent="base",
            child="fixed_link",
            origin=Transform(xyz=Vector3(-1.0, 0.0, 0.0)),
        )
    )

    # Roundtrip
    generator = URDFGenerator()
    urdf_string = generator.generate(robot)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        f.write(urdf_string)

    try:
        robot2 = URDFParser().parse(temp_path)
        differences = compare_robots(robot, robot2)

        assert len(differences) == 0, f"Joint types roundtrip failed: {differences}"

    finally:
        temp_path.unlink()


def test_mimic_joint_roundtrip() -> None:
    """Test that mimic joints survive roundtrip."""
    robot = Robot(name="mimic_test")
    robot.add_link(Link(name="base"))
    robot.add_link(Link(name="master_link"))
    robot.add_link(Link(name="follower_link"))

    robot.add_joint(
        Joint(
            name="master_joint",
            type=JointType.PRISMATIC,
            parent="base",
            child="master_link",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=0.0, upper=0.1, effort=1.0, velocity=0.5),
        )
    )

    robot.add_joint(
        Joint(
            name="follower_joint",
            type=JointType.PRISMATIC,
            parent="base",
            child="follower_link",
            axis=Vector3(0, 0, 1),
            limits=JointLimits(lower=-0.1, upper=0.0, effort=1.0, velocity=0.5),
            mimic=JointMimic(joint="master_joint", multiplier=-1.0, offset=0.0),
        )
    )

    # Roundtrip
    generator = URDFGenerator()
    urdf_string = generator.generate(robot)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        f.write(urdf_string)

    try:
        robot2 = URDFParser().parse(temp_path)
        differences = compare_robots(robot, robot2)

        assert len(differences) == 0, f"Mimic joint roundtrip failed: {differences}"

        # Specifically verify mimic
        follower = next(j for j in robot2.joints if j.name == "follower_joint")
        assert follower.mimic is not None
        assert follower.mimic.joint == "master_joint"
        assert follower.mimic.multiplier == -1.0
        assert follower.mimic.offset == 0.0

    finally:
        temp_path.unlink()


def test_material_preservation_roundtrip() -> None:
    """Test that materials with colors survive roundtrip."""
    robot = Robot(name="material_test")

    red_material = Material(name="red", color=Color(1.0, 0.0, 0.0, 1.0))
    blue_material = Material(name="blue", color=Color(0.0, 0.0, 1.0, 0.5))

    robot.add_link(Link(name="base"))

    robot.add_link(
        Link(
            name="red_link",
            initial_visuals=[Visual(geometry=Box(size=Vector3(1, 1, 1)), material=red_material)],
        )
    )
    robot.add_joint(
        Joint(
            name="base_to_red",
            type=JointType.FIXED,
            parent="base",
            child="red_link",
        )
    )

    robot.add_link(
        Link(
            name="blue_link",
            initial_visuals=[Visual(geometry=Sphere(radius=0.5), material=blue_material)],
        )
    )
    robot.add_joint(
        Joint(
            name="base_to_blue",
            type=JointType.FIXED,
            parent="base",
            child="blue_link",
        )
    )

    # Roundtrip
    generator = URDFGenerator()
    urdf_string = generator.generate(robot)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        f.write(urdf_string)

    try:
        robot2 = URDFParser().parse(temp_path)
        differences = compare_robots(robot, robot2)

        assert len(differences) == 0, f"Material roundtrip failed: {differences}"

    finally:
        temp_path.unlink()


def test_collision_geometry_roundtrip() -> None:
    """Test that collision geometries survive roundtrip."""
    robot = Robot(name="collision_test")

    robot.add_link(Link(name="base"))

    robot.add_link(
        Link(
            name="test_link",
            initial_visuals=[Visual(geometry=Box(size=Vector3(1, 1, 1)))],
            initial_collisions=[
                Collision(
                    geometry=Box(size=Vector3(1.1, 1.1, 1.1)),
                    origin=Transform(xyz=Vector3(0, 0, 0.05)),
                )
            ],
        )
    )
    robot.add_joint(
        Joint(
            name="base_to_test",
            type=JointType.FIXED,
            parent="base",
            child="test_link",
        )
    )

    # Roundtrip
    generator = URDFGenerator()
    urdf_string = generator.generate(robot)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        f.write(urdf_string)

    try:
        robot2 = URDFParser().parse(temp_path)
        differences = compare_robots(robot, robot2)

        assert len(differences) == 0, f"Collision roundtrip failed: {differences}"

    finally:
        temp_path.unlink()


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
