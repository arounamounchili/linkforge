"""Comprehensive roundtrip test - Import → Export → Re-import verification.

This test verifies that the complete workflow preserves all robot properties:
- Joint origins (position and rotation)
- Visual geometry origins
- Inertial properties
- Materials
- Joint limits and dynamics
- Transmissions and sensors
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest
from linkforge_core import URDFGenerator
from linkforge_core.parsers.urdf_parser import URDFParser


def test_comprehensive_roundtrip_preserves_structure(examples_dir: Path):
    """Test that export → re-import preserves robot structure perfectly."""
    # Step 1: Import original URDF
    original_path = examples_dir / "urdf" / "roundtrip_test_robot.urdf"
    robot1 = URDFParser().parse(original_path)

    # Step 2: Export to temporary file
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        generator = URDFGenerator()
        urdf_content = generator.generate(robot1)
        f.write(urdf_content)

    try:
        # Step 3: Re-import the exported URDF
        robot2 = URDFParser().parse(temp_path)

        # ========== VERIFY STRUCTURE ==========
        assert robot2.name == robot1.name
        assert len(robot2.links) == len(robot1.links)
        assert len(robot2.joints) == len(robot1.joints)
        assert len(robot2.transmissions) == len(robot1.transmissions)
        assert len(robot2.sensors) == len(robot1.sensors)

        # ========== VERIFY LINKS ==========
        link_map1 = {link.name: link for link in robot1.links}
        link_map2 = {link.name: link for link in robot2.links}

        assert set(link_map1.keys()) == set(link_map2.keys()), "Link names don't match"

        for link_name in link_map1:
            link1 = link_map1[link_name]
            link2 = link_map2[link_name]

            # Verify visual count
            assert len(link2.visuals) == len(link1.visuals), (
                f"Link {link_name}: visual count mismatch"
            )

            # Verify collision count
            assert len(link2.collisions) == len(link1.collisions), (
                f"Link {link_name}: collision count mismatch"
            )

            # Verify inertial properties
            if link1.inertial:
                assert link2.inertial is not None, f"Link {link_name}: missing inertial"
                assert abs(link2.inertial.mass - link1.inertial.mass) < 0.001, (
                    f"Link {link_name}: mass mismatch"
                )

                # Verify inertia tensor
                i1 = link1.inertial.inertia
                i2 = link2.inertial.inertia
                assert abs(i2.ixx - i1.ixx) < 0.0001, f"Link {link_name}: ixx mismatch"
                assert abs(i2.iyy - i1.iyy) < 0.0001, f"Link {link_name}: iyy mismatch"
                assert abs(i2.izz - i1.izz) < 0.0001, f"Link {link_name}: izz mismatch"

                # Verify COM origin
                if link1.inertial.origin:
                    o1 = link1.inertial.origin
                    o2 = link2.inertial.origin
                    assert abs(o2.xyz.x - o1.xyz.x) < 0.001, f"Link {link_name}: COM x mismatch"
                    assert abs(o2.xyz.y - o1.xyz.y) < 0.001, f"Link {link_name}: COM y mismatch"
                    assert abs(o2.xyz.z - o1.xyz.z) < 0.001, f"Link {link_name}: COM z mismatch"

            # Verify visual geometry origins
            for i, (vis1, vis2) in enumerate(zip(link1.visuals, link2.visuals, strict=False)):
                if vis1.origin:
                    assert vis2.origin is not None, f"Link {link_name} visual {i}: missing origin"
                    assert abs(vis2.origin.xyz.x - vis1.origin.xyz.x) < 0.001, (
                        f"Link {link_name} visual {i}: x mismatch"
                    )
                    assert abs(vis2.origin.xyz.y - vis1.origin.xyz.y) < 0.001, (
                        f"Link {link_name} visual {i}: y mismatch"
                    )
                    assert abs(vis2.origin.xyz.z - vis1.origin.xyz.z) < 0.001, (
                        f"Link {link_name} visual {i}: z mismatch"
                    )

                # Verify material
                if vis1.material:
                    assert vis2.material is not None, (
                        f"Link {link_name} visual {i}: missing material"
                    )
                    assert vis2.material.name == vis1.material.name, (
                        f"Link {link_name} visual {i}: material name mismatch"
                    )

        # ========== VERIFY JOINTS ==========
        joint_map1 = {joint.name: joint for joint in robot1.joints}
        joint_map2 = {joint.name: joint for joint in robot2.joints}

        assert set(joint_map1.keys()) == set(joint_map2.keys()), "Joint names don't match"

        for joint_name in joint_map1:
            joint1 = joint_map1[joint_name]
            joint2 = joint_map2[joint_name]

            # Verify parent-child
            assert joint2.parent == joint1.parent, f"Joint {joint_name}: parent link mismatch"
            assert joint2.child == joint1.child, f"Joint {joint_name}: child link mismatch"

            # Verify joint type
            assert joint2.type == joint1.type, (
                f"Joint {joint_name}: type mismatch ({joint2.type} vs {joint1.type})"
            )

            # Verify joint origin
            if joint1.origin:
                assert joint2.origin is not None, f"Joint {joint_name}: missing origin"
                assert abs(joint2.origin.xyz.x - joint1.origin.xyz.x) < 0.001, (
                    f"Joint {joint_name}: origin x mismatch"
                )
                assert abs(joint2.origin.xyz.y - joint1.origin.xyz.y) < 0.001, (
                    f"Joint {joint_name}: origin y mismatch"
                )
                assert abs(joint2.origin.xyz.z - joint1.origin.xyz.z) < 0.001, (
                    f"Joint {joint_name}: origin z mismatch"
                )
                assert abs(joint2.origin.rpy.x - joint1.origin.rpy.x) < 0.001, (
                    f"Joint {joint_name}: origin roll mismatch"
                )
                assert abs(joint2.origin.rpy.y - joint1.origin.rpy.y) < 0.001, (
                    f"Joint {joint_name}: origin pitch mismatch"
                )
                assert abs(joint2.origin.rpy.z - joint1.origin.rpy.z) < 0.001, (
                    f"Joint {joint_name}: origin yaw mismatch"
                )

            # Verify axis
            if joint1.axis:
                assert joint2.axis is not None, f"Joint {joint_name}: missing axis"
                assert abs(joint2.axis.x - joint1.axis.x) < 0.001, (
                    f"Joint {joint_name}: axis x mismatch"
                )
                assert abs(joint2.axis.y - joint1.axis.y) < 0.001, (
                    f"Joint {joint_name}: axis y mismatch"
                )
                assert abs(joint2.axis.z - joint1.axis.z) < 0.001, (
                    f"Joint {joint_name}: axis z mismatch"
                )

            # Verify limits
            if joint1.limits:
                assert joint2.limits is not None, f"Joint {joint_name}: missing limits"
                assert abs(joint2.limits.lower - joint1.limits.lower) < 0.001, (
                    f"Joint {joint_name}: lower limit mismatch"
                )
                assert abs(joint2.limits.upper - joint1.limits.upper) < 0.001, (
                    f"Joint {joint_name}: upper limit mismatch"
                )
                assert abs(joint2.limits.effort - joint1.limits.effort) < 0.001, (
                    f"Joint {joint_name}: effort limit mismatch"
                )
                assert abs(joint2.limits.velocity - joint1.limits.velocity) < 0.001, (
                    f"Joint {joint_name}: velocity limit mismatch"
                )

            # Verify dynamics
            if joint1.dynamics:
                assert joint2.dynamics is not None, f"Joint {joint_name}: missing dynamics"
                assert abs(joint2.dynamics.damping - joint1.dynamics.damping) < 0.001, (
                    f"Joint {joint_name}: damping mismatch"
                )
                assert abs(joint2.dynamics.friction - joint1.dynamics.friction) < 0.001, (
                    f"Joint {joint_name}: friction mismatch"
                )

            # Verify mimic
            if joint1.mimic:
                assert joint2.mimic is not None, f"Joint {joint_name}: missing mimic"
                assert joint2.mimic.joint == joint1.mimic.joint, (
                    f"Joint {joint_name}: mimic joint mismatch"
                )
                assert abs(joint2.mimic.multiplier - joint1.mimic.multiplier) < 0.001, (
                    f"Joint {joint_name}: mimic multiplier mismatch"
                )
                assert abs(joint2.mimic.offset - joint1.mimic.offset) < 0.001, (
                    f"Joint {joint_name}: mimic offset mismatch"
                )

        # ========== VERIFY TRANSMISSIONS ==========
        assert len(robot2.transmissions) == len(robot1.transmissions), "Transmission count mismatch"

        trans_map1 = {t.name: t for t in robot1.transmissions}
        trans_map2 = {t.name: t for t in robot2.transmissions}

        for trans_name in trans_map1:
            trans1 = trans_map1[trans_name]
            trans2 = trans_map2[trans_name]

            assert trans2.type == trans1.type, f"Transmission {trans_name}: type mismatch"
            assert len(trans2.joints) == len(trans1.joints), (
                f"Transmission {trans_name}: joint count mismatch"
            )

        # ========== VERIFY SENSORS ==========
        assert len(robot2.sensors) == len(robot1.sensors), "Sensor count mismatch"

        sensor_map1 = {s.name: s for s in robot1.sensors}
        sensor_map2 = {s.name: s for s in robot2.sensors}

        for sensor_name in sensor_map1:
            sensor1 = sensor_map1[sensor_name]
            sensor2 = sensor_map2[sensor_name]

            assert sensor2.type == sensor1.type, (
                f"Sensor {sensor_name}: type mismatch ({sensor2.type} vs {sensor1.type})"
            )
            assert sensor2.link_name == sensor1.link_name, (
                f"Sensor {sensor_name}: link name mismatch"
            )
            assert abs(sensor2.update_rate - sensor1.update_rate) < 0.01, (
                f"Sensor {sensor_name}: update rate mismatch"
            )

        # ========== VERIFY ROS2 CONTROL ==========
        assert len(robot2.ros2_controls) == len(robot1.ros2_controls), "Ros2Control count mismatch"

        rc_map1 = {rc.name: rc for rc in robot1.ros2_controls}
        rc_map2 = {rc.name: rc for rc in robot2.ros2_controls}

        assert set(rc_map1.keys()) == set(rc_map2.keys()), "Ros2Control names don't match"

        for rc_name in rc_map1:
            rc1 = rc_map1[rc_name]
            rc2 = rc_map2[rc_name]

            assert rc2.type == rc1.type, f"Ros2Control {rc_name}: type mismatch"
            assert rc2.hardware_plugin == rc1.hardware_plugin, (
                f"Ros2Control {rc_name}: hardware plugin mismatch"
            )
            assert len(rc2.joints) == len(rc1.joints), (
                f"Ros2Control {rc_name}: joint count mismatch"
            )

            # Check joints inside
            rc_joints1 = {j.name: j for j in rc1.joints}
            rc_joints2 = {j.name: j for j in rc2.joints}
            assert set(rc_joints1.keys()) == set(rc_joints2.keys())

            for j_name in rc_joints1:
                j1 = rc_joints1[j_name]
                j2 = rc_joints2[j_name]
                assert set(j2.command_interfaces) == set(j1.command_interfaces)
                assert set(j2.state_interfaces) == set(j1.state_interfaces)

    finally:
        # Cleanup
        temp_path.unlink()


def test_joint_origin_consistency(examples_dir: Path):
    """Test that joint origins are consistent across import-export-import."""
    original_path = examples_dir / "urdf" / "roundtrip_test_robot.urdf"
    robot1 = URDFParser().parse(original_path)

    # Export
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        generator = URDFGenerator()
        f.write(generator.generate(robot1))

    try:
        robot2 = URDFParser().parse(temp_path)

        # Check specific critical joints
        critical_joints = ["arm_base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]

        joint_map1 = {j.name: j for j in robot1.joints}
        joint_map2 = {j.name: j for j in robot2.joints}

        for joint_name in critical_joints:
            if joint_name in joint_map1:
                j1 = joint_map1[joint_name]
                j2 = joint_map2[joint_name]

                # Compare origins precisely
                if j1.origin and j2.origin:
                    print(f"\n{joint_name}:")
                    print(
                        f"  Original: xyz=({j1.origin.xyz.x}, {j1.origin.xyz.y}, {j1.origin.xyz.z})"
                    )
                    print(
                        f"  Re-import: xyz=({j2.origin.xyz.x}, {j2.origin.xyz.y}, {j2.origin.xyz.z})"
                    )

                    # Verify they match
                    assert abs(j2.origin.xyz.x - j1.origin.xyz.x) < 0.00001
                    assert abs(j2.origin.xyz.y - j1.origin.xyz.y) < 0.00001
                    assert abs(j2.origin.xyz.z - j1.origin.xyz.z) < 0.00001

    finally:
        temp_path.unlink()


def test_visual_geometry_origins_preserved(examples_dir: Path):
    """Test that visual geometry origins (offsets) are preserved."""
    original_path = examples_dir / "urdf" / "roundtrip_test_robot.urdf"
    robot1 = URDFParser().parse(original_path)

    # Export
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        generator = URDFGenerator()
        f.write(generator.generate(robot1))

    try:
        robot2 = URDFParser().parse(temp_path)

        # Check links with visual offsets
        links_with_offsets = ["upper_arm", "forearm", "left_finger", "right_finger"]

        link_map1 = {link.name: link for link in robot1.links}
        link_map2 = {link.name: link for link in robot2.links}

        for link_name in links_with_offsets:
            if link_name in link_map1:
                l1 = link_map1[link_name]
                l2 = link_map2[link_name]

                if l1.visuals and l2.visuals:
                    v1 = l1.visuals[0]
                    v2 = l2.visuals[0]

                    if v1.origin and v2.origin:
                        print(f"\n{link_name} visual origin:")
                        print(
                            f"  Original: ({v1.origin.xyz.x}, {v1.origin.xyz.y}, {v1.origin.xyz.z})"
                        )
                        print(
                            f"  Re-import: ({v2.origin.xyz.x}, {v2.origin.xyz.y}, {v2.origin.xyz.z})"
                        )

                        # Verify offsets preserved
                        assert abs(v2.origin.xyz.x - v1.origin.xyz.x) < 0.00001
                        assert abs(v2.origin.xyz.y - v1.origin.xyz.y) < 0.00001
                        assert abs(v2.origin.xyz.z - v1.origin.xyz.z) < 0.00001

    finally:
        temp_path.unlink()


def test_inertial_origins_preserved(examples_dir: Path):
    """Test that inertial origins (center of mass) are preserved in roundtrip."""
    original_path = examples_dir / "urdf" / "roundtrip_test_robot.urdf"
    robot1 = URDFParser().parse(original_path)

    # Export
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        temp_path = Path(f.name)
        generator = URDFGenerator()
        f.write(generator.generate(robot1))

    try:
        robot2 = URDFParser().parse(temp_path)

        # Check links with non-zero inertial origins
        links_with_com_offset = ["base_link", "upper_arm", "forearm", "left_finger", "right_finger"]

        link_map1 = {link.name: link for link in robot1.links}
        link_map2 = {link.name: link for link in robot2.links}

        for link_name in links_with_com_offset:
            if link_name in link_map1:
                l1 = link_map1[link_name]
                l2 = link_map2[link_name]

                if l1.inertial and l1.inertial.origin and l2.inertial and l2.inertial.origin:
                    o1 = l1.inertial.origin
                    o2 = l2.inertial.origin

                    print(f"\n{link_name} inertial origin:")
                    print(f"  Original: ({o1.xyz.x}, {o1.xyz.y}, {o1.xyz.z})")
                    print(f"  Re-import: ({o2.xyz.x}, {o2.xyz.y}, {o2.xyz.z})")

                    # Verify COM position preserved
                    assert abs(o2.xyz.x - o1.xyz.x) < 0.00001
                    assert abs(o2.xyz.y - o1.xyz.y) < 0.00001
                    assert abs(o2.xyz.z - o1.xyz.z) < 0.00001

                    # Verify COM rotation preserved
                    assert abs(o2.rpy.x - o1.rpy.x) < 0.00001
                    assert abs(o2.rpy.y - o1.rpy.y) < 0.00001
                    assert abs(o2.rpy.z - o1.rpy.z) < 0.00001

    finally:
        temp_path.unlink()


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
