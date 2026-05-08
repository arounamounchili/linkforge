"""Comprehensive roundtrip test - Import → Export → Re-import verification.

This test verifies that the complete workflow preserves all robot properties.
"""

from __future__ import annotations

from pathlib import Path

from linkforge_core.parsers.urdf_parser import URDFParser

from tests.core_test_utils import assert_robots_equal, perform_urdf_roundtrip


def test_comprehensive_roundtrip_preserves_structure(examples_dir: Path) -> None:
    """Test that export → re-import preserves robot structure perfectly."""
    # Load original
    original_path = examples_dir / "urdf" / "roundtrip_test_robot.urdf"
    robot1 = URDFParser().parse(original_path)

    # Roundtrip
    robot2 = perform_urdf_roundtrip(robot1)

    # Assert equality using comprehensive comparison engine
    assert_robots_equal(robot1, robot2)


def test_joint_origin_consistency(examples_dir: Path) -> None:
    """Test that joint origins are consistent across import-export-import."""
    original_path = examples_dir / "urdf" / "roundtrip_test_robot.urdf"
    robot1 = URDFParser().parse(original_path)

    robot2 = perform_urdf_roundtrip(robot1)

    critical_joints = ["arm_base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]
    joint_map1 = {j.name: j for j in robot1.joints}
    joint_map2 = {j.name: j for j in robot2.joints}

    for joint_name in critical_joints:
        if joint_name in joint_map1:
            j1 = joint_map1[joint_name]
            j2 = joint_map2[joint_name]
            assert abs(j2.origin.xyz.x - j1.origin.xyz.x) < 1e-6
            assert abs(j2.origin.xyz.y - j1.origin.xyz.y) < 1e-6
            assert abs(j2.origin.xyz.z - j1.origin.xyz.z) < 1e-6


def test_visual_geometry_origins_preserved(examples_dir: Path) -> None:
    """Test that visual geometry origins (offsets) are preserved."""
    original_path = examples_dir / "urdf" / "roundtrip_test_robot.urdf"
    robot1 = URDFParser().parse(original_path)

    robot2 = perform_urdf_roundtrip(robot1)

    links_with_offsets = ["upper_arm", "forearm", "left_finger", "right_finger"]
    link_map1 = {link.name: link for link in robot1.links}
    link_map2 = {link.name: link for link in robot2.links}

    for link_name in links_with_offsets:
        if link_name in link_map1:
            l1 = link_map1[link_name]
            l2 = link_map2[link_name]
            v1 = l1.visuals[0]
            v2 = l2.visuals[0]
            assert abs(v2.origin.xyz.x - v1.origin.xyz.x) < 1e-6
            assert abs(v2.origin.xyz.y - v1.origin.xyz.y) < 1e-6
            assert abs(v2.origin.xyz.z - v1.origin.xyz.z) < 1e-6


def test_inertial_origins_preserved(examples_dir: Path) -> None:
    """Test that inertial origins (center of mass) are preserved in roundtrip."""
    original_path = examples_dir / "urdf" / "roundtrip_test_robot.urdf"
    robot1 = URDFParser().parse(original_path)

    robot2 = perform_urdf_roundtrip(robot1)

    links_with_com_offset = ["base_link", "upper_arm", "forearm", "left_finger", "right_finger"]
    link_map1 = {link.name: link for link in robot1.links}
    link_map2 = {link.name: link for link in robot2.links}

    for link_name in links_with_com_offset:
        if link_name in link_map1:
            l1 = link_map1[link_name]
            l2 = link_map2[link_name]
            o1 = l1.inertial.origin
            o2 = l2.inertial.origin
            assert abs(o2.xyz.x - o1.xyz.x) < 1e-6
            assert abs(o2.xyz.y - o1.xyz.y) < 1e-6
            assert abs(o2.xyz.z - o1.xyz.z) < 1e-6
