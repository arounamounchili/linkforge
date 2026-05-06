"""Unit tests for complex robot assembly and semantic helpers."""

import pytest
from linkforge_core.models.geometry import Vector3
from linkforge_core.models.joint import Joint, JointLimits, JointType
from linkforge_core.models.link import Link
from linkforge_core.models.robot import Robot
from linkforge_core.models.sensor import CameraInfo, Sensor, SensorType


@pytest.fixture
def base_robot():
    """Create a simple base robot (a stand)."""
    robot = Robot(name="stand")
    robot.add_link(Link(name="base_link"))
    robot.add_link(Link(name="mount_point"))
    robot.add_joint(
        Joint(name="mount_joint", type=JointType.FIXED, parent="base_link", child="mount_point")
    )
    return robot


@pytest.fixture
def arm_robot():
    """Create a simple arm with a sensor."""
    robot = Robot(name="arm")
    robot.add_link(Link(name="arm_base"))
    robot.add_link(Link(name="link1"))
    robot.add_joint(
        Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="arm_base",
            child="link1",
            axis=Vector3(0, 0, 1),  # Axis is required for revolute
            limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
        )
    )
    robot.add_sensor(
        Sensor(
            name="camera",
            type=SensorType.CAMERA,
            link_name="link1",
            camera_info=CameraInfo(),
        )
    )

    # Add some semantic data
    robot.add_group("arm_group", links=["arm_base", "link1"])
    robot.disable_collisions("arm_base", "link1", reason="Adjacent")

    return robot


def test_robot_merge_integrity(base_robot, arm_robot):
    """Test merging an arm onto a stand."""
    # Merge arm onto the mount_point of the stand
    base_robot.merge(
        component=arm_robot, at_link="mount_point", joint_name="stand_to_arm", prefix="r1_"
    )

    # 1. Check Kinematics
    assert base_robot.has_link("base_link")
    assert base_robot.has_link("r1_arm_base")
    assert base_robot.has_link("r1_link1")
    assert base_robot.has_joint("stand_to_arm")

    # Verify connection
    conn_joint = base_robot.joint("stand_to_arm")
    assert conn_joint.parent == "mount_point"
    assert conn_joint.child == "r1_arm_base"

    # 2. Check Functional Elements
    assert base_robot.has_sensor("r1_camera")
    assert base_robot.sensor("r1_camera").link_name == "r1_link1"

    # 3. Check Semantic Data
    semantic = base_robot.semantic
    assert len(semantic.groups) == 1
    assert semantic.groups[0].name == "r1_arm_group"
    assert "r1_link1" in semantic.groups[0].links

    assert len(semantic.disabled_collisions) == 1
    assert semantic.disabled_collisions[0].link1 == "r1_arm_base"
    assert semantic.disabled_collisions[0].link2 == "r1_link1"


def test_semantic_helpers(base_robot):
    """Test standalone semantic helper methods in Robot."""
    base_robot.add_group("test_group", links=["base_link"])
    base_robot.disable_collisions("base_link", "mount_point", reason="Fixed")
    base_robot.disable_default_collisions("base_link")
    base_robot.add_joint_property("mount_joint", "type", "static")

    semantic = base_robot.semantic
    assert any(g.name == "test_group" for g in semantic.groups)
    assert any(dc.link1 == "base_link" for dc in semantic.disabled_collisions)
    assert "base_link" in semantic.no_default_collision_links
    assert any(jp.property_name == "type" for jp in semantic.joint_properties)


def test_merge_collision_deduplication(base_robot):
    """Test that merging integrates collision rules from both robots."""
    # Add a collision rule to base
    base_robot.disable_collisions("base_link", "mount_point")

    # Create another robot with its own rule
    other = Robot(name="other")
    other.add_link(Link(name="linkA"))
    other.add_link(Link(name="linkB"))
    other.add_joint(Joint(name="j", type=JointType.FIXED, parent="linkA", child="linkB"))
    other.disable_collisions("linkA", "linkB")

    # Merge with prefix
    base_robot.merge(other, at_link="mount_point", joint_name="bridge", prefix="other_")

    # Should have both rules now
    assert len(base_robot.semantic.disabled_collisions) == 2

    # Verify the prefixed rule is correct
    rules = [
        dc
        for dc in base_robot.semantic.disabled_collisions
        if {dc.link1, dc.link2} == {"other_link1", "other_link2"}
    ]
    # Wait, my prefix was other_, so it should be other_linkA and other_linkB
    rules = [
        dc
        for dc in base_robot.semantic.disabled_collisions
        if {dc.link1, dc.link2} == {"other_linkA", "other_linkB"}
    ]
    assert len(rules) == 1
