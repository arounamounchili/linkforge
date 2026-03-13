"""Tests for joint_utils module."""

import bpy
from linkforge.blender.utils.joint_utils import resolve_mimic_joints
from linkforge_core.models import Joint, JointLimits, JointMimic, JointType, Vector3


def test_resolve_mimic_joints_basic() -> None:
    """Test basic mimic joint resolution."""
    # Create two joints
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    driver_obj = bpy.context.active_object
    driver_obj.name = "driver_joint"
    driver_obj.linkforge_joint.is_robot_joint = True

    bpy.ops.object.empty_add()
    follower_obj = bpy.context.active_object
    follower_obj.name = "follower_joint"
    follower_obj.linkforge_joint.is_robot_joint = True

    # Create joint models
    mimic = JointMimic(joint="driver_joint", multiplier=2.0, offset=0.5)
    limits = JointLimits(lower=-3.14, upper=3.14, effort=10.0, velocity=1.0)
    driver_joint = Joint(
        name="driver_joint",
        type=JointType.REVOLUTE,
        parent="link1",
        child="link2",
        axis=Vector3(1, 0, 0),
        limits=limits,
    )
    follower_joint = Joint(
        name="follower_joint",
        type=JointType.REVOLUTE,
        parent="link2",
        child="link3",
        axis=Vector3(1, 0, 0),
        limits=limits,
        mimic=mimic,
    )

    joint_objects = {
        "driver_joint": driver_obj,
        "follower_joint": follower_obj,
    }

    # Resolve mimic joints
    resolve_mimic_joints([driver_joint, follower_joint], joint_objects)

    # Verify the mimic joint was set
    assert follower_obj.linkforge_joint.mimic_joint == driver_obj


def test_resolve_mimic_joints_missing_driver() -> None:
    """Test mimic joint resolution when driver joint doesn't exist."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    follower_obj = bpy.context.active_object
    follower_obj.name = "follower_joint"
    follower_obj.linkforge_joint.is_robot_joint = True

    # Create joint model with mimic referencing non-existent joint
    mimic = JointMimic(joint="nonexistent_joint", multiplier=1.0, offset=0.0)
    limits = JointLimits(lower=-3.14, upper=3.14, effort=10.0, velocity=1.0)
    follower_joint = Joint(
        name="follower_joint",
        type=JointType.REVOLUTE,
        parent="link1",
        child="link2",
        axis=Vector3(1, 0, 0),
        limits=limits,
        mimic=mimic,
    )

    joint_objects = {"follower_joint": follower_obj}

    # Should not raise error, just skip resolution
    resolve_mimic_joints([follower_joint], joint_objects)

    # Mimic joint should not be set
    assert follower_obj.linkforge_joint.mimic_joint is None


def test_resolve_mimic_joints_no_mimic() -> None:
    """Test processing joints without mimic."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    bpy.ops.object.empty_add()
    joint_obj = bpy.context.active_object
    joint_obj.name = "simple_joint"
    joint_obj.linkforge_joint.is_robot_joint = True

    # Create joint without mimic
    limits = JointLimits(lower=-3.14, upper=3.14, effort=10.0, velocity=1.0)
    simple_joint = Joint(
        name="simple_joint",
        type=JointType.REVOLUTE,
        parent="link1",
        child="link2",
        axis=Vector3(1, 0, 0),
        limits=limits,
    )

    joint_objects = {"simple_joint": joint_obj}

    # Should complete without errors
    resolve_mimic_joints([simple_joint], joint_objects)

    assert joint_obj.linkforge_joint.mimic_joint is None


def test_resolve_mimic_joints_empty_lists() -> None:
    """Test with empty joint lists."""
    resolve_mimic_joints([], {})
    # Should not raise errors
