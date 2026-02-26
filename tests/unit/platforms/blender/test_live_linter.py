"""Unit tests for the LiveLinter logic within the Blender environment.

Verifies linter status reporting and kinematic validation using real bpy objects.
"""

import bpy
from linkforge.blender.logic.live_linter import _perform_validation


def test_linter_cycle_detection():
    """Verify that the linter correctly reports kinematic cycles in the viewport."""
    scene = bpy.context.scene
    scene.linkforge.linter_active = True

    # Create two links in a cycle: A -> B, B -> A
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    link_a = bpy.context.active_object
    link_a.name = "link_A"
    link_a.linkforge.is_robot_link = True

    bpy.ops.object.empty_add(type="PLAIN_AXES")
    link_b = bpy.context.active_object
    link_b.name = "link_B"
    link_b.linkforge.is_robot_link = True

    # Joint A -> B
    bpy.ops.object.empty_add(type="ARROWS")
    joint_ab = bpy.context.active_object
    joint_ab.linkforge_joint.is_robot_joint = True
    joint_ab.linkforge_joint.parent_link = link_a
    joint_ab.linkforge_joint.child_link = link_b

    # Joint B -> A (The Cycle)
    bpy.ops.object.empty_add(type="ARROWS")
    joint_ba = bpy.context.active_object
    joint_ba.linkforge_joint.is_robot_joint = True
    joint_ba.linkforge_joint.parent_link = link_b
    joint_ba.linkforge_joint.child_link = link_a

    try:
        # Run validation
        _perform_validation()

        # Verify status updates
        assert "CRITICAL" in scene.linkforge.linter_status
        assert scene.linkforge.linter_error_count > 0
    finally:
        # Cleanup
        bpy.data.objects.remove(link_a, do_unlink=True)
        bpy.data.objects.remove(link_b, do_unlink=True)
        bpy.data.objects.remove(joint_ab, do_unlink=True)
        bpy.data.objects.remove(joint_ba, do_unlink=True)


def test_linter_ready_state():
    """Verify that the linter reports 'Ready' when the kinematic tree is valid."""
    scene = bpy.context.scene
    scene.linkforge.linter_active = True

    # Simple valid chain: A -> B
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    link_a = bpy.context.active_object
    link_a.linkforge.is_robot_link = True

    bpy.ops.object.empty_add(type="PLAIN_AXES")
    link_b = bpy.context.active_object
    link_b.linkforge.is_robot_link = True

    bpy.ops.object.empty_add(type="ARROWS")
    joint = bpy.context.active_object
    joint.linkforge_joint.is_robot_joint = True
    joint.linkforge_joint.parent_link = link_a
    joint.linkforge_joint.child_link = link_b

    try:
        _perform_validation()
        assert scene.linkforge.linter_status == "Ready"
        assert scene.linkforge.linter_error_count == 0
    finally:
        bpy.data.objects.remove(link_a, do_unlink=True)
        bpy.data.objects.remove(link_b, do_unlink=True)
        bpy.data.objects.remove(joint, do_unlink=True)


def test_linter_inactive():
    """Verify that validation is skipped if the user disables the linter."""
    scene = bpy.context.scene
    scene.linkforge.linter_active = False
    scene.linkforge.linter_status = "Initial"

    _perform_validation()

    # Status should remain unchanged if skipped
    assert scene.linkforge.linter_status == "Initial"
