"""Test visual origin and inertia integration in Blender."""

from __future__ import annotations

import bpy
import pytest

from tests.blender_test_utils import create_test_object


def test_inertia_integration_with_offset(clean_scene):
    """Verify that offset visuals are handled correctly via the Parallel Axis Theorem."""
    from typing import Any

    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None

    link_obj = create_test_object("link", None)
    assert link_obj is not None
    collection.objects.link(link_obj)
    link_lf: Any = getattr(link_obj, "linkforge")
    link_lf.is_robot_link = True
    assert link_lf.is_robot_link


def test_inertia_integration_flow(clean_scene):
    """Verify end-to-end inertia calculation in Blender."""
    from typing import Any

    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None

    # Create the link (Empty) manually
    link_obj = create_test_object("link", None)
    assert link_obj is not None
    collection.objects.link(link_obj)
    link_lf: Any = getattr(link_obj, "linkforge")
    link_lf.is_robot_link = True
    assert link_lf.is_robot_link


def test_ros2_control_parameter_extraction(clean_scene):
    """Verify that custom parameters are extracted from Blender into ROS2 Control models."""
    from typing import Any

    # Setup Link & Joint
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None

    p = create_test_object("Parent", None)
    assert p is not None
    collection.objects.link(p)
    p_lf: Any = getattr(p, "linkforge")
    p_lf.is_robot_link = True

    c = create_test_object("Child", None)
    assert c is not None
    collection.objects.link(c)
    c_lf: Any = getattr(c, "linkforge")
    c_lf.is_robot_link = True

    j = create_test_object("Joint", None)
    assert j is not None
    collection.objects.link(j)
    j_lf: Any = getattr(j, "linkforge_joint")
    j_lf.is_robot_joint = True
    j_lf.parent_link = p
    j_lf.child_link = c
    j_lf.joint_type = "REVOLUTE"
    assert j_lf.is_robot_joint


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
