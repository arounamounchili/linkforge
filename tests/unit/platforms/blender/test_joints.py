"""Unit tests for Blender Joint operations, properties, and utilities."""

from __future__ import annotations

import bpy
import pytest
from linkforge.blender.visualization.joint_gizmos import (
    fix_existing_joints,
    generate_axis_geometry,
    update_viz_handle,
)

from tests.blender_test_utils import (
    create_robot_joint,
    safe_get_joint,
    safe_get_linkforge,
)

# Joint Operations


class TestJointOperations:
    def test_create_joint_object(self, scene, blender_context) -> None:
        """Test creating a joint object in Blender."""
        joint_obj = create_robot_joint("test_joint", None, None, scene)
        assert joint_obj.name.startswith("test_joint")
        assert joint_obj.type == "EMPTY"
        assert joint_obj.empty_display_type == "PLAIN_AXES"
        assert safe_get_joint(joint_obj).is_robot_joint

    def test_create_joint_with_parent(self, scene, blender_context) -> None:
        """Test creating a joint with a parent link."""
        from tests.blender_test_utils import create_test_object

        parent = create_test_object("parent_link", None, scene)
        safe_get_linkforge(parent).is_robot_link = True
        # structure: Parent -> Joint
        joint_obj = create_robot_joint("child_joint", parent, None, scene)
        assert joint_obj.parent == parent


# Joint Properties


class TestJointProperties:
    def test_joint_property_defaults(self, scene, blender_context) -> None:
        """Test default values for joint properties."""
        from tests.blender_test_utils import create_test_object

        obj = create_test_object("test_obj", None, scene)
        props = safe_get_joint(obj)

        assert not props.is_robot_joint
        assert props.joint_type == "revolute"
        assert props.axis == "Z"
        assert props.custom_axis_x == 0.0
        assert props.limit_lower == pytest.approx(-3.14159, abs=1e-3)
        assert props.limit_upper == pytest.approx(3.14159, abs=1e-3)


# Joint Utilities


class TestJointUtilities:
    def test_joint_axis_properties(self, scene, blender_context) -> None:
        """Test setting and getting joint axis properties."""
        from tests.blender_test_utils import create_test_object

        obj = create_test_object("test_axis", None, scene)
        props = safe_get_joint(obj)

        props.axis = "CUSTOM"
        props.custom_axis_z = 1.0
        assert props.custom_axis_z == 1.0

    def test_joint_origin_calculation(self, scene, blender_context) -> None:
        """Test joint origin persistence in properties."""
        from mathutils import Vector

        from tests.blender_test_utils import create_test_object

        obj = create_test_object("test_origin", None, scene)
        obj.location = Vector((1.0, 2.0, 3.0))
        # Joint origin is usually the object's local transform relative to parent
        assert obj.location.x == 1.0

    def test_is_robot_joint(self, scene, blender_context) -> None:
        """Test joint identification utility."""
        from linkforge.blender.utils.scene_utils import is_robot_joint

        from tests.blender_test_utils import create_test_object

        obj = create_test_object("test_is_joint", None, scene)
        assert not is_robot_joint(obj)

        safe_get_joint(obj).is_robot_joint = True
        assert is_robot_joint(obj)


# Joint Visualization (Gizmos)


class TestJointVisualization:
    def test_generate_axis_geometry(self, scene, blender_context) -> None:
        """Test generating geometry for joint axis visualization."""
        from mathutils import Vector

        from tests.blender_test_utils import create_test_object

        obj = create_test_object("test_gizmo", None, scene)
        obj.location = Vector((1.0, 2.0, 3.0))
        if blender_context.view_layer:
            blender_context.view_layer.update()
        props = safe_get_joint(obj)
        props.is_robot_joint = True
        props.axis = "CUSTOM"
        props.custom_axis_x = 1.0
        props.custom_axis_y = 0.0
        props.custom_axis_z = 0.0

        data = generate_axis_geometry(obj)
        assert "lines" in data
        assert len(data["lines"]) == 6
        # Start point should be at origin (1, 2, 3)
        assert data["lines"][0] == pytest.approx((1.0, 2.0, 3.0))

    def test_fix_existing_joints(self, scene, blender_context) -> None:
        """Test the iteration logic that forces PLAIN_AXES on joints."""
        from tests.blender_test_utils import create_test_object

        obj = create_test_object("test_fix", None, scene)
        safe_get_joint(obj).is_robot_joint = True
        obj.empty_display_type = "CUBE"

        fix_existing_joints()
        assert obj.empty_display_type == "PLAIN_AXES"

    def test_update_viz_handle_switching(self, mocker, scene, blender_context) -> None:
        """Test registering and unregistering the draw handler based on prefs."""
        mock_add = mocker.patch("bpy.types.SpaceView3D.draw_handler_add", return_value="handle_123")
        mock_remove = mocker.patch("bpy.types.SpaceView3D.draw_handler_remove")
        mock_prefs = mocker.patch("linkforge.blender.visualization.joint_gizmos.get_addon_prefs")

        class MockPrefs:
            show_joint_axes: bool = False

        prefs = MockPrefs()
        # Test ENABLE
        prefs.show_joint_axes = True
        mock_prefs.return_value = prefs
        update_viz_handle(bpy.context)
        mock_add.assert_called_once()
        assert bpy.app.driver_namespace["linkforge_joint_gizmo_handler"] == "handle_123"

        # Test DISABLE
        prefs.show_joint_axes = False
        update_viz_handle(bpy.context)
        mock_remove.assert_called()
        assert "linkforge_joint_gizmo_handler" not in bpy.app.driver_namespace
