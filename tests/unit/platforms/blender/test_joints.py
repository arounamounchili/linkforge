"""Unit tests for Blender Joint operations, properties, and utilities."""

from __future__ import annotations

from unittest.mock import patch

import bpy
import pytest
from linkforge.blender.operators.joint_ops import (
    LINKFORGE_OT_auto_detect_parent_child,
    LINKFORGE_OT_create_joint,
    LINKFORGE_OT_delete_joint,
)
from linkforge.blender.visualization.joint_gizmos import (
    fix_existing_joints,
    generate_axis_geometry,
    update_viz_handle,
)

from tests.blender_test_utils import (
    create_robot_joint,
    create_robot_link,
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_linkforge_scene,
)


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
        parent = create_test_object("parent_link", None, scene)
        safe_get_linkforge(parent).is_robot_link = True
        joint_obj = create_robot_joint("child_joint", parent, None, scene)
        assert joint_obj.parent == parent

    def test_create_joint_operator_poll(self, mocker, scene, blender_context) -> None:
        """Test create joint operator poll method."""
        from unittest.mock import PropertyMock

        op = LINKFORGE_OT_create_joint

        # Active object is None
        mocker.patch.object(
            type(bpy.context), "active_object", new_callable=PropertyMock, return_value=None
        )
        assert not op.poll(bpy.context)

        # Active object exists but not selected
        link = create_robot_link("base", scene)
        link.select_set(False)
        mocker.patch.object(
            type(bpy.context), "active_object", new_callable=PropertyMock, return_value=link
        )
        assert not op.poll(bpy.context)

        # Selected and is link
        link.select_set(True)
        mocker.patch.object(
            type(bpy.context), "active_object", new_callable=PropertyMock, return_value=link
        )
        assert op.poll(bpy.context)

    def test_create_joint_operator_execute(self, scene, blender_context) -> None:
        """Test create joint operator execution."""
        link = create_robot_link("base", scene)
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = link
        link.select_set(True)

        op = LINKFORGE_OT_create_joint()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        # Check joint was created
        joint = bpy.context.active_object
        assert joint is not None
        assert joint.name.startswith("base_joint")
        assert safe_get_joint(joint).is_robot_joint
        assert safe_get_joint(joint).child_link == link

    def test_create_joint_operator_fallback(self, scene, blender_context) -> None:
        """Test create joint operator fallback when pref is missing."""
        link = create_robot_link("base", scene)
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = link
        link.select_set(True)

        with patch("linkforge.blender.preferences.get_addon_prefs", return_value=None):
            op = LINKFORGE_OT_create_joint()
            res = op.execute(bpy.context)
            assert res == {"FINISHED"}

    def test_delete_joint_operator(self, scene, blender_context) -> None:
        """Test delete joint operator poll and execute."""
        base = create_robot_link("base", scene)
        child = create_robot_link("child", scene)
        joint_obj = create_robot_joint("test_joint", base, child, scene)

        # Setup ROS2 control items to test cleanup
        props = safe_get_linkforge_scene(scene)
        rc_joint = props.ros2_control_joints.add()
        rc_joint.name = joint_obj.name

        op = LINKFORGE_OT_delete_joint

        # Poll should fail if not joint or not empty
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = base
        assert not op.poll(bpy.context)

        # Poll passes on joint
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = joint_obj
        assert op.poll(bpy.context)

        # Execute
        res = op().execute(bpy.context)
        assert res == {"FINISHED"}
        assert joint_obj.name not in scene.objects
        assert len(props.ros2_control_joints) == 0

    def test_auto_detect_parent_child_operator(self, scene, blender_context) -> None:
        """Test auto detect parent/child operator."""
        base = create_robot_link("base", scene)
        child = create_robot_link("child", scene)
        joint_obj = create_robot_joint("test_joint", None, None, scene)

        op = LINKFORGE_OT_auto_detect_parent_child

        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = joint_obj
        joint_obj.select_set(True)
        assert op.poll(bpy.context)

        # Execute auto detect when both links exist
        res = op().execute(bpy.context)
        assert res == {"FINISHED"}

        jp = safe_get_joint(joint_obj)
        assert jp.child_link == base or jp.child_link == child

    def test_auto_detect_parent_child_no_links(self, scene, blender_context) -> None:
        """Test auto-detect when no links are present in scene."""
        joint_obj = create_robot_joint("test_joint", None, None, scene)
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = joint_obj
        joint_obj.select_set(True)

        res = LINKFORGE_OT_auto_detect_parent_child().execute(bpy.context)
        assert res == {"CANCELLED"}


class TestJointProperties:
    def test_joint_property_defaults(self, scene, blender_context) -> None:
        """Test default values for joint properties."""
        obj = create_test_object("test_obj", None, scene)
        props = safe_get_joint(obj)

        assert not props.is_robot_joint
        assert props.joint_type == "revolute"
        assert props.axis == "Z"
        assert props.custom_axis_x == 0.0
        assert props.limit_lower == pytest.approx(-3.14159, abs=1e-3)
        assert props.limit_upper == pytest.approx(3.14159, abs=1e-3)


class TestJointUtilities:
    def test_joint_axis_properties(self, scene, blender_context) -> None:
        """Test setting and getting joint axis properties."""
        obj = create_test_object("test_axis", None, scene)
        props = safe_get_joint(obj)

        props.axis = "CUSTOM"
        props.custom_axis_z = 1.0
        assert props.custom_axis_z == 1.0

    def test_joint_origin_calculation(self, scene, blender_context) -> None:
        """Test joint origin persistence in properties."""
        from mathutils import Vector

        obj = create_test_object("test_origin", None, scene)
        obj.location = Vector((1.0, 2.0, 3.0))
        assert obj.location.x == 1.0

    def test_is_robot_joint(self, scene, blender_context) -> None:
        """Test joint identification utility."""
        from linkforge.blender.utils.scene_utils import is_robot_joint

        obj = create_test_object("test_is_joint", None, scene)
        assert not is_robot_joint(obj)

        safe_get_joint(obj).is_robot_joint = True
        assert is_robot_joint(obj)


class TestJointVisualization:
    def test_generate_axis_geometry(self, scene, blender_context) -> None:
        """Test generating geometry for joint axis visualization."""
        from mathutils import Vector

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
        assert data["lines"][0] == pytest.approx((1.0, 2.0, 3.0))

    def test_fix_existing_joints(self, scene, blender_context) -> None:
        """Test the iteration logic that forces PLAIN_AXES on joints."""
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
        prefs.show_joint_axes = True
        mock_prefs.return_value = prefs
        update_viz_handle(bpy.context)
        mock_add.assert_called_once()
        assert bpy.app.driver_namespace["linkforge_joint_gizmo_handler"] == "handle_123"

        prefs.show_joint_axes = False
        update_viz_handle(bpy.context)
        mock_remove.assert_called()
        assert "linkforge_joint_gizmo_handler" not in bpy.app.driver_namespace


class TestJointUtils:
    def test_resolve_mimic_joints(self, scene, blender_context) -> None:
        """Test resolve_mimic_joints logic and branches in joint_utils.py."""
        from linkforge.blender.utils.joint_utils import resolve_mimic_joints
        from linkforge.core import Joint, JointMimic, JointType

        joint1_obj = create_robot_joint("joint1", None, None, scene)
        joint2_obj = create_robot_joint("joint2", None, None, scene)

        joint_objects = {
            "joint1": joint1_obj,
            "joint2": joint2_obj,
        }

        # 1. Normal resolution: joint2 mimics joint1
        joints = [
            Joint(
                name="joint2",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
                mimic=JointMimic(joint="joint1", multiplier=1.5, offset=0.2),
            )
        ]

        resolve_mimic_joints(joints, joint_objects)

        jp2 = safe_get_joint(joint2_obj)
        assert jp2.use_mimic is True
        assert jp2.mimic_joint == joint1_obj
        assert jp2.mimic_multiplier == pytest.approx(1.5)
        assert jp2.mimic_offset == pytest.approx(0.2)

        # 2. Branch: mimic joint target not in joint_objects
        joint3_obj = create_robot_joint("joint3", None, None, scene)
        joint_objects_missing = {
            "joint3": joint3_obj,
        }
        joints_missing = [
            Joint(
                name="joint3",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
                mimic=JointMimic(joint="non_existent_joint", multiplier=2.0, offset=0.5),
            )
        ]

        resolve_mimic_joints(joints_missing, joint_objects_missing)
        jp3 = safe_get_joint(joint3_obj)
        assert jp3.use_mimic is False
        assert jp3.mimic_multiplier == pytest.approx(2.0)
        assert jp3.mimic_offset == pytest.approx(0.5)

        # 3. Branch: get_joint_props returns None
        class FakeObject:
            def __init__(self) -> None:
                self.linkforge_joint = None

        fake_obj = FakeObject()
        joint_objects_fake = {
            "joint4": fake_obj,
        }
        joints_fake = [
            Joint(
                name="joint4",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
                mimic=JointMimic(joint="joint1"),
            )
        ]
        resolve_mimic_joints(joints_fake, joint_objects_fake)

        # 4. Branch: joint does not mimic or joint name not in joint_objects
        joints_no_mimic = [
            Joint(
                name="joint_no_mimic",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
                mimic=None,
            ),
            Joint(
                name="joint_not_in_objects",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
                mimic=JointMimic(joint="joint1"),
            ),
        ]
        resolve_mimic_joints(joints_no_mimic, joint_objects)
