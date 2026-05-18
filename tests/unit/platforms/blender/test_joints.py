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
        non_match = props.ros2_control_joints.add()
        non_match.name = "non_matching_joint"
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
        assert len(props.ros2_control_joints) == 1
        assert props.ros2_control_joints[0].name == "non_matching_joint"

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

    def test_joint_ops_invalid_context(self, scene) -> None:
        """Verify joint operators handle invalid context gracefully."""
        op = LINKFORGE_OT_create_joint()

        class MockContextNoScene:
            scene = None
            active_object = None

        assert op.execute(MockContextNoScene()) == {"CANCELLED"}
        assert LINKFORGE_OT_delete_joint().execute(MockContextNoScene()) == {"CANCELLED"}
        assert LINKFORGE_OT_auto_detect_parent_child().execute(MockContextNoScene()) == {
            "CANCELLED"
        }
        assert not LINKFORGE_OT_auto_detect_parent_child.poll(MockContextNoScene())

        # Test delete_joint when scene is None but active_object is valid
        joint_obj = create_robot_joint("test_joint", None, None, scene)

        class MockContextNoSceneWithActive:
            scene = None
            active_object = joint_obj

        assert LINKFORGE_OT_delete_joint().execute(MockContextNoSceneWithActive()) == {"FINISHED"}
        assert LINKFORGE_OT_auto_detect_parent_child().execute(MockContextNoSceneWithActive()) == {
            "CANCELLED"
        }

    def test_delete_joint_non_matching(self, scene, blender_context) -> None:
        """Test delete joint when the joint is not registered in ROS2 control."""
        joint_obj = create_robot_joint("test_joint_not_registered", None, None, scene)
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = joint_obj
        joint_obj.select_set(True)

        res = LINKFORGE_OT_delete_joint().execute(bpy.context)
        assert res == {"FINISHED"}

    def test_create_joint_empty_users_collection(self, scene, blender_context) -> None:
        """Test create joint when the parent link is not in any collections."""
        link = create_robot_link("base", scene)
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = link
        link.select_set(True)

        link.users_collection = []
        op = LINKFORGE_OT_create_joint()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

    def test_create_joint_get_joint_props_none(self, scene, blender_context) -> None:
        """Test create joint when get_joint_props returns None."""
        link = create_robot_link("base", scene)
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = link
        link.select_set(True)

        op = LINKFORGE_OT_create_joint()
        with patch("linkforge.blender.operators.joint_ops.get_joint_props", return_value=None):
            res = op.execute(bpy.context)
            assert res == {"FINISHED"}

    def test_create_joint_mesh_parent(self, scene, blender_context) -> None:
        """Test create joint when active object is a visual mesh parented to a link."""
        link = create_robot_link("base", scene)
        mesh_obj = create_test_object("visual_mesh", None, scene)
        mesh_obj.parent = link

        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = mesh_obj
        mesh_obj.select_set(True)

        # Check poll passes
        op = LINKFORGE_OT_create_joint
        assert op.poll(bpy.context)

        # Check execute succeeds and sets parent-child link appropriately
        res = op().execute(bpy.context)
        assert res == {"FINISHED"}

        # Verify joint was created and has base link as child_link
        joint = bpy.context.active_object
        assert joint is not None
        assert safe_get_joint(joint).child_link == link

    def test_create_joint_missing_active_object(self, scene, blender_context) -> None:
        """Test create joint with invalid/missing active object cancels."""
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = None

        op = LINKFORGE_OT_create_joint()
        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}

    def test_create_joint_empty_fails(self, mocker, scene, blender_context) -> None:
        """Test create joint cancels when empty addition fails."""
        link = create_robot_link("base", scene)
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = link
        link.select_set(True)

        op = LINKFORGE_OT_create_joint()
        from unittest.mock import PropertyMock

        original_execute = op.execute

        def mock_execute(context):
            mock_active = mocker.patch.object(
                type(bpy.context), "active_object", new_callable=PropertyMock
            )
            # Use stateful side effect: first call returns link, subsequent calls return None
            calls = 0

            def side_effect_fn(*args, **kwargs):
                nonlocal calls
                calls += 1
                if calls == 1:
                    return link
                return None

            mock_active.side_effect = side_effect_fn

            return original_execute(bpy.context)

        res = mock_execute(bpy.context)
        assert res == {"CANCELLED"}

    def test_auto_detect_parent_child_smart_logic(self, scene, blender_context) -> None:
        """Test smart choice selection and fallback scenarios in auto-detect."""
        base = create_robot_link("base", scene)
        child = create_robot_link("child", scene)
        joint_obj = create_robot_joint("test_joint", None, None, scene)

        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = joint_obj
        joint_obj.select_set(True)

        jp = safe_get_joint(joint_obj)

        # 1. Child is already set to base -> parent should auto-detect as child
        jp.child_link = base
        res = LINKFORGE_OT_auto_detect_parent_child().execute(bpy.context)
        assert res == {"FINISHED"}
        assert jp.parent_link == child

        # 2. Child is already set to child -> parent should auto-detect as base
        jp.child_link = child
        jp.parent_link = None
        res = LINKFORGE_OT_auto_detect_parent_child().execute(bpy.context)
        assert res == {"FINISHED"}
        assert jp.parent_link == base

        # 3. Only 1 link in the scene -> should set child to that link, parent to None
        bpy.data.objects.remove(child, do_unlink=True)
        jp.child_link = None
        jp.parent_link = None
        res = LINKFORGE_OT_auto_detect_parent_child().execute(bpy.context)
        assert res == {"FINISHED"}
        assert jp.child_link == base
        assert jp.parent_link is None

    def test_auto_detect_parent_child_zero_links(self, scene, blender_context) -> None:
        """Test auto-detect when there are 0 links in the scene (len(links) == 0)."""
        joint_obj = create_robot_joint("test_joint", None, None, scene)
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = joint_obj
        joint_obj.select_set(True)

        # Remove base link as well to have 0 links
        for obj in list(scene.objects):
            if obj != joint_obj:
                bpy.data.objects.remove(obj, do_unlink=True)

        res = LINKFORGE_OT_auto_detect_parent_child().execute(bpy.context)
        # Should cancel since there are no links in the scene
        assert res == {"CANCELLED"}

    def test_auto_detect_parent_child_view_layer_none(self, scene, blender_context) -> None:
        """Test auto-detect when view_layer is None."""
        base = create_robot_link("base", scene)
        joint_obj = create_robot_joint("test_joint", None, None, scene)

        class CustomContext:
            def __init__(self) -> None:
                self.scene = scene
                self.active_object = joint_obj
                self.view_layer = None

        res = LINKFORGE_OT_auto_detect_parent_child().execute(CustomContext())
        assert res == {"FINISHED"}

    def test_auto_detect_parent_child_exception(self, mocker, scene, blender_context) -> None:
        """Test auto-detect when setting properties raises an exception."""
        base = create_robot_link("base", scene)
        joint_obj = create_robot_joint("test_joint", None, None, scene)
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = joint_obj
        joint_obj.select_set(True)

        jp = safe_get_joint(joint_obj)

        # Override __setattr__ on the specific instance to raise an exception
        def mock_setattr(self, name, value):
            if name in ("child_link", "parent_link"):
                raise AttributeError("Simulated write error")
            super(type(jp), self).__setattr__(name, value)

        mocker.patch.object(type(jp), "__setattr__", mock_setattr)

        # Execute should still return finished (it logs warning and returns FINISHED)
        res = LINKFORGE_OT_auto_detect_parent_child().execute(bpy.context)
        assert res == {"FINISHED"}

    def test_auto_detect_parent_child_poll_not_selected(self, scene, blender_context) -> None:
        """Test poll returns False when joint object is active but not selected."""
        joint_obj = create_robot_joint("test_joint", None, None, scene)
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = joint_obj
        joint_obj.select_set(False)

        assert not LINKFORGE_OT_auto_detect_parent_child.poll(bpy.context)

    def test_create_joint_mesh_no_parent_link(self, scene, blender_context) -> None:
        """Test create joint when active object is not a link and has no parent link (69->72)."""
        mesh_obj = create_test_object("non_link_mesh", None, scene)
        assert bpy.context.view_layer is not None
        bpy.context.view_layer.objects.active = mesh_obj
        mesh_obj.select_set(True)

        res = LINKFORGE_OT_create_joint().execute(bpy.context)
        assert res == {"CANCELLED"}

    def test_joint_ops_registration_and_main(self, mocker) -> None:
        """Verify registration and unregistration loops including double-registration."""
        import linkforge.blender.operators.joint_ops as joint_ops

        # Test unregister
        joint_ops.unregister()

        # Test register with ValueError fallback
        mock_reg = mocker.patch(
            "bpy.utils.register_class", side_effect=[ValueError("Already registered")] + [None] * 10
        )
        mock_unreg = mocker.patch("bpy.utils.unregister_class")
        joint_ops.register()
        assert mock_reg.call_count > 0
        assert mock_unreg.call_count > 0

        # Run __main__ entrypoint via compile + exec to trigger coverage for line 303
        with open(joint_ops.__file__) as f:
            code = compile(f.read(), joint_ops.__file__, "exec")

        global_dict = {
            "__name__": "__main__",
            "__package__": "linkforge.blender.operators",
            "__file__": joint_ops.__file__,
            "bpy": bpy,
        }
        exec(code, global_dict)


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
