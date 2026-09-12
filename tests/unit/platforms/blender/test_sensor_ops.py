"""Unit tests for Blender Sensor operators."""

from __future__ import annotations

from unittest.mock import patch

import bpy
from linkforge.blender.operators import sensor_ops
from linkforge.blender.operators.sensor_ops import (
    LINKFORGE_OT_create_sensor,
    LINKFORGE_OT_delete_sensor,
)

from tests.blender_test_utils import (
    create_mesh_object,
    create_robot_link,
    create_test_object,
    safe_get_sensor,
)


class TestSensorOperators:
    def test_create_sensor_operator_poll(self, mocker, scene, blender_context) -> None:
        """Test poll method of create sensor operator."""
        assert bpy.context.view_layer is not None
        op = LINKFORGE_OT_create_sensor

        # Active object is None
        bpy.context.view_layer.objects.active = None
        assert not op.poll(bpy.context)

        # Active object exists but not selected
        link = create_robot_link("base", scene)
        link.select_set(False)
        bpy.context.view_layer.objects.active = link
        assert not op.poll(bpy.context)

        # Selected and is link
        link.select_set(True)
        bpy.context.view_layer.objects.active = link
        assert op.poll(bpy.context)

        # Selected and is child of a link
        child = create_test_object("child_mesh", None, scene)
        child.parent = link
        child.select_set(True)
        bpy.context.view_layer.objects.active = child
        assert op.poll(bpy.context)

    def test_create_sensor_operator_execute(self, mocker, scene, blender_context) -> None:
        """Test execute method of create sensor operator."""
        assert bpy.context.view_layer is not None
        link = create_robot_link("base", scene)
        bpy.context.view_layer.objects.active = link

        op = LINKFORGE_OT_create_sensor()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        # Find created sensor empty
        sensor_obj = bpy.context.active_object
        assert sensor_obj is not None
        assert sensor_obj.name.startswith("base_sensor")
        assert sensor_obj.parent == link
        assert safe_get_sensor(sensor_obj).is_robot_sensor
        assert safe_get_sensor(sensor_obj).attached_link == link

    def test_create_sensor_operator_execute_child(self, mocker, scene, blender_context) -> None:
        """Test execute method when active object is a child of a link."""
        assert bpy.context.view_layer is not None
        link = create_robot_link("base", scene)
        child = create_test_object("child_mesh", None, scene)
        child.parent = link
        bpy.context.view_layer.objects.active = child

        op = LINKFORGE_OT_create_sensor()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        sensor_obj = bpy.context.active_object
        assert sensor_obj is not None
        assert sensor_obj.parent == link
        assert safe_get_sensor(sensor_obj).attached_link == link

    def test_create_sensor_operator_fallback(self, mocker, scene, blender_context) -> None:
        """Test execute fallback when preferences are missing."""
        assert bpy.context.view_layer is not None
        link = create_robot_link("base", scene)
        bpy.context.view_layer.objects.active = link

        with patch("linkforge.blender.preferences.get_addon_prefs", return_value=None):
            op = LINKFORGE_OT_create_sensor()
            res = op.execute(bpy.context)
            assert res == {"FINISHED"}

    def test_delete_sensor_operator(self, mocker, scene, blender_context) -> None:
        """Test delete sensor operator poll and execute."""
        assert bpy.context.view_layer is not None
        link = create_robot_link("base", scene)
        sensor_obj = create_test_object("test_sensor", None, scene)
        sensor_obj.parent = link
        safe_get_sensor(sensor_obj).is_robot_sensor = True

        op = LINKFORGE_OT_delete_sensor

        # Poll fails when active object is not selected
        sensor_obj.select_set(False)
        bpy.context.view_layer.objects.active = sensor_obj
        assert not op.poll(bpy.context)

        # Poll passes when selected
        sensor_obj.select_set(True)
        assert op.poll(bpy.context)

        res = op().execute(bpy.context)
        assert res == {"FINISHED"}
        assert sensor_obj.name not in scene.objects

    def test_sensor_ops_invalid_context(self) -> None:
        """Verify sensor operator handles invalid context gracefully."""
        op = LINKFORGE_OT_create_sensor()

        class MockContextNoScene:
            scene = None

        assert op.execute(MockContextNoScene()) == {"CANCELLED"}

    def test_create_sensor_no_link_resolved(self, scene, blender_context) -> None:
        """Execute cancels if the active object has no link props and no parent link."""
        loose = create_mesh_object("unrelated", scene)
        bpy.context.view_layer.objects.active = loose
        loose.select_set(True)

        op = LINKFORGE_OT_create_sensor()
        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}

    def test_delete_sensor_no_active_object(self, blender_context) -> None:
        """Delete sensor returns CANCELLED with no active object."""
        bpy.context.view_layer.objects.active = None

        op = LINKFORGE_OT_delete_sensor()
        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}

    def test_delete_sensor_poll_none_active(self, blender_context) -> None:
        """Poll returns False with no active object."""
        bpy.context.view_layer.objects.active = None
        assert not LINKFORGE_OT_delete_sensor.poll(bpy.context)

    def test_create_sensor_parent_not_robot_link(self, scene, blender_context) -> None:
        """Create sensor cancels if child has link props but neither child nor parent is robot link."""
        parent_mesh = create_mesh_object("p_mesh", scene)
        child_mesh = create_mesh_object("c_mesh", scene)
        child_mesh.parent = parent_mesh
        # child has linkforge property but is_robot_link is False
        bpy.context.view_layer.objects.active = child_mesh

        op = LINKFORGE_OT_create_sensor()
        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}

    def test_sensor_ops_register_unregister(self, monkeypatch) -> None:
        """Test register with ValueError retry and unregister."""
        sensor_ops.register()
        sensor_ops.unregister()

        # Simulate ValueError during register to exercise the except branch
        first_call = True
        orig_register = bpy.utils.register_class

        def mock_register(cls):
            nonlocal first_call
            if first_call:
                first_call = False
                raise ValueError("Already registered")
            orig_register(cls)

        monkeypatch.setattr(bpy.utils, "register_class", mock_register)
        sensor_ops.register()
