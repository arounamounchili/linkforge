"""Unit tests for Blender Properties, Validation, and Preferences."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.preferences import (
    LinkForgePreferences,
    get_addon_id,
    get_addon_prefs,
    update_inertia_size,
    update_inertia_visibility,
    update_joint_empty_size,
    update_link_empty_size,
    update_sensor_empty_size,
)
from linkforge.blender.preferences import (
    register as prefs_register,
)
from linkforge.blender.preferences import (
    unregister as prefs_unregister,
)
from linkforge.blender.utils.property_helpers import (
    find_property_owner,
    get_joint_props,
    get_link_props,
    get_robot_props,
    get_sensor_props,
)

from tests.blender_test_utils import (
    create_robot_joint,
    create_robot_link,
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_sensor,
    safe_get_validation,
)

# Property Helpers


class TestPropertyHelpers:
    def test_find_property_owner(self, scene, blender_context) -> None:
        """Test finding the owner object of a PropertyGroup."""
        obj = create_test_object("test_owner", None, scene)
        props = safe_get_linkforge(obj)

        owner = find_property_owner(bpy.context, props, "linkforge")
        assert owner == obj

    def test_get_props_edge_cases(self, scene, blender_context) -> None:
        """Test edge cases for get_*_props helpers."""
        assert get_joint_props(None) is None
        assert get_link_props(None) is None
        assert get_sensor_props(None) is None
        assert get_robot_props(None) is None


# Validation Properties


class TestValidationProperties:
    def test_validation_issue_line_splitting(self, scene, blender_context) -> None:
        """Test correctly splitting long messages and suggestions into lines."""
        wm = bpy.context.window_manager
        res = safe_get_validation(wm)
        res.clear()

        err = res.errors.add()
        err.message = "This is a very long message that should be split into multiple lines."

        # Verify splitting logic (assuming 60 chars limit)
        lines = err.message_lines
        assert len(lines) >= 1
        for line in lines:
            assert len(line) <= 60

    def test_validation_result_clearing(self, scene, blender_context) -> None:
        """Test clearing validation results."""
        wm = bpy.context.window_manager
        res = safe_get_validation(wm)
        res.has_results = True
        res.clear()
        assert res.has_results is False

    def test_validation_issue_properties(self, scene, blender_context) -> None:
        """Test the properties and helper methods of ValidationIssueProperty and ValidationResultProperty."""
        wm = bpy.context.window_manager
        res = safe_get_validation(wm)
        res.clear()

        # Add error
        err = res.errors.add()
        err.title = "Test Error"
        err.message = "Error message"
        err.suggestion = "Do this to fix it"
        err.affected_objects = "obj1,obj2"

        assert err.has_suggestion is True
        assert err.has_objects is True
        assert err.objects_str == "obj1,obj2"
        assert len(err.suggestion_lines) >= 1

        # Add warning
        warn = res.warnings.add()
        warn.title = "Test Warning"
        warn.message = "Warning message"
        warn.suggestion = ""
        warn.affected_objects = ""

        assert warn.has_suggestion is False
        assert warn.has_objects is False
        assert warn.objects_str == ""
        assert warn.suggestion_lines == []

        # Test index getters
        res.error_count = 1
        res.warning_count = 1
        assert res.get_error(0) == err
        assert res.get_warning(0) == warn

    def test_validation_properties_registration(self) -> None:
        """Test register and unregister functions for validation properties."""
        from linkforge.blender.properties.validation_props import (
            register as val_register,
        )
        from linkforge.blender.properties.validation_props import (
            unregister as val_unregister,
        )

        with (
            patch("bpy.utils.register_class") as mock_reg,
            patch("bpy.utils.unregister_class") as mock_unreg,
        ):
            val_register()
            assert mock_reg.called

            val_unregister()
            assert mock_unreg.called


# Addon Preferences


class TestPreferences:
    def test_update_joint_empty_size(self, scene, blender_context) -> None:
        """Test that updating joint size in prefs affects scene objects."""
        obj = create_test_object("test_joint_size", None, scene)

        # Ensure we are testing the linkforge joint props
        safe_get_joint(obj).is_robot_joint = True
        obj.empty_display_size = 0.1

        mock_prefs = MagicMock()
        mock_prefs.joint_empty_size = 0.5

        with patch("linkforge.blender.visualization.joint_gizmos.update_viz_handle"):
            update_joint_empty_size(mock_prefs, bpy.context)

        assert obj.empty_display_size == pytest.approx(0.5)

    def test_update_sensor_empty_size(self, scene, blender_context) -> None:
        """Test that updating sensor empty size in prefs affects scene objects."""
        obj = create_test_object("test_sensor_size", None, scene)
        safe_get_sensor(obj).is_robot_sensor = True
        obj.empty_display_size = 0.1

        mock_prefs = MagicMock()
        mock_prefs.sensor_empty_size = 0.6

        update_sensor_empty_size(mock_prefs, bpy.context)
        assert obj.empty_display_size == pytest.approx(0.6)

    def test_update_link_empty_size(self, scene, blender_context) -> None:
        """Test that updating link empty size in prefs affects scene objects."""
        obj = create_test_object("test_link_size", None, scene)
        safe_get_linkforge(obj).is_robot_link = True
        obj.empty_display_size = 0.1

        mock_prefs = MagicMock()
        mock_prefs.link_empty_size = 0.7

        update_link_empty_size(mock_prefs, bpy.context)
        assert obj.empty_display_size == pytest.approx(0.7)

    def test_update_inertia_visibility_and_size(self, scene, blender_context) -> None:
        """Test that updating inertia visibility and size tags redraw."""
        mock_prefs = MagicMock()
        mock_prefs.show_inertia_gizmos = True

        with (
            patch("linkforge.blender.visualization.inertia_gizmos.tag_redraw") as mock_redraw,
            patch(
                "linkforge.blender.visualization.inertia_gizmos.ensure_inertia_handler"
            ) as mock_ensure,
        ):
            update_inertia_visibility(mock_prefs, bpy.context)
            mock_redraw.assert_called_once()
            mock_ensure.assert_called_once()

        with patch("linkforge.blender.visualization.inertia_gizmos.tag_redraw") as mock_redraw:
            update_inertia_size(mock_prefs, bpy.context)
            mock_redraw.assert_called_once()

    def test_get_addon_id_and_prefs(self, scene, blender_context) -> None:
        """Test resolving addon ID and retrieving preferences."""
        addon_id = get_addon_id()
        assert addon_id == "linkforge"
        assert bpy.context.preferences is not None

        # Mock context.preferences.addons.get to return None so get_addon_prefs returns None
        with patch.object(bpy.context.preferences.addons, "get", return_value=None):
            prefs = get_addon_prefs(bpy.context)
            assert prefs is None

        # Mock context.preferences.addons.get to return an addon with preferences
        mock_addon = MagicMock()
        mock_prefs = LinkForgePreferences()
        mock_addon.preferences = mock_prefs
        with patch.object(bpy.context.preferences.addons, "get", return_value=mock_addon):
            prefs = get_addon_prefs(bpy.context)
            assert prefs == mock_prefs

    def test_preferences_draw(self, scene, blender_context) -> None:
        """Test drawing the preferences layout."""
        prefs = LinkForgePreferences()
        prefs.layout = MagicMock()
        prefs.show_inertia_gizmos = True
        prefs.show_joint_axes = True

        # Call draw, shouldn't raise any exception
        prefs.draw(bpy.context)
        assert prefs.layout.box.called

    def test_preferences_registration(self) -> None:
        """Test register and unregister functions for preferences."""
        with (
            patch("bpy.utils.register_class") as mock_reg,
            patch("bpy.utils.unregister_class") as mock_unreg,
        ):
            prefs_register()
            assert mock_reg.called

            prefs_unregister()
            assert mock_unreg.called


class TestGlobalPropertiesAndCallbacks:
    def test_properties_global_registration(self) -> None:
        """Test calling global register and unregister from linkforge.blender.properties."""
        from linkforge.blender.properties import (
            register as props_register,
        )
        from linkforge.blender.properties import (
            unregister as props_unregister,
        )

        with (
            patch("bpy.utils.register_class") as mock_reg,
            patch("bpy.utils.unregister_class") as mock_unreg,
        ):
            props_register()
            assert mock_reg.called

            props_unregister()
            assert mock_unreg.called

    def test_property_helpers_strategies(self, scene, blender_context) -> None:
        """Test find_property_owner strategy fallbacks and get_transmission_props."""
        from linkforge.blender.constants import PROP_LINK
        from linkforge.blender.utils.property_helpers import (
            find_property_owner,
            get_transmission_props,
        )

        obj1 = create_test_object("test_strat_3", None, scene)
        props = safe_get_linkforge(obj1)

        # Mock Context with selected_objects for Strategy 3
        mock_ctx = MagicMock()
        mock_ctx.object = None
        mock_ctx.selected_objects = [obj1]

        owner = find_property_owner(mock_ctx, props, PROP_LINK)
        assert owner == obj1

        # Mock Context with scene for Strategy 4
        mock_ctx_scene = MagicMock()
        mock_ctx_scene.object = None
        mock_ctx_scene.selected_objects = []
        mock_ctx_scene.scene = scene

        owner_scene = find_property_owner(mock_ctx_scene, props, PROP_LINK)
        assert owner_scene == obj1

        # Test get_transmission_props helper
        assert get_transmission_props(None) is None
        assert get_transmission_props(obj1) is not None

    def test_joint_properties_and_callbacks(self, scene, blender_context) -> None:
        """Test getters, setters, polls and hierarchy updates in JointPropertyGroup."""
        from linkforge.blender.properties.joint_props import (
            get_joint_name,
            poll_robot_joint,
            poll_robot_link,
            set_joint_name,
            update_joint_hierarchy,
        )

        # Create base/child links
        base = create_robot_link("base_link", scene)
        child = create_robot_link("child_link", scene)
        joint_obj = create_robot_joint("test_joint", base, child, scene)

        jp = safe_get_joint(joint_obj)

        # Test getters and setters
        assert get_joint_name(jp) == "test_joint"
        set_joint_name(jp, "renamed_joint")
        assert jp.source_name_stored == "renamed_joint"

        # Test deferring renamed name set when read-only in depsgraph
        with (
            patch("bpy.app.background", False),
            patch("bpy.app.timers") as mock_timers,
        ):

            class ReadOnlyNameObj:
                def __init__(self) -> None:
                    self._name = "test_joint"

                @property
                def name(self) -> str:
                    return self._name

                @name.setter
                def name(self, value: str) -> None:
                    raise AttributeError("Read-only")

            fake_obj = ReadOnlyNameObj()
            jp.id_data = fake_obj
            set_joint_name(jp, "deferred_joint")
            assert mock_timers.register.called

        # Restore original id_data
        jp.id_data = joint_obj

        # Test poll filters
        assert poll_robot_link(jp, base) is True
        assert poll_robot_link(jp, joint_obj) is False
        assert poll_robot_joint(jp, joint_obj) is False  # self-mimicry prevention

        # Test hierarchy update when clearing parents
        jp.parent_link = None
        jp.child_link = None
        update_joint_hierarchy(jp, bpy.context)
        assert joint_obj.parent is None

    def test_transmission_properties_and_callbacks(self, scene, blender_context) -> None:
        """Test getters, setters, polls and hierarchy updates in TransmissionPropertyGroup."""
        from linkforge.blender.constants import PROP_TRANSMISSION
        from linkforge.blender.properties.transmission_props import (
            get_transmission_name,
            poll_robot_joint,
            set_transmission_name,
            update_transmission_hierarchy,
        )
        from linkforge.blender.properties.transmission_props import (
            register as trans_register,
        )
        from linkforge.blender.properties.transmission_props import (
            unregister as trans_unregister,
        )
        from linkforge.core.constants import TRANS_DIFFERENTIAL

        base = create_robot_link("base_link", scene)
        child = create_robot_link("child_link", scene)
        joint_obj = create_robot_joint("test_joint", base, child, scene)

        trans_obj = create_test_object("test_trans", None, scene)
        tp = getattr(trans_obj, PROP_TRANSMISSION)
        tp.is_robot_transmission = True

        # Test getters and setters
        assert get_transmission_name(tp) == "test_trans"
        set_transmission_name(tp, "renamed_trans")
        assert tp.source_name_stored == "renamed_trans"

        # Test poll filters
        assert poll_robot_joint(tp, joint_obj) is True
        assert poll_robot_joint(tp, base) is False

        # Test hierarchy update
        tp.joint_name = joint_obj
        update_transmission_hierarchy(tp, bpy.context)
        assert trans_obj.parent == joint_obj

        # Test differential transmission type hierarchy update
        tp.transmission_type = TRANS_DIFFERENTIAL
        tp.joint1_name = joint_obj
        update_transmission_hierarchy(tp, bpy.context)
        assert trans_obj.parent == joint_obj

        # Test clean unregister/register
        with (
            patch("bpy.utils.register_class") as mock_reg,
            patch("bpy.utils.unregister_class") as mock_unreg,
        ):
            trans_register()
            assert mock_reg.called

            trans_unregister()
            assert mock_unreg.called
