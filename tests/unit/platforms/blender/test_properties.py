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

        # Mock Context with selected_objects for Strategy 3 (original test pattern)
        original_id_data = getattr(props, "id_data", None)
        try:
            # Clear id_data to bypass Strategy 1 and test fallbacks
            props.id_data = None

            mock_ctx = MagicMock()
            mock_ctx.object = None
            mock_ctx.selected_objects = [obj1]

            owner = find_property_owner(mock_ctx, props, PROP_LINK)
            assert owner == obj1

            # Strategy 2: Active Object check
            mock_ctx_obj = MagicMock()
            mock_ctx_obj.object = obj1
            owner_obj = find_property_owner(mock_ctx_obj, props, PROP_LINK)
            assert owner_obj == obj1

            # Mock Context with scene for Strategy 4
            mock_ctx_scene = MagicMock()
            mock_ctx_scene.object = None
            mock_ctx_scene.selected_objects = []
            mock_ctx_scene.scene = scene

            owner_scene = find_property_owner(mock_ctx_scene, props, PROP_LINK)
            assert owner_scene == obj1

            # Strategy 4 Fallback to None
            mock_ctx_none = MagicMock()
            mock_ctx_none.object = None
            mock_ctx_none.selected_objects = []
            mock_ctx_none.scene = MagicMock()
            mock_ctx_none.scene.objects = []
            owner_none = find_property_owner(mock_ctx_none, props, PROP_LINK)
            assert owner_none is None
        finally:
            props.id_data = original_id_data

        # Test get_transmission_props helper
        assert get_transmission_props(None) is None
        assert get_transmission_props(obj1) is not None

        # Test get_robot_props helper with a valid scene
        assert get_robot_props(scene) is not None

        # Clear id_data for fallback testing
        original_id_data = getattr(props, "id_data", None)
        try:
            props.id_data = None

            # Test selected_objects loop fall-through (empty list)
            mock_ctx_empty_sel = MagicMock()
            mock_ctx_empty_sel.object = None
            mock_ctx_empty_sel.selected_objects = []
            mock_ctx_empty_sel.scene = None
            assert find_property_owner(mock_ctx_empty_sel, props, PROP_LINK) is None

            # Test selected_objects loop continuation with non-matching object
            non_matching_obj = create_test_object("non_match", None, scene)
            mock_ctx_non_match_sel = MagicMock()
            mock_ctx_non_match_sel.object = None
            mock_ctx_non_match_sel.selected_objects = [non_matching_obj]
            mock_ctx_non_match_sel.scene = None
            assert find_property_owner(mock_ctx_non_match_sel, props, PROP_LINK) is None

            # Test scene.objects loop continuation and fall-through
            mock_ctx_non_match_scene = MagicMock()
            mock_ctx_non_match_scene.object = None
            mock_ctx_non_match_scene.selected_objects = []
            mock_ctx_non_match_scene.scene = MagicMock()
            mock_ctx_non_match_scene.scene.objects = [non_matching_obj]
            assert find_property_owner(mock_ctx_non_match_scene, props, PROP_LINK) is None

            # Test context without selected_objects attribute (covers 60->66 branch)
            mock_ctx_no_sel = MagicMock()
            del mock_ctx_no_sel.selected_objects
            mock_ctx_no_sel.object = None
            mock_ctx_no_sel.scene = None
            assert find_property_owner(mock_ctx_no_sel, props, PROP_LINK) is None
        finally:
            props.id_data = original_id_data

    def test_duplicate_registrations(self) -> None:
        """Test duplicate registration handling and main blocks in property groups."""
        import runpy

        from linkforge.blender.properties import (
            control_props,
            joint_props,
            link_props,
            robot_props,
            sensor_props,
            transmission_props,
            validation_props,
        )

        modules = [
            control_props,
            validation_props,
            link_props,
            joint_props,
            sensor_props,
            transmission_props,
            robot_props,
        ]

        # 1. Test standard register/unregister cycle
        for mod in modules:
            mod.register()
            mod.unregister()
            mod.register()

        # 2. Test ValueError fallback handling in register()
        # Mock register_class to raise ValueError on first call for each class, then succeed on retry
        for mod in modules:
            orig_register_class = bpy.utils.register_class
            try:
                failed_classes = set()

                def mock_register_class(cls, failed=failed_classes, orig=orig_register_class):
                    if cls not in failed:
                        failed.add(cls)
                        raise ValueError("Mock registration failure")
                    return orig(cls)

                bpy.utils.register_class = mock_register_class
                mod.register()
            finally:
                bpy.utils.register_class = orig_register_class

        # 3. Test execution under __main__ namespace using runpy
        for mod_name in [
            "linkforge.blender.properties.control_props",
            "linkforge.blender.properties.validation_props",
            "linkforge.blender.properties.link_props",
            "linkforge.blender.properties.joint_props",
            "linkforge.blender.properties.sensor_props",
            "linkforge.blender.properties.transmission_props",
            "linkforge.blender.properties.robot_props",
        ]:
            runpy.run_module(mod_name, run_name="__main__")

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

    def test_validation_props_extra_coverage(self, scene) -> None:
        """Test validation_props properties and methods for 100% coverage."""
        from linkforge.blender.constants import PROP_VALIDATION

        # Get validation properties stored on window manager
        wm = bpy.context.window_manager
        vp = getattr(wm, PROP_VALIDATION)
        vp.clear()

        # Add an error
        err = vp.errors.add()
        err.message = "This is a very long error message that will definitely exceed the standard fifty eight character limit to force a wrapped line."
        err.suggestion = "This is a very long suggestion that will definitely exceed the standard fifty eight character limit to force a wrapped line."

        # Retrieve suggesting / message lines to verify they wrap
        assert len(err.message_lines) > 1
        assert len(err.suggestion_lines) > 1

        # Test fallback with spacing-only inputs to get_error / get_warning / message_lines
        err2 = vp.errors.add()
        err2.message = "    "
        err2.suggestion = "    "
        assert len(err2.message_lines) == 0
        assert len(err2.suggestion_lines) == 0

        assert vp.get_error(0) == err

        warn = vp.warnings.add()
        assert vp.get_warning(0) == warn

    def test_joint_props_extra_coverage(self, scene) -> None:
        """Test joint_props edge cases for 100% coverage."""
        from linkforge.blender.properties.joint_props import (
            get_joint_name,
            poll_robot_joint,
            set_joint_name,
            update_joint_hierarchy,
        )

        # Create mock joint property group without id_data
        class MockJointProps:
            id_data = None
            source_name_stored = ""
            is_robot_joint = True

        mjp = MockJointProps()
        assert get_joint_name(mjp) == ""

        # Null value set name
        set_joint_name(mjp, "")
        set_joint_name(mjp, "test")  # When id_data is None

        # Test deferred rename callback execution in GUI mode
        bpy.app.background = False

        class ReadOnlyNameObj:
            def __init__(self) -> None:
                self._name = "old"

            @property
            def name(self) -> str:
                return self._name

            @name.setter
            def name(self, value: str) -> None:
                raise AttributeError("Read-only")

        ro_obj = ReadOnlyNameObj()
        mjp.id_data = ro_obj
        set_joint_name(mjp, "new_val")
        # Run all pending timers
        bpy.app.timers.run_all()

        # Trigger Exception path in callback
        mjp.id_data = ro_obj
        set_joint_name(mjp, "exc_new")

        class ExceptionObj:
            @property
            def name(self) -> str:
                return "exc"

            @name.setter
            def name(self, val: str) -> None:
                raise ValueError("Fail assignment")

        mjp.id_data = ExceptionObj()
        bpy.app.timers.run_all()

        # Trigger None id_data path in callback
        mjp.id_data = ro_obj
        set_joint_name(mjp, "exc_new")
        mjp.id_data = None
        bpy.app.timers.run_all()

        # Background mode name queue sync
        from linkforge.blender.handlers.name_sync_handler import PENDING_RENAMES

        orig_len = len(PENDING_RENAMES)
        bpy.app.background = True
        mjp.id_data = ro_obj
        set_joint_name(mjp, "bg_val")
        assert len(PENDING_RENAMES) == orig_len + 1

        # update_joint_hierarchy early return if bpy is None
        with patch("linkforge.blender.properties.joint_props.bpy", None):
            assert update_joint_hierarchy(mjp, None) is None

        # update_joint_hierarchy with missing context/scene or joint_obj None
        assert update_joint_hierarchy(mjp, MagicMock()) is None

        # update_joint_hierarchy child/parent unparenting loop
        base = create_robot_link("base_link", scene)
        child = create_robot_link("child_link", scene)
        joint_obj = create_robot_joint("test_joint", base, child, scene)

        # Setup another object parented to joint_obj to test clear child unparenting
        other_child = create_test_object("other_child_collision", None, scene)
        other_child.parent = joint_obj
        # Ensure it has link properties so it matches criteria
        lp = safe_get_linkforge(other_child)
        lp.is_robot_link = True

        jp = safe_get_joint(joint_obj)
        jp.id_data = joint_obj
        jp.child_link = None  # Trigger clearing parent/child logic
        update_joint_hierarchy(jp, bpy.context)
        # Ensure other_child was unparented
        assert other_child.parent is None

        # update_joint_hierarchy with missing scene inside context
        mock_ctx_no_scene = MagicMock()
        mock_ctx_no_scene.scene = None
        update_joint_hierarchy(jp, mock_ctx_no_scene)

        # poll_robot_joint checks
        assert poll_robot_joint(jp, None) is False
        non_empty = create_test_object("non_empty", None, scene)
        non_empty.type = "MESH"
        assert poll_robot_joint(jp, non_empty) is False

        non_empty_empty = create_test_object("non_empty_empty", None, scene)
        non_empty_empty.type = "EMPTY"
        # Without joint props
        with patch("linkforge.blender.utils.property_helpers.get_joint_props", return_value=None):
            assert poll_robot_joint(jp, non_empty_empty) is False

    def test_link_props_extra_coverage(self, scene) -> None:
        """Test link_props edge cases for 100% coverage."""
        from linkforge.blender.properties.link_props import (
            get_kd_scientific,
            get_kp_scientific,
            get_link_name,
            on_collision_quality_update,
            set_kd_scientific,
            set_kp_scientific,
            set_link_name,
            update_auto_inertia_toggle,
            update_inertia_viz,
        )

        class MockLinkProps:
            id_data = None
            source_name_stored = ""
            use_auto_inertia = True

        mlp = MockLinkProps()
        assert get_link_name(mlp) == ""

        # Null value set name
        set_link_name(mlp, "")

        # Test scientific notation get/set methods
        link_obj = create_test_object("real_link_props", None, scene)
        lp = safe_get_linkforge(link_obj)
        lp.id_data = link_obj
        set_kp_scientific(lp, "1e5")
        assert get_kp_scientific(lp) == "1.00e+05"
        set_kd_scientific(lp, "2e3")
        assert get_kd_scientific(lp) == "2.00e+03"

        # Test deferred rename callback execution in GUI mode
        bpy.app.background = False

        class ReadOnlyNameObj:
            def __init__(self) -> None:
                self._name = "old"
                self.children = []

            @property
            def name(self) -> str:
                return self._name

            @name.setter
            def name(self, value: str) -> None:
                raise AttributeError("Read-only")

        ro_obj = ReadOnlyNameObj()
        mlp.id_data = ro_obj
        set_link_name(mlp, "new_val")
        bpy.app.timers.run_all()

        # Trigger Exception path in callback
        mlp.id_data = ro_obj
        set_link_name(mlp, "exc_new")

        class ExceptionObj:
            def __init__(self) -> None:
                self.children = []

            @property
            def name(self) -> str:
                return "exc"

            @name.setter
            def name(self, val: str) -> None:
                raise ValueError("Fail assignment")

        mlp.id_data = ExceptionObj()
        bpy.app.timers.run_all()

        # Trigger None id_data path in callback
        mlp.id_data = ro_obj
        set_link_name(mlp, "exc_new")
        mlp.id_data = None
        bpy.app.timers.run_all()

        # Test child renaming logic in link_props.py (line 147)
        # Clean up any potential conflicting names from prior tests/runs
        for n in [
            "unique_parent_link_x",
            "unique_parent_link_x_visual",
            "unique_new_parent_link",
            "unique_new_parent_link_visual",
        ]:
            if n in bpy.data.objects:
                bpy.data.objects.remove(bpy.data.objects[n], do_unlink=True)

        parent_link = create_test_object("unique_parent_link_x", None, scene)
        child_visual = create_test_object("unique_parent_link_x_visual", None, scene)
        child_visual.parent = parent_link
        # Refresh parent_link children list mock
        parent_link.children = [child_visual]

        lp_parent = safe_get_linkforge(parent_link)
        lp_parent.id_data = parent_link
        lp_parent.source_name_stored = "unique_parent_link_x"
        set_link_name(lp_parent, "unique_new_parent_link")
        assert child_visual.name == "unique_new_parent_link_visual"

        # Background sync queue coverage
        from linkforge.blender.handlers.name_sync_handler import PENDING_RENAMES

        orig_len = len(PENDING_RENAMES)

        class ReadOnlyNameObjSync:
            def __init__(self) -> None:
                self._name = "link_obj"
                self.children = []

            @property
            def name(self) -> str:
                return self._name

            @name.setter
            def name(self, value: str) -> None:
                raise AttributeError("Read-only")

        fake_obj = ReadOnlyNameObjSync()

        class MockLinkProps2:
            id_data = fake_obj
            source_name_stored = ""

        mlp2 = MockLinkProps2()

        with patch("bpy.app.background", True):
            set_link_name(mlp2, "new_name")
            assert len(PENDING_RENAMES) == orig_len + 1

        # Test on_collision_quality_update when self has no id_data
        assert on_collision_quality_update(mlp, None) is None

        # Test on_collision_quality_update when object is not a robot link
        link_obj = create_test_object("not_a_robot_link", None, scene)
        mlp3 = safe_get_linkforge(link_obj)
        mlp3.id_data = link_obj
        mlp3.is_robot_link = False
        on_collision_quality_update(mlp3, None)

        # Test on_collision_quality_update when no collision child exists
        mlp3.is_robot_link = True
        on_collision_quality_update(mlp3, None)

        # Test on_collision_quality_update when collision child has TAG_IMPORTED_SOURCE = True
        collision_child = create_test_object("not_a_robot_link_collision", None, scene)
        collision_child.parent = link_obj
        from linkforge.blender.constants import TAG_IMPORTED_SOURCE

        collision_child[TAG_IMPORTED_SOURCE] = True
        on_collision_quality_update(mlp3, None)

        # Test on_collision_quality_update when collision child has TAG_IMPORTED_SOURCE = False (branch 179->186)
        collision_child[TAG_IMPORTED_SOURCE] = False
        with patch(
            "linkforge.blender.operators.link_ops.update_collision_quality_realtime"
        ) as mock_realtime:
            on_collision_quality_update(mlp3, None)
            assert mock_realtime.called

        # Test on_collision_quality_update without TAG_IMPORTED_SOURCE (triggers regeneration line 181-188)
        clean_link = create_test_object("clean_link", None, scene)
        mlp_clean = safe_get_linkforge(clean_link)
        mlp_clean.id_data = clean_link
        mlp_clean.is_robot_link = True
        clean_collision_child = create_test_object("clean_link_collision", None, scene)
        clean_collision_child.parent = clean_link
        # Delete TAG_IMPORTED_SOURCE if present to raise KeyError
        if TAG_IMPORTED_SOURCE in clean_collision_child:
            del clean_collision_child[TAG_IMPORTED_SOURCE]
        with patch(
            "linkforge.blender.operators.link_ops.update_collision_quality_realtime"
        ) as mock_realtime:
            on_collision_quality_update(mlp_clean, None)
            assert mock_realtime.called

        # Test update_inertia_viz directly
        with patch("linkforge.blender.properties.link_props.tag_redraw") as mock_redraw:
            update_inertia_viz(None, None)
            assert mock_redraw.called

        # Test update_auto_inertia_toggle when object has no use_auto_inertia
        class NonInertiaProps:
            pass

        nip = NonInertiaProps()
        update_auto_inertia_toggle(nip, None)

        # Test update_auto_inertia_toggle calling ensure_inertia_handler when use_auto_inertia is False
        with patch(
            "linkforge.blender.visualization.inertia_gizmos.ensure_inertia_handler"
        ) as mock_handler:
            lp.id_data = link_obj
            lp.use_auto_inertia = False
            update_auto_inertia_toggle(lp, None)
            assert mock_handler.called

        # Test update_auto_inertia_toggle when use_auto_inertia is True (branch 205->exit)
        with patch(
            "linkforge.blender.visualization.inertia_gizmos.ensure_inertia_handler"
        ) as mock_handler:
            lp.use_auto_inertia = True
            update_auto_inertia_toggle(lp, None)
            assert not mock_handler.called

    def test_robot_props_extra_coverage(self, scene) -> None:
        """Test robot_props edge cases for 100% coverage."""
        from linkforge.blender.properties.robot_props import (
            update_collision_visibility,
        )

        # Empty scene or context scene
        assert update_collision_visibility(None, None) is None

        class MockRobotProps:
            show_collisions = True

        mrp = MockRobotProps()

        # Trigger visibility update on a structured object
        link_obj = create_test_object("my_link", None, scene)
        lp = safe_get_linkforge(link_obj)
        lp.id_data = link_obj
        lp.is_robot_link = True

        collision_child = create_test_object("my_link_collision", None, scene)
        collision_child.parent = link_obj

        mock_ctx = MagicMock()
        mock_ctx.scene = scene

        # Enable collisions
        update_collision_visibility(mrp, mock_ctx)
        assert collision_child.hide_viewport is False

        # Disable collisions
        mrp.show_collisions = False
        update_collision_visibility(mrp, mock_ctx)
        assert collision_child.hide_viewport is True

    def test_sensor_props_extra_coverage(self, scene) -> None:
        """Test sensor_props edge cases for 100% coverage."""
        from linkforge.blender.properties.sensor_props import (
            get_sensor_name,
            poll_robot_link,
            set_sensor_name,
            update_sensor_hierarchy,
        )

        class MockSensorProps:
            id_data = None
            source_name_stored = ""
            is_robot_sensor = True
            attached_link = None

        msp = MockSensorProps()
        assert get_sensor_name(msp) == ""

        # Test get_sensor_name when source_name_stored is present
        msp.source_name_stored = "my_stored_sensor"
        assert get_sensor_name(msp) == "my_stored_sensor"

        # Test get_sensor_name when source_name_stored is empty but id_data exists
        class FakeSensorObj:
            def __init__(self) -> None:
                self.name = "my_sensor_obj"

        fso = FakeSensorObj()
        msp.source_name_stored = ""
        msp.id_data = fso
        assert get_sensor_name(msp) == "my_sensor_obj"

        # Test set_sensor_name when name changes
        set_sensor_name(msp, "new_sensor_name")
        assert msp.source_name_stored == "new_sensor_name"
        assert fso.name == "new_sensor_name"

        # Test set_sensor_name when name is already equal (no assignment)
        set_sensor_name(msp, "new_sensor_name")

        # Test set_sensor_name when msp has no id_data or empty value
        msp.id_data = None
        set_sensor_name(msp, "")

        # update_sensor_hierarchy with missing sensor_obj or not robot sensor
        assert update_sensor_hierarchy(msp, MagicMock()) is None

        # update_sensor_hierarchy reparenting when attached link is None but has parent
        sensor_obj = create_test_object("test_sensor", None, scene)
        parent_obj = create_test_object("some_parent", None, scene)
        sensor_obj.parent = parent_obj

        sp = safe_get_sensor(sensor_obj)
        sp.id_data = sensor_obj
        sp.is_robot_sensor = True
        sp.attached_link = None

        update_sensor_hierarchy(sp, bpy.context)
        assert sensor_obj.parent is None

        # update_sensor_hierarchy reparenting when attached link is present (line 130-137)
        link_obj = create_robot_link("attached_link_obj", scene)
        with (
            patch(
                "linkforge.blender.utils.transform_utils.set_parent_keep_transform"
            ) as mock_set_parent,
            patch("linkforge.blender.utils.scene_utils.sync_object_collections") as mock_sync_coll,
        ):
            sp.attached_link = link_obj
            update_sensor_hierarchy(sp, bpy.context)
            assert mock_set_parent.called
            assert mock_sync_coll.called

        # update_sensor_hierarchy when already parented to link_obj (branch 130->135)
        sensor_obj.parent = link_obj
        with (
            patch(
                "linkforge.blender.utils.transform_utils.set_parent_keep_transform"
            ) as mock_set_parent,
            patch("linkforge.blender.utils.scene_utils.sync_object_collections") as mock_sync_coll,
        ):
            update_sensor_hierarchy(sp, bpy.context)
            assert not mock_set_parent.called
            assert mock_sync_coll.called

        # Test poll_robot_link
        assert poll_robot_link(sp, None) is False
        assert poll_robot_link(sp, link_obj) is True

    def test_transmission_props_extra_coverage(self, scene) -> None:
        """Test transmission_props edge cases for 100% coverage."""
        from linkforge.blender.properties.transmission_props import (
            get_transmission_name,
            set_transmission_name,
            update_transmission_hierarchy,
        )

        class MockTransProps:
            id_data = None
            source_name_stored = ""
            is_robot_transmission = True
            joint_name = None
            joint1_name = None
            transmission_type = "SIMPLE"

        mtp = MockTransProps()
        assert get_transmission_name(mtp) == ""

        # Test get_transmission_name when source_name_stored is present
        mtp.source_name_stored = "my_stored_trans"
        assert get_transmission_name(mtp) == "my_stored_trans"

        set_transmission_name(mtp, "")

        # Test set_transmission_name when transmission name is already equal
        class FakeTransObj:
            def __init__(self) -> None:
                self.name = "my_trans_obj"

        fto = FakeTransObj()
        mtp.id_data = fto
        set_transmission_name(mtp, "my_trans_obj")

        # update_transmission_hierarchy with missing transmission_obj or not robot transmission
        assert update_transmission_hierarchy(mtp, MagicMock()) is None

        # update_transmission_hierarchy clearing joint name but has parent
        trans_obj = create_test_object("test_trans", None, scene)
        parent_obj = create_test_object("some_parent", None, scene)
        trans_obj.parent = parent_obj

        from linkforge.blender.constants import PROP_TRANSMISSION

        tp = getattr(trans_obj, PROP_TRANSMISSION)
        tp.id_data = trans_obj
        tp.is_robot_transmission = True
        tp.joint_name = None

        update_transmission_hierarchy(tp, bpy.context)
        assert trans_obj.parent is None


class TestPreferencesExtra:
    def test_update_joint_axes_visibility(self) -> None:
        """Verify update_joint_axes_visibility triggers visualization update."""
        from linkforge.blender.preferences import update_joint_axes_visibility

        with patch("linkforge.blender.visualization.joint_gizmos.update_viz_handle") as mock_update:
            update_joint_axes_visibility(None, bpy.context)
            mock_update.assert_called_once_with(bpy.context)

    def test_update_empty_sizes_area_redraw(self, scene) -> None:
        """Verify that updating empty sizes triggers redraw loops on VIEW_3D areas."""

        class MockArea:
            type = "VIEW_3D"

            def __init__(self):
                self.redraw_called = False

            def tag_redraw(self):
                self.redraw_called = True

        class MockAreaOther:
            type = "PROPERTIES"

            def __init__(self):
                self.redraw_called = False

            def tag_redraw(self):
                self.redraw_called = True

        class MockScreen:
            def __init__(self, areas):
                self.areas = areas

        class MockWindow:
            def __init__(self, screen):
                self.screen = screen

        class MockWindowManager:
            def __init__(self, windows):
                self.windows = windows

        # Create mock scene and objects
        class MockObject:
            def __init__(self, name, obj_type, is_robot=True):
                self.name = name
                self.type = obj_type
                self.empty_display_size = 0.0

                # Mock custom properties for getters
                if "joint" in name:
                    self.linkforge_joint = MagicMock()
                    self.linkforge_joint.is_robot_joint = is_robot
                elif "sensor" in name:
                    self.linkforge_sensor = MagicMock()
                    self.linkforge_sensor.is_robot_sensor = is_robot
                elif "link" in name:
                    self.linkforge = MagicMock()
                    self.linkforge.is_robot_link = is_robot

        class MockScene:
            def __init__(self):
                self.objects = [
                    # Matching
                    MockObject("r_joint", "EMPTY"),
                    MockObject("r_sensor", "EMPTY"),
                    MockObject("r_link", "EMPTY"),
                    # Non-matching (non-empties)
                    MockObject("r_mesh", "MESH"),
                    # Non-matching (not robot)
                    MockObject("non_robot_joint", "EMPTY", is_robot=False),
                    MockObject("non_robot_sensor", "EMPTY", is_robot=False),
                    MockObject("non_robot_link", "EMPTY", is_robot=False),
                ]

        class MockContext:
            preferences = MagicMock()

            def __init__(self, wm, scene_val):
                self.window_manager = wm
                self.scene = scene_val

        mock_area = MockArea()
        mock_area_other = MockAreaOther()
        screen1 = MockScreen([mock_area, mock_area_other])
        win1 = MockWindow(screen1)

        mock_wm = MockWindowManager([win1])
        mock_scene = MockScene()
        mock_ctx = MockContext(mock_wm, mock_scene)

        class FakePrefs:
            joint_empty_size = 0.1
            sensor_empty_size = 0.2
            link_empty_size = 0.3
            show_inertia_gizmos = False

        prefs = FakePrefs()

        with patch("linkforge.blender.visualization.joint_gizmos.update_viz_handle"):
            update_joint_empty_size(prefs, mock_ctx)
            assert mock_area.redraw_called
            assert not mock_area_other.redraw_called
            assert mock_scene.objects[0].empty_display_size == 0.1

            mock_area.redraw_called = False
            update_sensor_empty_size(prefs, mock_ctx)
            assert mock_area.redraw_called
            assert mock_scene.objects[1].empty_display_size == 0.2

            mock_area.redraw_called = False
            update_link_empty_size(prefs, mock_ctx)
            assert mock_area.redraw_called
            assert mock_scene.objects[2].empty_display_size == 0.3

        # Test when context.scene is None
        mock_ctx_no_scene = MockContext(mock_wm, None)
        with patch("linkforge.blender.visualization.joint_gizmos.update_viz_handle"):
            update_joint_empty_size(prefs, mock_ctx_no_scene)
            update_sensor_empty_size(prefs, mock_ctx_no_scene)
            update_link_empty_size(prefs, mock_ctx_no_scene)

        # Test when window_manager is None
        mock_ctx_no_wm = MockContext(None, mock_scene)
        with patch("linkforge.blender.visualization.joint_gizmos.update_viz_handle"):
            update_joint_empty_size(prefs, mock_ctx_no_wm)
            update_sensor_empty_size(prefs, mock_ctx_no_wm)
            update_link_empty_size(prefs, mock_ctx_no_wm)

    def test_update_inertia_visibility_false(self) -> None:
        """Verify update_inertia_visibility behaves correctly when show_inertia_gizmos is False."""
        from linkforge.blender.preferences import update_inertia_visibility

        class FakePrefs:
            show_inertia_gizmos = False

        with (
            patch("linkforge.blender.visualization.inertia_gizmos.tag_redraw") as mock_redraw,
            patch(
                "linkforge.blender.visualization.inertia_gizmos.ensure_inertia_handler"
            ) as mock_handler,
        ):
            update_inertia_visibility(FakePrefs(), bpy.context)
            mock_redraw.assert_called_once()
            mock_handler.assert_not_called()

    def test_get_addon_prefs_no_preferences(self) -> None:
        """Verify get_addon_prefs returns None when context.preferences is None."""

        class MockContextNoPrefs:
            preferences = None

        from linkforge.blender.preferences import get_addon_prefs

        assert get_addon_prefs(MockContextNoPrefs()) is None

    def test_preferences_draw_conditional(self) -> None:
        """Test drawing preferences when show_joint_axes and show_inertia_gizmos are False."""
        prefs = LinkForgePreferences()
        prefs.show_joint_axes = False
        prefs.show_inertia_gizmos = False

        class MockLayout:
            def __init__(self):
                self.box_calls = 0
                self.separator_calls = 0
                self.prop_calls = []
                self.label_calls = []

            def box(self):
                self.box_calls += 1
                return self

            def separator(self):
                self.separator_calls += 1
                return self

            def row(self):
                return self

            def column(self, align=False):
                return self

            def prop(self, data, attr, text=None, slider=False):
                self.prop_calls.append(attr)

            def label(self, text, icon=None):
                self.label_calls.append(text)

        prefs.layout = MockLayout()
        prefs.draw(bpy.context)

        # Verify it drew all basic sections
        assert prefs.layout.box_calls > 0
        assert "additional_search_paths" in prefs.layout.prop_calls

    def test_get_addon_id_extension_prefix(self) -> None:
        """Verify get_addon_id handles Blender 4.2+ extension namespaces correctly."""
        with patch(
            "linkforge.blender.preferences.__package__", "bl_ext.user_default.linkforge.something"
        ):
            res = get_addon_id()
            assert res == "bl_ext.user_default.linkforge"

    def test_preferences_registration_errors(self) -> None:
        """Verify registration unregisters first if ValueError is raised."""
        call_count = 0

        def mock_register(cls):
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                raise ValueError("Already registered")
            return None

        with (
            patch("bpy.utils.register_class", side_effect=mock_register) as mock_reg,
            patch("bpy.utils.unregister_class") as mock_unreg,
        ):
            prefs_register()
            assert mock_reg.call_count == 2
            assert mock_unreg.called

    def test_preferences_main_entrypoint(self) -> None:
        """Test running preferences.py as __main__."""
        import runpy

        with (
            patch("bpy.utils.register_class") as mock_reg,
            patch("bpy.utils.unregister_class") as mock_unreg,
        ):
            runpy.run_module("linkforge.blender.preferences", run_name="__main__")
            assert mock_reg.called


class TestSimulationProperties:
    """Tests for advanced simulation physics properties."""

    def test_scientific_proxies(self) -> None:
        """Test scientific notation conversion logic."""

        # Create a mock instance (dataclass-like behavior for testing logic)
        class MockProps:
            def __init__(self):
                self.kp = 0.0
                self.kd = 0.0

        from typing import Any

        mock: Any = MockProps()
        from linkforge.blender.properties.link_props import (
            get_kd_scientific,
            get_kp_scientific,
            set_kd_scientific,
            set_kp_scientific,
        )

        # Test KP (Large value)
        mock.kp = 1.0e12
        assert get_kp_scientific(mock) == "1.00e+12"

        set_kp_scientific(mock, "5.5e+09")
        assert mock.kp == 5.5e9
        assert get_kp_scientific(mock) == "5.50e+09"

        # Test KD (Small value)
        mock.kd = 0.01
        assert get_kd_scientific(mock) == "1.00e-02"

        set_kd_scientific(mock, "1.0")
        assert mock.kd == 1.0
        assert get_kd_scientific(mock) == "1.00e+00"
