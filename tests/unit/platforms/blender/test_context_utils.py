"""Unit tests for context and mode guards."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import bpy
from linkforge.blender.utils.context import context_and_mode_guard


def test_context_and_mode_guard_object_mode() -> None:
    """Verify object mode guard does not switch mode."""
    context = MagicMock()
    context.mode = "OBJECT"
    context.area = MagicMock()
    context.window = MagicMock()

    with patch.object(bpy.ops.object, "mode_set") as mock_mode_set:
        with context_and_mode_guard(context) as override:
            assert override == {}
        assert not mock_mode_set.called


def test_context_and_mode_guard_edit_mode() -> None:
    """Verify edit mode guard switches to object mode and restores edit mode on exit."""
    context = MagicMock()
    context.mode = "EDIT_MESH"
    context.area = MagicMock()
    context.window = MagicMock()

    with patch.object(bpy.ops.object, "mode_set") as mock_mode_set:
        with context_and_mode_guard(context):
            pass

        # Called once to enter OBJECT, once to restore EDIT_MESH
        assert mock_mode_set.call_count == 2
        mock_mode_set.assert_any_call(mode="OBJECT")
        mock_mode_set.assert_any_call(mode="EDIT_MESH")


def test_context_and_mode_guard_override_kwargs() -> None:
    """Verify non-UI area execution finds VIEW_3D area and overrides context."""
    context = MagicMock()
    context.mode = "OBJECT"
    # Cause override path to execute
    context.area = None
    context.window = None

    # Setup windows and areas
    mock_window = MagicMock()
    mock_area = MagicMock()
    mock_area.type = "VIEW_3D"
    mock_region = MagicMock()
    mock_region.type = "WINDOW"
    mock_area.regions = [mock_region]
    mock_window.screen.areas = [mock_area]

    with (
        patch.object(bpy.context, "window_manager") as mock_wm,
        patch.object(bpy.context, "temp_override") as mock_temp,
    ):
        # Window manager windows
        mock_wm.windows = [mock_window]

        with context_and_mode_guard(context) as override:
            # Inside the block, temp_override should be active
            pass

        assert mock_temp.called
        assert "window" in override
        assert override["window"] == mock_window
        assert override["area"] == mock_area
        assert override["region"] == mock_region
