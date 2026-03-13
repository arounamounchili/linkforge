"""Utilities for managing Blender execution context and modes."""

from __future__ import annotations

import contextlib
import typing

import bpy
from bpy.types import Context


@contextlib.contextmanager
def context_and_mode_guard(context: Context) -> typing.Iterator[dict[str, typing.Any]]:
    """Context manager to ensure safe execution of Blender operators.

    This handles two critical 'PRO' scenarios:
    1. Mode Switching: If the user is in Edit Mode, it switches to Object Mode
       and restores Edit Mode afterwards.
    2. Context Overriding: If operators are called from non-UI contexts (timers,
       background threads), it provides a valid 3D View context.
    """
    original_mode = getattr(context, "mode", "OBJECT")
    if "EDIT" in original_mode:
        bpy.ops.object.mode_set(mode="OBJECT")

    # Determine a valid context for operators
    override_kwargs: dict[str, typing.Any] = {}
    wm = bpy.context.window_manager
    with contextlib.suppress(Exception):
        if (not hasattr(context, "area") or not context.area or not context.window) and wm:
            for window in wm.windows:
                for area in window.screen.areas:
                    if area.type == "VIEW_3D":
                        override_kwargs["window"] = window
                        override_kwargs["area"] = area
                        override_kwargs["screen"] = window.screen
                        for region in area.regions:
                            if region.type == "WINDOW":
                                override_kwargs["region"] = region
                        break
                if override_kwargs:
                    break

    try:
        if override_kwargs:
            with bpy.context.temp_override(**override_kwargs):
                yield override_kwargs
        else:
            yield {}
    finally:
        # Restore original mode if we switched
        if "EDIT" in original_mode:
            with contextlib.suppress(Exception):
                bpy.ops.object.mode_set(mode=original_mode)  # type: ignore[arg-type]
