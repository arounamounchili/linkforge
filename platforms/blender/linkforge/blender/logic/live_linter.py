"""Live Linter for LinkForge Blender.

This module provides real-time, debounced validation of the robot structure
in the Blender scene. It monitors the scene for changes via handlers and
provides visual feedback on kinematic cycles and property errors.
"""

from __future__ import annotations

import contextlib
import typing

import bpy
from bpy.app.handlers import persistent

from ...linkforge_core.logging_config import get_logger
from ...linkforge_core.models.graph import KinematicGraph

logger = get_logger(__name__)

# Debounce timer configuration
_LINTR_TIMER_INTERVAL = 1.0  # Seconds to wait after last change before linting
_linter_timer_active: bool = False


def _perform_validation() -> None:
    """Execute the lightweight scene validation."""
    context = bpy.context
    scene = getattr(context, "scene", None)
    if scene is None or not hasattr(scene, "linkforge") or not scene.linkforge.linter_active:
        return

    logger.debug("Live Linter: Performing validation...")

    # Update status safely
    if hasattr(scene.linkforge, "linter_status"):
        scene.linkforge.linter_status = "Validating..."

    try:
        from ..adapters.blender_to_core import _categorize_scene_objects

        # 1. Categorize objects (this is the expensive loop, but necessary)
        link_objects, _, _, _, joints_map, _ = _categorize_scene_objects(scene)

        # 2. Check for Kinematic Cycles (Using Formal Core Graph)
        from collections import namedtuple

        LinkMock = namedtuple("LinkMock", ["name"])
        JointMock = namedtuple("JointMock", ["name", "parent", "child"])

        links = [LinkMock(name) for name in link_objects]
        joints = [JointMock(obj.name, parent, child) for child, (parent, obj) in joints_map.items()]

        # Use Any cast for mocks as they provide structural compatibility
        graph = KinematicGraph(typing.cast(typing.Any, links), typing.cast(typing.Any, joints))
        has_cycles = graph.has_cycle()

        # 3. Update Linter Results
        if has_cycles:
            if hasattr(scene.linkforge, "linter_status"):
                scene.linkforge.linter_status = "CRITICAL: Kinematic Cycle(s) detected!"
            if hasattr(scene.linkforge, "linter_error_count"):
                scene.linkforge.linter_error_count = 1
            logger.warning("Live Linter found kinematic cycles.")
        else:
            if hasattr(scene.linkforge, "linter_status"):
                scene.linkforge.linter_status = "Ready"
            if hasattr(scene.linkforge, "linter_error_count"):
                scene.linkforge.linter_error_count = 0

    except Exception as e:
        logger.error(f"Live Linter failed: {e}")
        if hasattr(scene.linkforge, "linter_status"):
            scene.linkforge.linter_status = "Linter Error"


def _timer_callback() -> None:
    """Timer callback that executes after the debounce interval."""
    global _linter_timer_active
    _linter_timer_active = False
    _perform_validation()


@persistent  # type: ignore[misc]
def depsgraph_update_post_handler(scene: bpy.types.Scene, depsgraph: bpy.types.Depsgraph) -> None:
    """Handle depsgraph updates by triggering a debounced validation."""
    global _linter_timer_active

    # Avoid triggering if linter is disabled
    linkforge = getattr(scene, "linkforge", None)
    if linkforge is None or not linkforge.linter_active:
        return

    # Check if anything relevant changed (objects added/moved/parented)
    # Cast updates to avoid type errors with dynamic bpy attributes
    updates = typing.cast(list[typing.Any], depsgraph.updates)
    if not any(getattr(update.id, "type", None) == "OBJECT" for update in updates if update.id):
        return

    # Reset/Register timer for debouncing
    if _linter_timer_active:
        with contextlib.suppress(Exception):
            bpy.app.timers.unregister(_timer_callback)

    _linter_timer_active = True
    bpy.app.timers.register(_timer_callback, first_interval=_LINTR_TIMER_INTERVAL)


def register() -> None:
    """Register the live linter handlers."""
    if depsgraph_update_post_handler not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(depsgraph_update_post_handler)
    logger.info("Live Linter registered.")


def unregister() -> None:
    """Unregister the live linter handlers."""
    global _linter_timer_active

    if depsgraph_update_post_handler in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(depsgraph_update_post_handler)

    if _linter_timer_active:
        with contextlib.suppress(Exception):
            bpy.app.timers.unregister(_timer_callback)
        _linter_timer_active = False
    logger.info("Live Linter unregistered.")
