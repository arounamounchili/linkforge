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
_linter_timer_handle: typing.Any = None


def _perform_validation() -> None:
    """Execute the lightweight scene validation."""
    context = bpy.context
    scene = context.scene
    if not hasattr(scene, "linkforge") or not scene.linkforge.linter_active:
        return

    logger.debug("Live Linter: Performing validation...")

    # Update status
    scene.linkforge.linter_status = "Validating..."

    try:
        from ..adapters.blender_to_core import _categorize_scene_objects

        # 1. Categorize objects (this is the expensive loop, but necessary)
        link_objects, _, _, _, joints_map, _ = _categorize_scene_objects(scene)

        # 2. Check for Kinematic Cycles (Using Formal Core Graph)
        class MockLink:
            def __init__(self, name):
                self.name = name

        class MockJoint:
            def __init__(self, name, parent, child):
                self.name = name
                self.parent = parent
                self.child = child

        links = [MockLink(name) for name in link_objects]
        joints = [MockJoint(obj.name, parent, child) for child, (parent, obj) in joints_map.items()]

        graph = KinematicGraph(links, joints)
        has_cycles = graph.has_cycle()

        # 3. Update Linter Results
        if has_cycles:
            scene.linkforge.linter_status = "CRITICAL: Kinematic Cycle(s) detected!"
            scene.linkforge.linter_error_count = 1  # Simplified for live feedback
            logger.warning("Live Linter found kinematic cycles.")
        else:
            scene.linkforge.linter_status = "Ready"
            scene.linkforge.linter_error_count = 0

    except Exception as e:
        logger.error(f"Live Linter failed: {e}")
        scene.linkforge.linter_status = "Linter Error"


def _timer_callback() -> None:
    """Timer callback that executes after the debounce interval."""
    global _linter_timer_handle
    _linter_timer_handle = None
    _perform_validation()


@persistent
def depsgraph_update_post_handler(scene: bpy.types.Scene, depsgraph: bpy.types.Depsgraph) -> None:
    """Handle depsgraph updates by triggering a debounced validation."""
    global _linter_timer_handle

    # Avoid triggering if linter is disabled
    if not hasattr(scene, "linkforge") or not scene.linkforge.linter_active:
        return

    # Check if anything relevant changed (objects added/moved/parented)
    # We check if there are any object updates in the depsgraph
    if not any(update.id.type == "OBJECT" for update in depsgraph.updates):
        return

    # Reset/Register timer for debouncing
    if _linter_timer_handle is not None:
        bpy.app.timers.unregister(_linter_timer_handle)

    _linter_timer_handle = bpy.app.timers.register(
        _timer_callback, first_interval=_LINTR_TIMER_INTERVAL
    )


def register() -> None:
    """Register the live linter handlers."""
    if depsgraph_update_post_handler not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(depsgraph_update_post_handler)
    logger.info("Live Linter registered.")


def unregister() -> None:
    """Unregister the live linter handlers."""
    global _linter_timer_handle

    if depsgraph_update_post_handler in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(depsgraph_update_post_handler)

    if _linter_timer_handle is not None:
        with contextlib.suppress(Exception):
            bpy.app.timers.unregister(_linter_timer_handle)
        _linter_timer_handle = None
    logger.info("Live Linter unregistered.")
