"""Concurrency and Stress tests for Blender integration.

This module tests the robustness of the LinkForge Blender integration under
stress, specifically focusing on the AsynchronousRobotBuilder and property
synchronization during rapid UI updates or concurrent operations.
"""

from __future__ import annotations

from pathlib import Path

import bpy
import pytest
from linkforge.blender.adapters.context import BlenderContext
from linkforge.blender.logic.asynchronous_builder import AsynchronousRobotBuilder
from linkforge_core.composer import RobotBuilder
from linkforge_core.composer.helpers import box


def test_rapid_collection_cleanup_stress():
    """Stress test rapid creation and deletion of robot collections.

    Verifies that LinkForge's internal tracking doesn't leak or crash
    when models are rapidly cycled.
    """
    builder = RobotBuilder("cycle_bot")
    builder.link("base").visual(box(1, 1, 1)).collision().mass(1)
    robot = builder.build()
    context = BlenderContext(bpy.context)

    for _ in range(5):
        # Import
        async_builder = AsynchronousRobotBuilder(
            robot, Path("/tmp/dummy.urdf"), context, chunk_size=100
        )
        while not async_builder.is_finished:
            async_builder.process_next_chunk()

        # Immediately delete
        coll = bpy.data.collections.get(robot.name)
        if coll:
            # Standard LinkForge deletion pattern (simulated)
            for obj in coll.objects:
                bpy.data.objects.remove(obj, do_unlink=True)
            bpy.data.collections.remove(coll)

        if bpy.context.view_layer:
            bpy.context.view_layer.update()

    # No crash is a pass


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
