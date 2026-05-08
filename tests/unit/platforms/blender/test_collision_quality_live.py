from typing import cast

import bpy
import pytest
from bpy.types import DecimateModifier
from linkforge.blender.operators.link_ops import (
    create_collision_for_link,
    update_collision_quality_realtime,
)

from tests.blender_test_utils import safe_get_linkforge


def test_collision_quality_live_modifier_persistence(scene) -> None:
    """Verify that generating a mesh collision preserves the Decimate modifier."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Create a link with a visual mesh
    bpy.ops.mesh.primitive_monkey_add()
    getattr(bpy.ops, "linkforge").create_link_from_mesh()
    link_obj = bpy.context.active_object
    assert link_obj is not None

    # Set quality to 50%
    safe_get_linkforge(link_obj).collision_quality = 50.0

    # Generate MESH collision
    create_collision_for_link(link_obj, "MESH", bpy.context)

    # Find collision object
    collision_obj = next(c for c in link_obj.children if "_collision" in c.name)

    # VERIFY: Decimate modifier should exist and NOT be applied
    decimate_mod = cast(
        DecimateModifier, next((m for m in collision_obj.modifiers if m.type == "DECIMATE"), None)
    )
    assert decimate_mod is not None
    assert decimate_mod.ratio == 0.5
    assert decimate_mod.decimate_type == "COLLAPSE"


def test_update_collision_quality_realtime_success(scene) -> None:
    """Verify that the realtime update modifies the existing Decimate modifier."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Setup link and collision with modifier
    bpy.ops.mesh.primitive_monkey_add()
    getattr(bpy.ops, "linkforge").create_link_from_mesh()
    link_obj = bpy.context.active_object
    assert link_obj is not None

    create_collision_for_link(link_obj, "MESH", bpy.context)
    collision_obj = next(c for c in link_obj.children if "_collision" in c.name)
    decimate_mod = cast(
        DecimateModifier, next(m for m in collision_obj.modifiers if m.type == "DECIMATE")
    )

    # Change quality in properties
    safe_get_linkforge(link_obj).collision_quality = 20.0

    # Call the realtime update (normally called by the property update callback)
    update_collision_quality_realtime(link_obj, collision_obj)

    # VERIFY: Modifier ratio updated immediately
    assert decimate_mod.ratio == pytest.approx(0.2)


def test_update_collision_quality_realtime_fallback(scene) -> None:
    """Verify fallback to debounced regeneration if modifier is missing."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Setup link and collision without modifier
    bpy.ops.mesh.primitive_monkey_add()
    getattr(bpy.ops, "linkforge").create_link_from_mesh()
    link_obj = bpy.context.active_object
    assert link_obj is not None
    safe_get_linkforge(link_obj).collision_quality = 20.0

    create_collision_for_link(link_obj, "MESH", bpy.context)
    collision_obj = next(c for c in link_obj.children if "_collision" in c.name)

    # Remove all modifiers
    collision_obj.modifiers.clear()

    update_collision_quality_realtime(link_obj, collision_obj)

    # Verify modifier restoration (Fast Path)
    decimate_mod = cast(
        DecimateModifier, next((m for m in collision_obj.modifiers if m.type == "DECIMATE"), None)
    )
    assert decimate_mod is not None
    assert decimate_mod.ratio == pytest.approx(0.2)
