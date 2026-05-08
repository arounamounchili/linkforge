import typing
from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.operators.link_ops import (
    create_collision_for_link,
    execute_collision_preview_update,
    regenerate_collision_mesh,
)

from tests.blender_test_utils import (
    create_test_object,
    safe_get_linkforge,
)

if typing.TYPE_CHECKING:
    pass


def test_execute_collision_preview_update_branches(clean_scene, scene) -> None:
    """Hit edge cases."""
    link_obj = create_test_object("Link", None, scene)
    assert link_obj is not None
    safe_get_linkforge(link_obj).is_robot_link = True

    col_mesh = bpy.data.meshes.new("col")
    col_obj = create_test_object("Link_collision", col_mesh, scene)
    assert col_obj is not None
    col_obj.parent = link_obj

    # No view_layer
    with patch("linkforge.blender.operators.link_ops.bpy") as mock_bpy:
        # Simulate missing view_layer context
        mock_bpy.data = bpy.data
        mock_bpy.context = MagicMock()
        mock_bpy.context.view_layer = None

        # We need to set the global _preview_pending_object
        import linkforge.blender.operators.link_ops as link_ops

        link_ops._preview_pending_object = link_obj
        link_ops._preview_last_request_time = 0.0
        assert execute_collision_preview_update() is None

    # imported_from_source
    col_obj["imported_from_source"] = True
    link_ops._preview_pending_object = link_obj
    link_ops._preview_last_request_time = 0.0
    assert execute_collision_preview_update() is None
    col_obj["imported_from_source"] = False


def test_regenerate_collision_mesh_validation(clean_scene, scene) -> None:
    """Hit edge cases validation in regenerate."""
    # Passing None or non-link object
    regenerate_collision_mesh(None, "AUTO", bpy.context)

    o = create_test_object("NotLink", None, scene)
    assert o is not None
    regenerate_collision_mesh(o, "AUTO", bpy.context)


def test_create_collision_failure_branches(clean_scene, scene) -> None:
    """Hit edge cases collision creation failure."""
    link_obj = create_test_object("Link", None, scene)
    assert link_obj is not None
    safe_get_linkforge(link_obj).is_robot_link = True

    # Force _create_primitive_collision to fail (return None)
    with patch(
        "linkforge.blender.operators.link_ops._create_primitive_collision",
        return_value=(None, (0, 0, 0)),
    ):
        assert create_collision_for_link(link_obj, "BOX", bpy.context) is None


def test_generate_collision_all_skip(clean_scene, scene) -> None:
    """Hit edge cases skipping links with no visuals."""
    link_obj = create_test_object("EmptyLink", None, scene)
    assert link_obj is not None
    safe_get_linkforge(link_obj).is_robot_link = True

    # link_obj has no children, so generate_collision_all should skip it
    bpy.ops.linkforge.generate_collision_all()  # type: ignore[attr-defined]
    assert not any("_collision" in obj.name for obj in bpy.data.objects)


def test_add_material_slot_skip(clean_scene, scene) -> None:
    """Hit edge cases skipping if no visual."""
    link_obj = create_test_object("MatLink", None, scene)
    assert link_obj is not None
    safe_get_linkforge(link_obj).is_robot_link = True

    assert bpy.context.view_layer is not None
    bpy.context.view_layer.objects.active = link_obj
    # add_material_slot should do nothing/return if no visual found
    with pytest.raises(RuntimeError, match="No visual mesh found"):
        bpy.ops.linkforge.add_material_slot()  # type: ignore[attr-defined]
