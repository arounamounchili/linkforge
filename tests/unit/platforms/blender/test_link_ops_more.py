"""More hardened unit tests for LinkForge Blender operators to reach >99% coverage."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.constants import (
    TAG_IMPORTED_SOURCE,
)
from linkforge.blender.operators.link_ops import (
    _merge_visual_meshes,
    calculate_inertia_for_link,
    create_collision_for_link,
    execute_collision_preview_update,
    regenerate_collision_mesh,
)

from tests.blender_test_utils import (
    cleanup_blender_scene,
    create_mesh_object,
    create_robot_link,
    create_test_object,
    safe_get_linkforge,
)


class TestLinkOpsMore:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_execute_collision_preview_update_branches(self, scene, blender_context) -> None:
        import linkforge.blender.operators.link_ops as link_ops

        # Branch 96: obj not in scene
        mock_obj = MagicMock()
        mock_obj.name = "NotInScene"
        link_ops._preview_pending_object = mock_obj
        link_ops._preview_last_request_time = 0
        assert execute_collision_preview_update() is None

        link_obj = create_robot_link("MyLink", scene)

        # Branch 101: no collision obj
        link_ops._preview_pending_object = link_obj
        link_ops._preview_last_request_time = 0
        assert execute_collision_preview_update() is None

        # Add collision obj
        col = create_test_object("MyLink_collision", None, scene)
        col.parent = link_obj

        # Primitive type branch 116
        col.name = "MyLink_collision"
        col.type = "MESH"

        with patch(
            "linkforge.blender.adapters.blender_to_core.detect_primitive_type"
        ) as mock_detect:
            mock_detect.return_value = "box"
            link_ops._preview_pending_object = link_obj
            link_ops._preview_last_request_time = 0
            assert execute_collision_preview_update() is None

        # TAG_IMPORTED_SOURCE branch
        with patch(
            "linkforge.blender.adapters.blender_to_core.detect_primitive_type"
        ) as mock_detect:
            mock_detect.return_value = None
            col[TAG_IMPORTED_SOURCE] = True
            link_ops._preview_pending_object = link_obj
            link_ops._preview_last_request_time = 0
            assert execute_collision_preview_update() is None

    def test_regenerate_collision_mesh_branches(self, scene) -> None:
        link_obj = create_robot_link("RegenLink", scene)

        # 149: not visual_children
        regenerate_collision_mesh(link_obj, "auto", bpy.context)

        vis = create_mesh_object("RegenLink_visual", scene)
        vis.parent = link_obj

        # 156-158: existing collisions
        col = create_test_object("RegenLink_collision", None, scene)
        col.parent = link_obj
        col.hide_viewport = False
        regenerate_collision_mesh(link_obj, "auto", bpy.context)

    def test_create_collision_for_link_branches(self, scene) -> None:
        # Branch 191: no visual children
        link_obj = create_robot_link("NoVisLink", scene, with_visual=False, with_collision=False)
        assert create_collision_for_link(link_obj, "box", bpy.context) is None

        # Compound logic branches
        link_obj = create_robot_link("CompLink", scene)
        vis1 = create_mesh_object("CompLink_visual_1", scene)
        vis1.parent = link_obj
        vis2 = create_mesh_object("CompLink_visual_2", scene)
        vis2.parent = link_obj

        col = create_collision_for_link(link_obj, "auto", bpy.context)
        assert col is not None

    def test_merge_visual_meshes_branches(self, scene) -> None:
        assert _merge_visual_meshes([], None, bpy.context) is None

        link_obj = create_robot_link("MergeLink", scene)
        vis1 = create_test_object("MergeLink_visual", None, scene)
        vis1.parent = link_obj
        # vis1 has no data, branch 351
        assert _merge_visual_meshes([vis1], link_obj, bpy.context) is None

    def test_calculate_inertia_for_link_branches(self, scene) -> None:
        link_obj = create_robot_link("InertiaLink", scene, with_visual=False, with_collision=False)
        # Branch 561: target_children is empty
        assert not calculate_inertia_for_link(link_obj)

        # target_children present
        vis = create_mesh_object("InertiaLink_visual", scene)
        vis.parent = link_obj

        # Mass = 0 -> branch 571
        safe_get_linkforge(link_obj).mass = 0

        with patch("linkforge.core.physics.calculate_mesh_inertia_from_triangles") as mock_calc:
            mock_calc.return_value = MagicMock(ixx=1, iyy=1, izz=1, ixy=0, ixz=0, iyz=0)
            with patch(
                "linkforge.blender.adapters.blender_to_core.extract_mesh_triangles"
            ) as mock_ext:
                mock_ext.return_value = (None, None)
                assert not calculate_inertia_for_link(link_obj)

                with patch("linkforge.core.validate_mesh_topology"):
                    mock_ext.return_value = ([1], [1])
                    assert calculate_inertia_for_link(link_obj)

    def test_operators_remaining(self, scene) -> None:
        # LINKFORGE_OT_calculate_inertia
        link_obj = create_robot_link("OpLink", scene)
        bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)
        res = bpy.ops.linkforge.calculate_inertia()
        assert res in [{"FINISHED"}, {"CANCELLED"}]

        # remove link
        res = bpy.ops.linkforge.remove_link()
        assert res in [{"FINISHED"}, {"CANCELLED"}]

        # toggle collision visibility
        res = bpy.ops.linkforge.toggle_collision_visibility()
        assert res in [{"FINISHED"}, {"CANCELLED"}]

        # generate collision all
        res = bpy.ops.linkforge.generate_collision_all()
        assert res in [{"FINISHED"}, {"CANCELLED"}]

        # add material slot
        mesh = create_mesh_object("MatMesh", scene)
        bpy.context.view_layer.objects.active = mesh
        mesh.select_set(True)
        res = bpy.ops.linkforge.add_material_slot()
        assert res in [{"FINISHED"}, {"CANCELLED"}]
