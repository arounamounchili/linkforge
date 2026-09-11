"""Unit tests for procedural collision generation logic in collision_builder."""

from __future__ import annotations

import bpy
import pytest
from linkforge.blender.logic.collision_builder import (
    _create_primitive_collision,
    _merge_visual_meshes,
    create_collision_for_link,
    regenerate_collision_mesh,
)
from linkforge.blender.properties.geom_props import (
    GEOM_BOX,
    GEOM_CYLINDER,
    GEOM_SPHERE,
    PROP_GEOM,
)

from tests.blender_test_utils import (
    create_mesh_object,
    create_robot_link,
    create_test_object,
)


class TestCollisionBuilder:
    def test_regenerate_collision_mesh_empty_or_invalid(self, scene) -> None:
        """Verify early exit when object is invalid, not a robot link, or has no visuals."""
        # None object
        regenerate_collision_mesh(None, "auto", bpy.context)

        # Plain object without link property
        plain_obj = bpy.data.objects.new("plain", None)
        regenerate_collision_mesh(plain_obj, "auto", bpy.context)

        # Robot link without visual children
        link_obj = create_robot_link("empty_link", scene, with_visual=False, with_collision=False)
        regenerate_collision_mesh(link_obj, "auto", bpy.context)
        assert len([c for c in link_obj.children if "collision" in c.name.lower()]) == 0

    def test_regenerate_collision_mesh_compound(self, scene) -> None:
        """Verify that regenerate_collision_mesh replaces existing collisions with compound mesh."""
        link_obj = create_robot_link(
            "multi_vis_link", scene, with_visual=False, with_collision=False
        )
        v1 = create_mesh_object("multi_vis_link_visual_1", scene, with_cube=True)
        v1.parent = link_obj
        v2 = create_mesh_object("multi_vis_link_visual_2", scene, with_cube=True)
        v2.parent = link_obj

        # Add an initial collision object
        c_old = create_test_object("multi_vis_link_collision", None, scene)
        c_old.parent = link_obj
        c_old.hide_viewport = False

        regenerate_collision_mesh(link_obj, "mesh", bpy.context)

        col_children = [c for c in link_obj.children if "collision" in c.name.lower()]
        assert len(col_children) >= 1
        assert not col_children[0].hide_viewport

    def test_create_collision_for_link_no_visuals(self, scene) -> None:
        """Verify returning None when link has no visual children."""
        link_obj = create_robot_link("no_vis_link", scene, with_visual=False, with_collision=False)
        assert create_collision_for_link(link_obj, "box", bpy.context) is None

    def test_create_collision_primitive_types(self, scene) -> None:
        """Verify primitive collision generation for box, sphere, cylinder, and auto."""
        # Box
        link_box = create_robot_link("link_box", scene, with_visual=True, with_collision=False)
        col_box = create_collision_for_link(link_box, "box", bpy.context)
        assert col_box is not None
        assert col_box.parent == link_box
        assert "collision" in col_box.name.lower()
        assert getattr(col_box, PROP_GEOM).geometry_type == GEOM_BOX

        # Sphere
        link_sphere = create_robot_link(
            "link_sphere", scene, with_visual=True, with_collision=False
        )
        col_sphere = create_collision_for_link(link_sphere, "sphere", bpy.context)
        assert col_sphere is not None
        assert getattr(col_sphere, PROP_GEOM).geometry_type == GEOM_SPHERE

        # Cylinder
        link_cyl = create_robot_link("link_cyl", scene, with_visual=True, with_collision=False)
        col_cyl = create_collision_for_link(link_cyl, "cylinder", bpy.context)
        assert col_cyl is not None
        assert getattr(col_cyl, PROP_GEOM).geometry_type == GEOM_CYLINDER

        # Auto
        link_auto = create_robot_link("link_auto", scene, with_visual=True, with_collision=False)
        col_auto = create_collision_for_link(link_auto, "auto", bpy.context)
        assert col_auto is not None

    def test_create_collision_mesh_compound_quality(self, scene) -> None:
        """Verify compound mesh collision generation with Decimate modifier."""
        link_obj = create_robot_link("quality_link", scene, with_visual=False, with_collision=False)
        v1 = create_mesh_object("quality_link_visual_1", scene, with_cube=True)
        v1.parent = link_obj

        col = create_collision_for_link(link_obj, "mesh", bpy.context)
        assert col is not None
        assert col.parent == link_obj

        # Check for decimate modifier
        decimate_mod = next((m for m in col.modifiers if m.type == "DECIMATE"), None)
        assert decimate_mod is not None
        assert pytest.approx(decimate_mod.ratio, rel=1e-3) == 0.5

    def test_merge_visual_meshes_empty_and_invalid(self, scene) -> None:
        """Verify merge visual meshes returns None on empty or data-less meshes."""
        assert _merge_visual_meshes([], None, bpy.context) is None

        link_obj = create_robot_link("merge_empty", scene)
        vis_empty = create_test_object("merge_empty_visual", None, scene)
        vis_empty.parent = link_obj
        assert _merge_visual_meshes([vis_empty], link_obj, bpy.context) is None

    def test_merge_visual_meshes_success(self, scene) -> None:
        """Verify merging multiple valid meshes into a single mesh object."""
        link_obj = create_robot_link("merge_valid", scene)
        vis1 = create_mesh_object("merge_valid_visual_1", scene, with_cube=True)
        vis1.parent = link_obj
        vis2 = create_mesh_object("merge_valid_visual_2", scene, with_cube=True)
        vis2.parent = link_obj

        merged = _merge_visual_meshes([vis1, vis2], link_obj, bpy.context)
        assert merged is not None
        assert merged.data is not None

    def test_create_primitive_collision_unknown_type(self, scene) -> None:
        """Verify _create_primitive_collision with unknown collision type returns None."""
        link_obj = create_robot_link("unknown_type_link", scene, with_visual=True)
        vis = link_obj.children[0]
        res, offset = _create_primitive_collision(
            vis, "non_existent_type", "unknown_type_link", bpy.context
        )
        assert res is None
