"""Hardened unit tests for LinkForge Blender operators and link operations."""

from __future__ import annotations

import time
from typing import cast
from unittest.mock import MagicMock, patch

import bmesh
import bpy
import linkforge.blender.operators.link_ops as link_ops
import pytest
from bpy.types import DecimateModifier
from linkforge.blender.constants import TAG_IMPORTED_SOURCE
from linkforge.blender.logic.collision_builder import (
    _merge_visual_meshes,
    create_collision_for_link,
    regenerate_collision_mesh,
)
from linkforge.blender.operators.link_ops import (
    COLLISION_PREVIEW_DEBOUNCE_DELAY,
    LINKFORGE_OT_add_empty_link,
    LINKFORGE_OT_add_material_slot,
    LINKFORGE_OT_calculate_inertia,
    LINKFORGE_OT_calculate_inertia_all,
    LINKFORGE_OT_create_link_from_mesh,
    LINKFORGE_OT_generate_collision,
    LINKFORGE_OT_generate_collision_all,
    LINKFORGE_OT_remove_link,
    LINKFORGE_OT_toggle_collision_visibility,
    calculate_inertia_for_link,
    execute_collision_preview_update,
    schedule_collision_preview_update,
    update_collision_quality_realtime,
)
from linkforge.blender.properties.geom_props import (
    GEOM_BOX,
    GEOM_CYLINDER,
    GEOM_MESH,
    GEOM_SPHERE,
    PROP_GEOM,
)

from tests.blender_test_utils import (
    cleanup_blender_scene,
    create_mesh_object,
    create_robot_link,
    create_test_object,
    safe_get_linkforge,
    safe_update,
)


class TestLinkOperators:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_add_empty_link_operator(self, scene, blender_context) -> None:
        """Verify that the add_empty_link operator creates a valid link frame."""
        op = LINKFORGE_OT_add_empty_link()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        assert "base_link" in bpy.data.objects
        link_obj = bpy.data.objects["base_link"]
        assert link_obj.type == "EMPTY"

        lf = safe_get_linkforge(link_obj)
        assert lf.is_robot_link is True
        assert lf.link_name == "base_link"

    def test_create_link_from_mesh_operator(self, scene, blender_context) -> None:
        """Verify that create_link_from_mesh correctly restructures a mesh object."""
        mesh_obj = create_mesh_object("arm_segment", scene=scene, with_cube=True)
        mesh_obj.location = (1, 2, 3)
        safe_update(scene)

        # Poll checking
        op = LINKFORGE_OT_create_link_from_mesh
        assert not op.poll(bpy.context)  # Selected but not active yet

        view_layer = bpy.context.view_layer
        assert view_layer is not None
        view_layer.objects.active = mesh_obj
        mesh_obj.select_set(True)
        assert op.poll(bpy.context)

        res = op().execute(bpy.context)
        assert res == {"FINISHED"}

        # The Empty should now have the original name "arm_segment"
        assert "arm_segment" in bpy.data.objects
        empty_obj = bpy.data.objects["arm_segment"]
        assert empty_obj.type == "EMPTY"

        # The mesh should be renamed to "arm_segment_visual" and parented
        assert "arm_segment_visual" in bpy.data.objects
        visual_obj = bpy.data.objects["arm_segment_visual"]
        assert visual_obj.parent == empty_obj

        assert (empty_obj.location - (1, 2, 3)).length < 1e-5
        assert visual_obj.matrix_parent_inverse.is_identity
        assert visual_obj.location.length < 1e-5

    def test_link_ops_invalid_context(self) -> None:
        """Verify link operators handle invalid context gracefully."""
        op = LINKFORGE_OT_add_empty_link()

        class MockContextNoScene:
            scene = None

        assert op.execute(MockContextNoScene()) == {"CANCELLED"}


class TestCollisionGeneration:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_generate_collision_operator_poll_and_empty_visuals(
        self, scene, blender_context
    ) -> None:
        """Test generate collision poll and scenario with zero visuals."""
        op = LINKFORGE_OT_generate_collision
        assert not op.poll(bpy.context)

        link_obj = create_robot_link("empty_link", scene, with_visual=False, with_collision=False)
        view_layer = bpy.context.view_layer
        assert view_layer is not None
        view_layer.objects.active = link_obj
        link_obj.select_set(True)

        assert op.poll(bpy.context)

        # Running execution with no visual meshes should report ERROR and return CANCELLED
        res = op().execute(bpy.context)
        assert res == {"CANCELLED"}

    def test_generate_collision_primitive_shapes(self, scene, blender_context) -> None:
        """Test generating Box, Sphere, Cylinder primitives, and Auto-Detect."""
        view_layer = bpy.context.view_layer
        assert view_layer is not None

        link_obj_box = create_robot_link(
            "test_link_box", scene, with_visual=True, with_collision=False
        )
        visual_obj_box = link_obj_box.children[0]
        view_layer.objects.active = visual_obj_box
        visual_obj_box.select_set(True)

        assert LINKFORGE_OT_generate_collision.poll(bpy.context)

        op = LINKFORGE_OT_generate_collision()
        op.collision_type = "box"
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        col_box = next(c for c in link_obj_box.children if "collision" in c.name)
        assert getattr(col_box, PROP_GEOM).geometry_type == "box"

        link_obj_sphere = create_robot_link(
            "test_link_sphere", scene, with_visual=True, with_collision=False
        )
        visual_obj_sphere = link_obj_sphere.children[0]
        view_layer.objects.active = visual_obj_sphere
        visual_obj_sphere.select_set(True)

        op.collision_type = "sphere"
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        col_sphere = next(c for c in link_obj_sphere.children if "collision" in c.name)
        assert getattr(col_sphere, PROP_GEOM).geometry_type == "sphere"

        link_obj_cyl = create_robot_link(
            "test_link_cyl", scene, with_visual=True, with_collision=False
        )
        visual_obj_cyl = link_obj_cyl.children[0]
        view_layer.objects.active = visual_obj_cyl
        visual_obj_cyl.select_set(True)

        op.collision_type = "cylinder"
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        col_cylinder = next(c for c in link_obj_cyl.children if "collision" in c.name)
        assert getattr(col_cylinder, PROP_GEOM).geometry_type == "cylinder"

        link_obj_auto = create_robot_link(
            "test_link_auto", scene, with_visual=True, with_collision=False
        )
        visual_obj_auto = link_obj_auto.children[0]
        view_layer.objects.active = visual_obj_auto
        visual_obj_auto.select_set(True)

        op.collision_type = "auto"
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        col_auto = next(c for c in link_obj_auto.children if "collision" in c.name)
        assert col_auto is not None

    def test_generate_collision_all_operator(self, scene, blender_context) -> None:
        """Test batch generate collision operator."""
        link1 = create_robot_link("link1", scene, with_visual=True, with_collision=False)
        link2 = create_robot_link("link2", scene, with_visual=True, with_collision=False)

        non_link = create_mesh_object("non_link", scene)

        op = LINKFORGE_OT_generate_collision_all()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        assert any("collision" in c.name for c in link1.children)
        assert any("collision" in c.name for c in link2.children)


class TestCollisionVisibility:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_toggle_visibility_poll_and_execute(self, scene, blender_context) -> None:
        """Test polling and toggling collision visibility."""
        op = LINKFORGE_OT_toggle_collision_visibility
        assert not op.poll(bpy.context)

        link_obj = create_robot_link("test_link", scene, with_visual=True, with_collision=True)
        col_obj = next(c for c in link_obj.children if "collision" in c.name)
        col_obj.hide_viewport = False

        view_layer = bpy.context.view_layer
        assert view_layer is not None
        view_layer.objects.active = link_obj
        link_obj.select_set(True)
        assert op.poll(bpy.context)

        # Toggle on parent link
        res = op().execute(bpy.context)
        assert res == {"FINISHED"}
        assert col_obj.hide_viewport is True

        # Toggle on child visual
        visual_obj = link_obj.children[0]
        view_layer.objects.active = visual_obj
        visual_obj.select_set(True)
        assert op.poll(bpy.context)

        res = op().execute(bpy.context)
        assert res == {"FINISHED"}
        assert col_obj.hide_viewport is False


class TestInertiaCalculation:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_calculate_inertia_primitive_box(self, scene, blender_context) -> None:
        """Verify inertia calculation for a box primitive."""
        link_obj = create_robot_link("box_link", scene, with_collision=False)
        visual_obj = link_obj.children[0]
        visual_obj.scale = (1.0, 0.5, 0.25)
        safe_update(scene)

        lf = safe_get_linkforge(link_obj)
        lf.mass = 12.0

        success = calculate_inertia_for_link(link_obj)
        assert success is True

        assert abs(lf.inertia_ixx - 1.25) < 1e-3
        assert abs(lf.inertia_iyy - 4.25) < 1e-3
        assert abs(lf.inertia_izz - 5.0) < 1e-3

    def test_calculate_inertia_sphere_and_cylinder(self, scene, blender_context) -> None:
        """Verify primitive detection for sphere and cylinder works correctly."""
        # Sphere primitive test
        link_sphere = create_robot_link("sphere_link", scene, with_collision=False)
        vis_sphere = link_sphere.children[0]
        # Set sphere-like dimensions
        vis_sphere.dimensions = (2.0, 2.0, 2.0)
        safe_update(scene)

        with patch(
            "linkforge.blender.operators.link_ops.detect_primitive_type",
            return_value="sphere",
        ):
            lf = safe_get_linkforge(link_sphere)
            lf.mass = 5.0
            assert calculate_inertia_for_link(link_sphere) is True
            assert lf.inertia_ixx > 0

        # Cylinder primitive test
        link_cyl = create_robot_link("cyl_link", scene, with_collision=False)
        vis_cyl = link_cyl.children[0]
        vis_cyl.dimensions = (1.0, 1.0, 3.0)
        safe_update(scene)

        with patch(
            "linkforge.blender.operators.link_ops.detect_primitive_type",
            return_value="cylinder",
        ):
            lf = safe_get_linkforge(link_cyl)
            lf.mass = 3.0
            assert calculate_inertia_for_link(link_cyl) is True
            assert lf.inertia_ixx > 0

    def test_calculate_inertia_mesh_fallback(self, scene, blender_context) -> None:
        """Verify inertia calculation fallback for non-primitive meshes."""
        link_obj = create_robot_link("mesh_link", scene, with_visual=True)
        visual_obj = link_obj.children[0]
        mesh = bpy.data.meshes.new("mesh_with_cube")

        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=2.0)
        bm.to_mesh(mesh)
        bm.free()
        visual_obj.data = mesh
        safe_update(scene)

        bm = bmesh.new()
        bm.from_mesh(visual_obj.data)
        assert len(bm.verts) > 0
        bm.verts[0].co.x += 0.5
        bm.to_mesh(visual_obj.data)
        bm.free()
        safe_update(scene)

        lf = safe_get_linkforge(link_obj)
        lf.mass = 1.0

        success = calculate_inertia_for_link(link_obj)
        assert success is True
        assert lf.inertia_ixx > 0
        assert lf.inertia_iyy > 0
        assert lf.inertia_izz > 0

    def test_calculate_inertia_empty_mesh_warning(self, scene, blender_context) -> None:
        """Verify warning and failure path when target visual has no geometry."""
        link_obj = create_robot_link("empty_mesh_link", scene, with_visual=True)
        visual_obj = link_obj.children[0]

        # Empty mesh
        mesh = bpy.data.meshes.new("empty_mesh")
        visual_obj.data = mesh
        safe_update(scene)

        with (
            patch(
                "linkforge.blender.operators.link_ops.detect_primitive_type",
                return_value=None,
            ),
            patch(
                "linkforge.blender.operators.link_ops.extract_mesh_triangles",
                return_value=([], []),
            ),
        ):
            success = calculate_inertia_for_link(link_obj)
            assert success is False

    def test_calculate_inertia_operators(self, scene, blender_context) -> None:
        """Verify active and batch inertia calculation operators."""
        link_obj = create_robot_link("test_link", scene, with_visual=True, with_collision=False)
        view_layer = bpy.context.view_layer
        assert view_layer is not None
        view_layer.objects.active = link_obj
        link_obj.select_set(True)

        # Single active link calculate
        op = LINKFORGE_OT_calculate_inertia()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        # Batch all calculate
        op_all = LINKFORGE_OT_calculate_inertia_all()
        res_all = op_all.execute(bpy.context)
        assert res_all == {"FINISHED"}


class TestLinkRemoval:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_remove_virtual_empty_link(self, scene, blender_context) -> None:
        """Verify remove link operator on a virtual link frame (no visual mesh)."""
        link_obj = create_robot_link("virtual_link", scene, with_visual=False, with_collision=True)
        view_layer = bpy.context.view_layer
        assert view_layer is not None
        view_layer.objects.active = link_obj
        link_obj.select_set(True)

        op = LINKFORGE_OT_remove_link()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        assert "virtual_link" not in bpy.data.objects

    def test_remove_link_with_visual_mesh(self, scene, blender_context) -> None:
        """Verify remove link operator correctly restores original mesh object."""
        link_obj = create_robot_link("mesh_link", scene, with_visual=True, with_collision=True)
        visual_obj = link_obj.children[0]
        view_layer = bpy.context.view_layer
        assert view_layer is not None
        view_layer.objects.active = visual_obj
        visual_obj.select_set(True)

        op = LINKFORGE_OT_remove_link()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        # The visual mesh should be restored back to a root-level object with original name
        assert "mesh_link" in bpy.data.objects
        assert bpy.data.objects["mesh_link"].type == "MESH"
        assert bpy.data.objects["mesh_link"].parent is None


class TestMaterialSlotAddition:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_add_material_slot_from_link(self, scene, blender_context) -> None:
        """Verify adding material slot to link active object."""
        link_obj = create_robot_link("mat_link", scene, with_visual=True, with_collision=False)
        view_layer = bpy.context.view_layer
        assert view_layer is not None
        view_layer.objects.active = link_obj
        link_obj.select_set(True)

        op = LINKFORGE_OT_add_material_slot()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        visual_obj = link_obj.children[0]
        assert len(visual_obj.data.materials) == 1
        assert visual_obj.data.materials[0].name == "mat_link_material"

    def test_add_material_slot_from_visual(self, scene, blender_context) -> None:
        """Verify adding material slot directly to visual child."""
        link_obj = create_robot_link("mat_link_2", scene, with_visual=True, with_collision=False)
        visual_obj = link_obj.children[0]
        view_layer = bpy.context.view_layer
        assert view_layer is not None
        view_layer.objects.active = visual_obj
        visual_obj.select_set(True)

        op = LINKFORGE_OT_add_material_slot()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        assert len(visual_obj.data.materials) == 1
        assert visual_obj.data.materials[0].name == "mat_link_2_material"


class TestCompoundOperations:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_regenerate_collision_compound(self, scene, blender_context) -> None:
        """Verify that regenerate_collision_mesh creates a compound mesh for multiple visuals."""
        link_obj = create_robot_link(
            "multi_visual_link", scene, with_visual=False, with_collision=False
        )

        v1 = create_mesh_object("v1_visual", scene=scene, with_cube=True)
        v1.parent = link_obj
        v1.location = (1, 0, 0)

        v2 = create_mesh_object("v2_visual", scene=scene, with_cube=True)
        v2.parent = link_obj
        v2.location = (-1, 0, 0)

        safe_update(scene)

        regenerate_collision_mesh(link_obj, "mesh", bpy.context)

        collision_objs = [c for c in link_obj.children if "_collision" in c.name]
        assert len(collision_objs) == 1
        col_obj = collision_objs[0]

        assert abs(col_obj.dimensions.x - 4.0) < 0.1
        assert abs(col_obj.dimensions.y - 2.0) < 0.1
        assert abs(col_obj.dimensions.z - 2.0) < 0.1


class TestRealtimePreviewsAndDebounce:
    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_realtime_preview_quality_decimate_ratio(self, scene, blender_context) -> None:
        """Verify update_collision_quality_realtime updates Decimate modifier or adds it."""
        link_obj = create_robot_link("quality_link", scene, with_visual=True, with_collision=True)
        col_obj = next(c for c in link_obj.children if "collision" in c.name)

        # Set quality to 50%
        lf = safe_get_linkforge(link_obj)
        lf.collision_quality = 50.0

        # Existing Decimate modifier update
        col_obj.modifiers._items.clear()
        decimate_mod = col_obj.modifiers.new(name="Decimate", type="DECIMATE")
        decimate_mod.type = "DECIMATE"
        decimate_mod.ratio = 1.0

        update_collision_quality_realtime(link_obj, col_obj)
        assert decimate_mod.ratio == 0.5

        # Missing Decimate modifier creation on mesh object
        col_obj.modifiers.remove(decimate_mod)

        getattr(col_obj, PROP_GEOM).collision_quality = 30.0

        update_collision_quality_realtime(link_obj, col_obj)
        new_mod = next(m for m in col_obj.modifiers if m.type == "DECIMATE")
        new_mod.type = "DECIMATE"
        assert new_mod.ratio == 0.3

    def test_realtime_preview_primitive_invariance_and_cleanup(
        self, scene, blender_context
    ) -> None:
        """Verify primitive types are exempt from decimation and modifiers are cleaned up."""
        link_obj = create_robot_link(
            "prim_invariance_link", scene, with_visual=True, with_collision=True
        )
        col_obj = next(c for c in link_obj.children if "collision" in c.name)
        geom = getattr(col_obj, PROP_GEOM)

        # Attach a Decimate modifier while in MESH mode
        geom.geometry_type = GEOM_MESH
        geom.collision_quality = 50.0
        col_obj.modifiers._items.clear()
        decimate_mod = col_obj.modifiers.new(name="Decimate", type="DECIMATE")
        decimate_mod.type = "DECIMATE"

        # Switch to primitive shapes and verify modifier removal and decimation abortion
        for prim_type in (GEOM_BOX, GEOM_CYLINDER, GEOM_SPHERE):
            geom.geometry_type = prim_type
            if not any(m.type == "DECIMATE" for m in col_obj.modifiers):
                mod = col_obj.modifiers.new(name="Decimate", type="DECIMATE")
                mod.type = "DECIMATE"
            update_collision_quality_realtime(link_obj, col_obj)
            assert not any(m.type == "DECIMATE" for m in col_obj.modifiers)

    def test_debounce_timer_lifecycle(self, scene, blender_context) -> None:
        """Verify schedule_collision_preview_update schedules and debounces correctly."""
        link_obj = create_robot_link("debounce_link", scene, with_visual=True, with_collision=True)

        # Clear existing timers
        getattr(bpy.app.timers, "_timers").clear()

        # Schedule preview
        schedule_collision_preview_update(link_obj)

        # Should be registered now
        assert execute_collision_preview_update in getattr(bpy.app.timers, "_timers")

        # Trigger execute within delay (should reschedule by returning remaining wait time)
        link_ops._preview_pending_object = link_obj
        link_ops._preview_last_request_time = time.time()  # just now

        wait_time = execute_collision_preview_update()
        assert wait_time is not None
        assert 0.0 < wait_time <= COLLISION_PREVIEW_DEBOUNCE_DELAY

        # Trigger execute after delay passes (should run actual update and clear pending)
        link_ops._preview_last_request_time = time.time() - 1.0
        res = execute_collision_preview_update()
        assert res is None
        assert link_ops._preview_pending_object is None


class TestCollisionAlignment:
    def test_collision_alignment_on_rotated_link(self, scene, blender_context) -> None:
        """Verify that generating collision for a rotated link avoids offsets."""
        link_obj = create_test_object("link_obj", None, scene=scene)
        safe_get_linkforge(link_obj).is_robot_link = True
        link_obj.rotation_euler = (1.5708, 0, 0)  # 90 deg X

        visual_obj = create_mesh_object("part_visual", scene=scene)
        visual_obj.parent = link_obj
        visual_obj.matrix_parent_inverse.identity()

        # Generate Collision
        collision_obj = create_collision_for_link(link_obj, "mesh", bpy.context)

        assert collision_obj is not None
        assert collision_obj.parent == link_obj
        # Local transform should be near identity
        assert collision_obj.location.length < 1e-5
        assert collision_obj.rotation_euler.x < 1e-5


class TestCollisionQuality:
    def test_collision_modifier_persistence(self, scene, blender_context) -> None:
        """Verify that generating mesh collision preserves Decimate modifier."""
        link_obj = create_mesh_object("link_obj", scene=scene)
        safe_get_linkforge(link_obj).is_robot_link = True

        safe_get_linkforge(link_obj).collision_quality = 50.0
        create_collision_for_link(link_obj, "mesh", bpy.context)

        collision_obj = next(c for c in link_obj.children if "_collision" in c.name)
        decimate_mod = cast(
            DecimateModifier, next(m for m in collision_obj.modifiers if m.type == "DECIMATE")
        )
        assert decimate_mod.ratio == 0.5


class TestCollisionScaling:
    def test_box_collision_scaling(self, scene, blender_context) -> None:
        """Verify that a scaled cube results in a matching collision primitive."""
        link_obj = create_mesh_object("scaled_link", scene=scene, with_cube=True)
        link_obj.scale = (2.0, 1.5, 0.5)
        safe_update(scene)

        safe_get_linkforge(link_obj).is_robot_link = True

        collision_obj = create_collision_for_link(link_obj, "box", bpy.context)
        assert collision_obj is not None
        # Dimensions should be 4x3x1
        assert abs(collision_obj.dimensions.x - 4.0) < 1e-5
        assert abs(collision_obj.dimensions.y - 3.0) < 1e-5
        assert abs(collision_obj.dimensions.z - 1.0) < 1e-5


class TestLinkCreationAndCollisionHelpers:
    def test_create_link_object(self, scene, blender_context) -> None:
        """Test creating a link object (empty) in Blender."""
        link_obj = create_robot_link("test_link", scene)
        assert link_obj.name.startswith("test_link")
        assert link_obj.type == "EMPTY"
        assert safe_get_linkforge(link_obj).is_robot_link

    def test_create_collision_no_geometry(self, scene, blender_context) -> None:
        """Test robustness when creating collision for link with no geometry."""
        link_obj = create_robot_link("empty_link", scene)

        # No children, no geometry
        col_obj = create_collision_for_link(link_obj, "box", bpy.context)
        assert link_obj.type == "EMPTY"
        assert safe_get_linkforge(link_obj).is_robot_link

    def test_create_collision_for_link(self, scene, blender_context) -> None:
        """Test generating a primitive collision for a link."""
        link_obj = create_robot_link("link_with_collision", scene)

        vis = create_mesh_object("link_visual", scene)
        vis.parent = link_obj

        col_obj = create_collision_for_link(link_obj, "box", bpy.context)

        assert col_obj is not None
        assert col_obj.parent == link_obj
        assert "collision" in col_obj.name.lower()


class TestLinkProperties:
    def test_link_property_persistence(self, scene, blender_context) -> None:
        """Test setting and getting link forge properties."""
        obj = create_test_object("test_props", None, scene)
        props = safe_get_linkforge(obj)
        assert props is not None
        obj.name = "Original Name"
        safe_get_linkforge(obj).is_robot_link = True

        # Getter should return sanitized name
        assert safe_get_linkforge(obj).link_name == "Original_Name"

        # Setter should update object name
        safe_get_linkforge(obj).link_name = "New-Link-Name!"
        assert obj.name == "New-Link-Name_"

    def test_automatic_child_renaming(self, scene, blender_context) -> None:
        """Test that renaming a link object also renames its children."""
        link_obj = create_robot_link("base_link", scene)

        vis_obj = create_test_object("base_link_visual", None, scene)
        vis_obj.parent = link_obj

        # Rename the link
        safe_get_linkforge(link_obj).link_name = "chassis"

        assert link_obj.name == "chassis"
        assert vis_obj.name.startswith("chassis_visual")


class TestLinkRobustness:
    def test_execute_collision_preview_update_branches(self, scene, blender_context) -> None:
        """Test edge cases in collision preview update."""
        link_obj = create_robot_link("Link", scene)

        # Simulate missing view_layer context
        with patch("linkforge.blender.operators.link_ops.bpy") as mock_bpy:
            mock_bpy.data = bpy.data
            mock_bpy.context = MagicMock()
            mock_bpy.context.view_layer = None

            link_ops._preview_pending_object = link_obj
            assert execute_collision_preview_update() is None

    def test_regenerate_collision_mesh_validation(self, scene, blender_context) -> None:
        """Test validation in regenerate_collision_mesh."""
        # Passing non-link object should not crash
        obj = create_test_object("NotALink", None, scene)
        regenerate_collision_mesh(obj, "auto", bpy.context)

    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_execute_collision_preview_update_branches_more(self, scene, blender_context) -> None:
        # obj not in scene
        mock_obj = MagicMock()
        mock_obj.name = "NotInScene"
        link_ops._preview_pending_object = mock_obj
        link_ops._preview_last_request_time = 0
        assert execute_collision_preview_update() is None

        link_obj = create_robot_link("MyLink", scene)

        # no collision obj
        link_ops._preview_pending_object = link_obj
        link_ops._preview_last_request_time = 0
        assert execute_collision_preview_update() is None

        col = create_test_object("MyLink_collision", None, scene)
        col.parent = link_obj

        # Primitive type fallback
        col.name = "MyLink_collision"
        col.type = "MESH"

        with patch("linkforge.blender.operators.link_ops.detect_primitive_type") as mock_detect:
            mock_detect.return_value = "box"
            link_ops._preview_pending_object = link_obj
            link_ops._preview_last_request_time = 0
            assert execute_collision_preview_update() is None

        # Verify handling with imported source tag present
        with patch("linkforge.blender.operators.link_ops.detect_primitive_type") as mock_detect:
            mock_detect.return_value = None
            col[TAG_IMPORTED_SOURCE] = True
            link_ops._preview_pending_object = link_obj
            link_ops._preview_last_request_time = 0
            assert execute_collision_preview_update() is None

    def test_regenerate_collision_mesh_branches(self, scene) -> None:
        link_obj = create_robot_link("RegenLink", scene)

        # not visual_children
        regenerate_collision_mesh(link_obj, "auto", bpy.context)

        vis = create_mesh_object("RegenLink_visual", scene)
        vis.parent = link_obj

        # existing collisions
        col = create_test_object("RegenLink_collision", None, scene)
        col.parent = link_obj
        col.hide_viewport = False
        regenerate_collision_mesh(link_obj, "auto", bpy.context)

    def test_create_collision_for_link_branches(self, scene) -> None:
        # no visual children
        link_obj = create_robot_link("NoVisLink", scene, with_visual=False, with_collision=False)
        assert create_collision_for_link(link_obj, "box", bpy.context) is None

        # Verify collision creation for link with multiple visual meshes
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
        # vis1 has no data
        assert _merge_visual_meshes([vis1], link_obj, bpy.context) is None

    def test_calculate_inertia_for_link_branches(self, scene) -> None:
        link_obj = create_robot_link("InertiaLink", scene, with_visual=False, with_collision=False)
        # target_children is empty
        assert not calculate_inertia_for_link(link_obj)

        # target_children present
        vis = create_mesh_object("InertiaLink_visual", scene)
        vis.parent = link_obj

        # Mass = 0
        safe_get_linkforge(link_obj).mass = 0

        with patch("linkforge.core.physics.calculate_mesh_inertia_from_triangles") as mock_calc:
            mock_calc.return_value = MagicMock(ixx=1, iyy=1, izz=1)
            with patch("linkforge.blender.operators.link_ops.extract_mesh_triangles") as mock_ext:
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

        mesh = create_mesh_object("MatMesh", scene)
        bpy.context.view_layer.objects.active = mesh
        mesh.select_set(True)
        res = bpy.ops.linkforge.add_material_slot()
        assert res in [{"FINISHED"}, {"CANCELLED"}]


class TestResolveActiveLink:
    """Tests for the _resolve_active_link helper function."""

    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_resolve_from_direct_link(self, scene, blender_context) -> None:
        """Active object is a robot link — should resolve directly."""
        link_obj = create_robot_link("direct_link", scene)
        bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        result = link_ops._resolve_active_link(bpy.context)
        assert result is link_obj

    def test_resolve_from_visual_child(self, scene, blender_context) -> None:
        """Active object is a visual child — should resolve to parent link."""
        link_obj = create_robot_link("parent_link", scene, with_visual=True)
        visual_child = link_obj.children[0]
        bpy.context.view_layer.objects.active = visual_child
        visual_child.select_set(True)

        result = link_ops._resolve_active_link(bpy.context)
        assert result is link_obj

    def test_resolve_from_selected_objects(self, scene, blender_context) -> None:
        """Active object is a loose mesh but a link is selected — resolve from selection."""
        link_obj = create_robot_link("sel_link", scene, with_visual=False, with_collision=False)
        loose_mesh = create_mesh_object("loose", scene)

        bpy.context.view_layer.objects.active = loose_mesh
        bpy.context.selected_objects = [loose_mesh, link_obj]

        result = link_ops._resolve_active_link(bpy.context)
        assert result is link_obj

    def test_resolve_from_selected_child(self, scene, blender_context) -> None:
        """Active object is loose but selected objects include a visual child of a link."""
        link_obj = create_robot_link("sel_parent", scene, with_visual=True)
        visual_child = link_obj.children[0]
        loose_mesh = create_mesh_object("loose2", scene)

        bpy.context.view_layer.objects.active = loose_mesh
        bpy.context.selected_objects = [loose_mesh, visual_child]

        result = link_ops._resolve_active_link(bpy.context)
        assert result is link_obj

    def test_resolve_returns_none(self, scene, blender_context) -> None:
        """No link or link child anywhere — should return None."""
        loose = create_mesh_object("unrelated", scene)
        bpy.context.view_layer.objects.active = loose
        loose.select_set(True)

        result = link_ops._resolve_active_link(bpy.context)
        assert result is None

    def test_resolve_no_active_object(self, scene, blender_context) -> None:
        """No active object — should return None."""
        bpy.context.view_layer.objects.active = None
        result = link_ops._resolve_active_link(bpy.context)
        assert result is None


class TestAssignAsVisual:
    """Tests for LINKFORGE_OT_assign_as_visual."""

    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_assign_visual_execute(self, scene, blender_context) -> None:
        """Assigning a loose mesh as visual child of a link."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_assign_as_visual

        link_obj = create_robot_link("target_link", scene, with_visual=False, with_collision=False)
        loose_mesh = create_mesh_object("my_mesh", scene, with_cube=True)

        bpy.context.view_layer.objects.active = link_obj
        bpy.context.selected_objects = [link_obj, loose_mesh]

        op = LINKFORGE_OT_assign_as_visual()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        # Mesh should be parented to link
        assert loose_mesh.parent is link_obj
        assert "_visual" in loose_mesh.name.lower()

    def test_assign_visual_poll_fails_no_loose_mesh(self, scene, blender_context) -> None:
        """Poll should fail when no loose mesh is selected."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_assign_as_visual

        link_obj = create_robot_link("solo_link", scene, with_visual=False, with_collision=False)
        bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        assert not LINKFORGE_OT_assign_as_visual.poll(bpy.context)

    def test_assign_visual_no_link_resolved(self, scene, blender_context) -> None:
        """Execute cancels if no link is resolved."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_assign_as_visual

        loose = create_mesh_object("orphan", scene)
        bpy.context.view_layer.objects.active = loose
        loose.select_set(True)

        op = LINKFORGE_OT_assign_as_visual()
        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}


class TestAssignAsCollision:
    """Tests for LINKFORGE_OT_assign_as_collision."""

    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_assign_collision_execute(self, scene, blender_context) -> None:
        """Assigning a loose mesh as collision child of a link."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_assign_as_collision

        link_obj = create_robot_link("col_link", scene, with_visual=False, with_collision=False)
        loose_mesh = create_mesh_object("col_mesh", scene, with_cube=True)

        bpy.context.view_layer.objects.active = link_obj
        bpy.context.selected_objects = [link_obj, loose_mesh]

        op = LINKFORGE_OT_assign_as_collision()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        # Mesh should be parented and configured as collision
        assert loose_mesh.parent is link_obj
        assert "_collision" in loose_mesh.name.lower()
        assert loose_mesh.display_type == "WIRE"
        assert loose_mesh.show_in_front is True
        assert loose_mesh.hide_render is True


class TestSetActiveGeometry:
    """Tests for LINKFORGE_OT_set_active_geometry."""

    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_set_active_visual(self, scene, blender_context) -> None:
        """Set active visual geometry index."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_set_active_geometry

        link_obj = create_robot_link("geo_link", scene, with_visual=True, with_collision=True)
        bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        op = LINKFORGE_OT_set_active_geometry()
        op.geometry_type = "VISUAL"
        op.index = 0
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

    def test_set_active_collision(self, scene, blender_context) -> None:
        """Set active collision geometry index."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_set_active_geometry

        link_obj = create_robot_link("geo_link2", scene, with_visual=True, with_collision=True)
        bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        op = LINKFORGE_OT_set_active_geometry()
        op.geometry_type = "COLLISION"
        op.index = 0
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

    def test_set_active_no_link(self, scene, blender_context) -> None:
        """Returns cancelled when no link is resolved."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_set_active_geometry

        bpy.context.view_layer.objects.active = None
        op = LINKFORGE_OT_set_active_geometry()
        op.geometry_type = "VISUAL"
        op.index = 0
        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}


class TestRemoveVisual:
    """Tests for LINKFORGE_OT_remove_visual."""

    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_remove_visual_execute(self, scene, blender_context) -> None:
        """Remove a visual child from a link."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_remove_visual

        link_obj = create_robot_link("rv_link", scene, with_visual=True, with_collision=False)
        visual_child = link_obj.children[0]
        visual_name = visual_child.name

        lf = safe_get_linkforge(link_obj)
        lf.active_visual_index = 0

        bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        op = LINKFORGE_OT_remove_visual()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        # Visual should be unparented (not deleted, just detached)
        assert visual_child.parent is None

    def test_remove_visual_poll_no_index(self, scene, blender_context) -> None:
        """Poll fails if active_visual_index is negative."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_remove_visual

        link_obj = create_robot_link("rv_link2", scene, with_visual=True, with_collision=False)
        lf = safe_get_linkforge(link_obj)
        lf.active_visual_index = -1

        bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        assert not LINKFORGE_OT_remove_visual.poll(bpy.context)

    def test_remove_visual_no_link(self, scene, blender_context) -> None:
        """Execute cancels when no link is resolved."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_remove_visual

        bpy.context.view_layer.objects.active = None
        op = LINKFORGE_OT_remove_visual()
        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}


class TestRemoveCollision:
    """Tests for LINKFORGE_OT_remove_collision."""

    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_remove_collision_execute(self, scene, blender_context) -> None:
        """Remove a collision child from a link."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_remove_collision

        link_obj = create_robot_link("rc_link", scene, with_visual=False, with_collision=True)
        collision_child = [c for c in link_obj.children if "_collision" in c.name.lower()][0]

        lf = safe_get_linkforge(link_obj)
        lf.active_collision_index = 0

        bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        op = LINKFORGE_OT_remove_collision()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        # Collision object should be removed from data
        assert collision_child.name not in bpy.data.objects

    def test_remove_collision_no_link(self, scene, blender_context) -> None:
        """Execute cancels when no link is resolved."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_remove_collision

        bpy.context.view_layer.objects.active = None
        op = LINKFORGE_OT_remove_collision()
        res = op.execute(bpy.context)
        assert res == {"CANCELLED"}


class TestRemoveLinkWithVisuals:
    """Tests for LINKFORGE_OT_remove_link with visual children (the mesh path)."""

    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_remove_link_restores_visuals(self, scene, blender_context) -> None:
        """Remove link with visual children — visuals should be unparented and restored."""
        link_obj = create_robot_link("rl_link", scene, with_visual=True, with_collision=True)
        visual_child = [c for c in link_obj.children if "_visual" in c.name.lower()][0]

        bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        op = LINKFORGE_OT_remove_link()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

        # Link should be deleted
        assert link_obj not in list(bpy.data.objects)

        # Visual should be restored (unparented)
        assert visual_child.parent is None

    def test_remove_link_poll_from_child(self, scene, blender_context) -> None:
        """Poll should accept when a visual/collision child is selected."""
        link_obj = create_robot_link("rl_poll", scene, with_visual=True, with_collision=True)
        visual = [c for c in link_obj.children if "_visual" in c.name.lower()][0]

        bpy.context.view_layer.objects.active = visual
        visual.select_set(True)

        assert LINKFORGE_OT_remove_link.poll(bpy.context)

    def test_remove_link_poll_unselected(self, scene, blender_context) -> None:
        """Poll should fail if object is not selected."""
        link_obj = create_robot_link("rl_unsel", scene)

        bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(False)

        assert not LINKFORGE_OT_remove_link.poll(bpy.context)


class TestCalculateInertiaAllBranches:
    """Tests for batch inertia calculation branches."""

    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_calculate_inertia_all_no_links(self, scene, blender_context) -> None:
        """Calculate inertia all with no links should report 'no links found'."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_calculate_inertia_all

        op = LINKFORGE_OT_calculate_inertia_all()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

    def test_calculate_inertia_all_success(self, scene, blender_context) -> None:
        """Calculate inertia all with valid links should succeed."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_calculate_inertia_all

        link1 = create_robot_link("ia_link1", scene, with_visual=True, with_collision=False)
        link2 = create_robot_link("ia_link2", scene, with_visual=True, with_collision=False)

        op = LINKFORGE_OT_calculate_inertia_all()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

    def test_calculate_inertia_all_mixed_results(self, scene, blender_context) -> None:
        """Calculate inertia all with some failures."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_calculate_inertia_all

        # Create link with visual that has cube (should succeed)
        create_robot_link("ia_ok", scene, with_visual=True, with_collision=False)
        # Create link with empty visual (no geometry — should exercise the failure path)
        create_robot_link(
            "ia_empty", scene, with_visual=True, with_collision=False, with_cube=False
        )

        op = LINKFORGE_OT_calculate_inertia_all()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}

    def test_calculate_inertia_all_no_scene(self) -> None:
        """Calculate inertia all when context has no scene."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_calculate_inertia_all

        class MockContextNoScene:
            scene = None

        op = LINKFORGE_OT_calculate_inertia_all()
        assert op.execute(MockContextNoScene()) == {"FINISHED"}

    def test_calculate_inertia_all_all_failed(self, scene, monkeypatch) -> None:
        """Calculate inertia all when all links fail."""
        from linkforge.blender.operators import link_ops

        create_robot_link("ia_fail", scene, with_visual=True, with_collision=False)
        monkeypatch.setattr(link_ops, "calculate_inertia_for_link", lambda obj: False)
        op = link_ops.LINKFORGE_OT_calculate_inertia_all()
        assert op.execute(bpy.context) == {"FINISHED"}


class TestLinkOpsUncoveredBranches:
    """Tests covering remaining edge branches in link_ops."""

    @pytest.fixture(autouse=True)
    def setup_cleanup(self, scene):
        cleanup_blender_scene(scene)
        yield
        cleanup_blender_scene(scene)

    def test_calculate_inertia_single_fail(self, scene, monkeypatch) -> None:
        """Single link calculate inertia reports warning on failure."""
        from linkforge.blender.operators import link_ops

        link_obj = create_robot_link("single_fail", scene, with_visual=True)
        bpy.context.view_layer.objects.active = link_obj
        bpy.context.selected_objects = [link_obj]
        monkeypatch.setattr(link_ops, "calculate_inertia_for_link", lambda obj: False)
        op = link_ops.LINKFORGE_OT_calculate_inertia()
        assert op.execute(bpy.context) == {"CANCELLED"}

    def test_calculate_inertia_no_active(self, scene) -> None:
        """Calculate inertia returns CANCELLED when no active object."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_calculate_inertia

        bpy.context.view_layer.objects.active = None
        op = LINKFORGE_OT_calculate_inertia()
        assert op.execute(bpy.context) == {"CANCELLED"}

    def test_generate_collision_no_links(self, scene) -> None:
        """Generate collision returns CANCELLED if no robot links."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_generate_collision

        op = LINKFORGE_OT_generate_collision()
        assert op.execute(bpy.context) == {"CANCELLED"}

    def test_generate_collision_no_active(self, scene) -> None:
        """Generate collision returns CANCELLED if active object is None."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_generate_collision

        create_robot_link("gen_link", scene, with_visual=True)
        bpy.context.view_layer.objects.active = None
        op = LINKFORGE_OT_generate_collision()
        assert op.execute(bpy.context) == {"CANCELLED"}

    def test_generate_collision_no_visuals(self, scene) -> None:
        """Generate collision fails with error if link has no visual mesh."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_generate_collision

        link_obj = create_robot_link("virtual_link", scene, with_visual=False, with_collision=False)
        bpy.context.view_layer.objects.active = link_obj
        bpy.context.selected_objects = [link_obj]
        op = LINKFORGE_OT_generate_collision()
        assert op.execute(bpy.context) == {"CANCELLED"}

    def test_generate_collision_creation_failure(self, scene, monkeypatch) -> None:
        """Generate collision fails if collision generator returns None."""
        from linkforge.blender.operators import link_ops

        link_obj = create_robot_link("fail_col_link", scene, with_visual=True, with_collision=False)
        bpy.context.view_layer.objects.active = link_obj
        bpy.context.selected_objects = [link_obj]
        monkeypatch.setattr(link_ops, "create_collision_for_link", lambda obj, t, ctx: None)
        op = link_ops.LINKFORGE_OT_generate_collision()
        assert op.execute(bpy.context) == {"CANCELLED"}

    def test_assign_as_visual_no_meshes_selected(self, scene) -> None:
        """Assign as visual cancels if only the link itself is selected."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_assign_as_visual

        link_obj = create_robot_link("vis_only", scene, with_visual=False)
        bpy.context.view_layer.objects.active = link_obj
        bpy.context.selected_objects = [link_obj]
        op = LINKFORGE_OT_assign_as_visual()
        assert op.execute(bpy.context) == {"CANCELLED"}

    def test_assign_as_collision_no_meshes_selected(self, scene) -> None:
        """Assign as collision cancels if only the link itself is selected."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_assign_as_collision

        link_obj = create_robot_link("col_only", scene, with_visual=False)
        bpy.context.view_layer.objects.active = link_obj
        bpy.context.selected_objects = [link_obj]
        op = LINKFORGE_OT_assign_as_collision()
        assert op.execute(bpy.context) == {"CANCELLED"}

    def test_assign_as_collision_no_link_resolved(self, scene) -> None:
        """Assign as collision cancels if active is loose with no link."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_assign_as_collision

        loose = create_mesh_object("orphan_col", scene)
        bpy.context.view_layer.objects.active = loose
        bpy.context.selected_objects = [loose]
        op = LINKFORGE_OT_assign_as_collision()
        assert op.execute(bpy.context) == {"CANCELLED"}
        assert not LINKFORGE_OT_assign_as_collision.poll(bpy.context)

    def test_remove_visual_multi_adjust_index(self, scene) -> None:
        """Removing last visual of multiple adjusts active_visual_index."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_remove_visual

        link_obj = create_robot_link("mv_link", scene, with_visual=True, with_collision=False)
        mesh2 = create_mesh_object("mv_link_visual_1", scene=scene, with_cube=True)
        mesh2.parent = link_obj
        bpy.context.view_layer.objects.active = link_obj
        bpy.context.selected_objects = [link_obj]
        link_obj.linkforge.active_visual_index = 1
        op = LINKFORGE_OT_remove_visual()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        assert link_obj.linkforge.active_visual_index == 0

    def test_remove_visual_no_link(self, scene) -> None:
        """Remove visual returns CANCELLED if no link resolved."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_remove_visual

        bpy.context.view_layer.objects.active = None
        bpy.context.selected_objects = []
        op = LINKFORGE_OT_remove_visual()
        assert op.execute(bpy.context) == {"CANCELLED"}
        assert not LINKFORGE_OT_remove_visual.poll(bpy.context)

    def test_remove_collision_multi_adjust_index(self, scene) -> None:
        """Removing last collision of multiple adjusts active_collision_index."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_remove_collision

        link_obj = create_robot_link("mc_link", scene, with_visual=False, with_collision=True)
        col2 = create_mesh_object("mc_link_collision_1", scene=scene, with_cube=True)
        col2.parent = link_obj
        bpy.context.view_layer.objects.active = link_obj
        bpy.context.selected_objects = [link_obj]
        link_obj.linkforge.active_collision_index = 1
        op = LINKFORGE_OT_remove_collision()
        res = op.execute(bpy.context)
        assert res == {"FINISHED"}
        assert link_obj.linkforge.active_collision_index == 0

    def test_remove_collision_no_link(self, scene) -> None:
        """Remove collision returns CANCELLED if no link resolved."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_remove_collision

        bpy.context.view_layer.objects.active = None
        bpy.context.selected_objects = []
        op = LINKFORGE_OT_remove_collision()
        assert op.execute(bpy.context) == {"CANCELLED"}
        assert not LINKFORGE_OT_remove_collision.poll(bpy.context)

    def test_add_material_slot_link_no_visuals(self, scene) -> None:
        """Add material slot fails when link has no visual mesh."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_add_material_slot

        link_obj = create_robot_link("novis_mat", scene, with_visual=False, with_collision=False)
        bpy.context.view_layer.objects.active = link_obj
        op = LINKFORGE_OT_add_material_slot()
        assert op.execute(bpy.context) == {"CANCELLED"}

    def test_add_material_slot_mesh_no_parent(self, scene) -> None:
        """Add material slot cancels when active mesh has no parent."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_add_material_slot

        orphan_mesh = create_mesh_object("orphan_mesh", scene)
        bpy.context.view_layer.objects.active = orphan_mesh
        op = LINKFORGE_OT_add_material_slot()
        assert op.execute(bpy.context) == {"CANCELLED"}

    def test_add_material_slot_existing_material(self, scene) -> None:
        """Add material slot reuses material if name already exists."""
        from linkforge.blender.operators.link_ops import LINKFORGE_OT_add_material_slot

        link_obj = create_robot_link("reuse_mat_link", scene, with_visual=True)
        bpy.data.materials.new("reuse_mat_link_material")
        bpy.context.view_layer.objects.active = link_obj
        op = LINKFORGE_OT_add_material_slot()
        assert op.execute(bpy.context) == {"FINISHED"}

    def test_update_collision_quality_realtime_paths(self, scene) -> None:
        """Test realtime collision quality update branches."""
        from linkforge.blender.operators import link_ops

        # None inputs
        link_ops.update_collision_quality_realtime(None, None)

        link_obj = create_robot_link("ucq_link", scene, with_visual=True, with_collision=True)
        col_child = [c for c in link_obj.children if "_collision" in c.name.lower()][0]

        # Box geometry (primitive) removes decimate modifier if present
        col_child.linkforge_geom.geometry_type = "BOX"
        col_child.modifiers.new(name="Decimate", type="DECIMATE")
        link_ops.update_collision_quality_realtime(link_obj, col_child)
        assert not any(m.type == "DECIMATE" for m in col_child.modifiers)

        # Mesh geometry without decimate modifier creates one
        col_child.linkforge_geom.geometry_type = "mesh"
        col_child.linkforge_geom.collision_quality = 50.0
        link_ops.update_collision_quality_realtime(link_obj, col_child)
        assert any(m.type == "DECIMATE" for m in col_child.modifiers)

    def test_execute_collision_preview_update_branches(self, scene) -> None:
        """Test debounce timer callback branches."""
        import time

        from linkforge.blender.operators import link_ops

        # No pending object
        link_ops._preview_pending_object = None
        assert link_ops.execute_collision_preview_update() is None

        # Pending object recently set (elapsed < debounce)
        link_obj = create_robot_link("deb_link", scene, with_visual=True, with_collision=True)
        link_ops._preview_pending_object = link_obj
        link_ops._preview_last_request_time = time.time()
        res = link_ops.execute_collision_preview_update()
        assert res is not None

        # Elapsed passes, object has PROP_LINK
        link_ops._preview_last_request_time = time.time() - 1.0
        link_ops._preview_pending_object = link_obj
        assert link_ops.execute_collision_preview_update() is None

    def test_link_ops_register_unregister(self) -> None:
        """Register and unregister link operators."""
        from linkforge.blender.operators import link_ops

        link_ops.register()
        link_ops.unregister()
        link_ops.register()
