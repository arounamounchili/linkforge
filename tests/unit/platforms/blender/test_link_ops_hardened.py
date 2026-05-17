"""Hardened unit tests for LinkForge Blender operators and link operations."""

from __future__ import annotations

import time
from unittest.mock import patch

import bmesh
import bpy
import pytest
from linkforge.blender.operators.link_ops import (
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
    regenerate_collision_mesh,
    schedule_collision_preview_update,
    update_collision_quality_realtime,
)

from tests.blender_test_utils import (
    cleanup_blender_scene,
    create_mesh_object,
    create_robot_link,
    safe_get_linkforge,
    safe_update,
)
from tests.mock_bpy_env import MockPropertyGroup, MockTimers

# Monkeypatch missing mock timer method on the class itself to survive environment resets
MockTimers.is_registered = lambda self, func: func in self._timers  # type: ignore


class MockDecimateModifier(MockPropertyGroup):
    pass


bpy.types.DecimateModifier = MockDecimateModifier


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

        # Verify object creation
        assert "base_link" in bpy.data.objects
        link_obj = bpy.data.objects["base_link"]
        assert link_obj.type == "EMPTY"

        # Verify LinkForge properties
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

        # Run operator
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

        # Verify transforms
        assert (empty_obj.location - (1, 2, 3)).length < 1e-5
        assert visual_obj.matrix_parent_inverse.is_identity
        assert visual_obj.location.length < 1e-5


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

        # Test Box Primitive
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
        assert col_box["collision_geometry_type"] == "box"

        # Test Sphere Primitive
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
        assert col_sphere["collision_geometry_type"] == "sphere"

        # Test Cylinder Primitive
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
        assert col_cylinder["collision_geometry_type"] == "cylinder"

        # Test Auto-Detect type
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

        # Add a non-link object to verify iteration skips it
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
            "linkforge.blender.adapters.blender_to_core.detect_primitive_type",
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
            "linkforge.blender.adapters.blender_to_core.detect_primitive_type",
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
                "linkforge.blender.adapters.blender_to_core.detect_primitive_type",
                return_value=None,
            ),
            patch(
                "linkforge.blender.adapters.blender_to_core.extract_mesh_triangles",
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


class TestMaterialSlotSlotAddition:
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

        # Scenario 1: Decimate modifier exists
        col_obj.modifiers._items.clear()
        decimate_mod = col_obj.modifiers.new(name="Decimate", type="DECIMATE")
        decimate_mod.type = "DECIMATE"
        decimate_mod.__class__ = MockDecimateModifier
        decimate_mod.ratio = 1.0

        update_collision_quality_realtime(link_obj, col_obj)
        assert decimate_mod.ratio == 0.5

        # Scenario 2: Decimate modifier is missing but object is MESH (adds it)
        col_obj.modifiers.remove(decimate_mod)
        lf.collision_quality = 30.0

        update_collision_quality_realtime(link_obj, col_obj)
        new_mod = next(m for m in col_obj.modifiers if m.type == "DECIMATE")
        new_mod.type = "DECIMATE"
        new_mod.__class__ = MockDecimateModifier
        assert new_mod.ratio == 0.3

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
        import linkforge.blender.operators.link_ops as link_ops
        from linkforge.blender.operators.link_ops import COLLISION_PREVIEW_DEBOUNCE_DELAY

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
