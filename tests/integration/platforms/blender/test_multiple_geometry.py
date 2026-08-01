"""Integration tests for Multiple Visuals & Collisions Geometry Workflow."""

from __future__ import annotations

import time
from pathlib import Path

import bpy
from linkforge.blender.operators.export_ops import LINKFORGE_OT_export_robot_model
from linkforge.blender.operators.import_ops import LINKFORGE_OT_import_robot_model
from linkforge.blender.operators.link_ops import (
    LINKFORGE_OT_add_empty_link,
    LINKFORGE_OT_assign_as_collision,
    LINKFORGE_OT_assign_as_visual,
)
from linkforge.blender.properties.geom_props import GEOM_CYLINDER, GEOM_MESH, GEOM_SPHERE, PROP_GEOM
from linkforge.blender.utils.transform_utils import Vector3 as MockVector

from tests.blender_test_utils import (
    cleanup_blender_scene,
    create_test_object,
    safe_get_linkforge_scene,
    safe_update,
)


class TestMultipleGeometryWorkflow:
    def test_multi_visual_collision_assignment_and_export_import(
        self, blender_clean_scene, tmp_path: Path
    ) -> None:
        """Verify full end-to-end workflow for assigning multiple visuals/collisions,

        preserving transforms, exporting URDF, and re-importing.
        """
        scene = bpy.context.scene
        lf_scene = safe_get_linkforge_scene(scene)
        lf_scene.robot_name = "multi_geom_bot"
        lf_scene.export_format = "URDF"

        # Create a Link Empty
        bpy.ops.object.select_all(action="DESELECT")
        res = LINKFORGE_OT_add_empty_link.execute(LINKFORGE_OT_add_empty_link(), bpy.context)
        assert res == {"FINISHED"}
        link_obj = bpy.context.active_object
        assert link_obj is not None

        # Create loose mesh objects in scene
        sphere_mesh = create_test_object("sphere_mesh", None, scene)
        sphere_mesh.type = "MESH"
        sphere_mesh.location = (1, 0, 0)
        sphere_mesh._base_dimensions = MockVector(1.0, 1.0, 1.0)

        cylinder_mesh = create_test_object("cylinder_mesh", None, scene)
        cylinder_mesh.type = "MESH"
        cylinder_mesh.location = (0, 1, 0)
        cylinder_mesh._base_dimensions = MockVector(0.6, 0.6, 1.0)

        # Assign sphere as Visual
        bpy.context.view_layer.objects.active = link_obj
        bpy.context.selected_objects = [sphere_mesh, link_obj]
        sphere_mesh.type = "MESH"

        res_vis = LINKFORGE_OT_assign_as_visual.execute(
            LINKFORGE_OT_assign_as_visual(), bpy.context
        )
        assert res_vis == {"FINISHED"}
        assert sphere_mesh.parent == link_obj
        vis_geom = getattr(sphere_mesh, PROP_GEOM)
        assert vis_geom.geom_role == "VISUAL"
        vis_geom.geometry_type = GEOM_SPHERE

        # Assign Cylinder as Collision
        bpy.context.view_layer.objects.active = link_obj
        bpy.context.selected_objects = [cylinder_mesh, link_obj]
        cylinder_mesh.type = "MESH"

        res_col = LINKFORGE_OT_assign_as_collision.execute(
            LINKFORGE_OT_assign_as_collision(), bpy.context
        )
        assert res_col == {"FINISHED"}
        assert cylinder_mesh.parent == link_obj
        col_geom = getattr(cylinder_mesh, PROP_GEOM)
        assert col_geom.geom_role == "COLLISION"
        col_geom.geometry_type = GEOM_CYLINDER
        assert cylinder_mesh.display_type == "WIRE"
        assert cylinder_mesh.show_in_front is True
        assert cylinder_mesh.hide_render is True
        assert col_geom.collision_quality == 100.0
        assert not any(m.type == "DECIMATE" for m in cylinder_mesh.modifiers)

        # Test Primitive Invariance & Modifier Cleanup:
        # Switch to MESH and lower quality to 50% to trigger decimation
        col_geom.geometry_type = GEOM_MESH
        col_geom.collision_quality = 50.0
        assert any(m.type == "DECIMATE" for m in cylinder_mesh.modifiers)

        # Switch back to a primitive shape and verify decimate modifier is removed automatically
        col_geom.geometry_type = GEOM_CYLINDER
        assert not any(m.type == "DECIMATE" for m in cylinder_mesh.modifiers)

        # Export URDF
        export_path = tmp_path / "multi_geom_bot.urdf"

        class MockExportOp:
            filepath = str(export_path)

            def report(self, level, message):
                pass

        res_exp = LINKFORGE_OT_export_robot_model.execute(MockExportOp(), bpy.context)
        assert res_exp == {"FINISHED"}
        assert export_path.exists()

        urdf_text = export_path.read_text()
        assert "<sphere" in urdf_text
        assert "<cylinder" in urdf_text

        # Re-import URDF and verify geometry preservation
        cleanup_blender_scene(scene)

        class MockImportOp:
            filepath = str(export_path)

            def report(self, level, message):
                pass

        res_imp = LINKFORGE_OT_import_robot_model.execute(MockImportOp(), bpy.context)
        assert res_imp == {"FINISHED"}

        start_time = time.time()
        while time.time() - start_time < 10.0:
            safe_update()
            if not lf_scene.is_importing and len(bpy.data.objects) > 0:
                break
            time.sleep(0.1)

        imported_sphere = next(
            o
            for o in bpy.data.objects
            if "sphere" in o.name.lower() and getattr(o, PROP_GEOM, None)
        )
        assert getattr(imported_sphere, PROP_GEOM).geometry_type in (GEOM_SPHERE, GEOM_MESH)
