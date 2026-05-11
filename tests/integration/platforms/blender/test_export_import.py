"""Integration tests for Blender Export/Import roundtrip."""

from __future__ import annotations

import time
from pathlib import Path

import bpy
import pytest
from linkforge.blender.operators.export_ops import LINKFORGE_OT_export_robot_model
from linkforge.blender.operators.import_ops import LINKFORGE_OT_import_robot_model

from tests.blender_test_utils import (
    create_robot_joint,
    create_robot_link,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_linkforge_scene,
    safe_update,
)


class TestExportImportRoundtrip:
    def test_full_urdf_roundtrip(self, blender_clean_scene, tmp_path: Path) -> None:
        """Verify that a robot can be exported and re-imported with property parity."""
        scene = bpy.context.scene
        lf_scene = safe_get_linkforge_scene(scene)
        lf_scene.robot_name = "roundtrip_bot"
        lf_scene.export_format = "URDF"

        # 1. Setup a simple 2-link robot in Blender
        # base_link (at origin)
        base = create_robot_link("base_link", scene, with_visual=True)
        # child_link (offset)
        child = create_robot_link("child_link", scene, with_visual=True)
        child.location = (0, 0, 1.0)

        # Connect with a revolute joint
        joint = create_robot_joint("joint1", base, child, scene, joint_type="REVOLUTE")
        j_props = safe_get_joint(joint)
        j_props.limit_lower = -1.57
        j_props.limit_upper = 1.57

        safe_update()

        # 2. Export to URDF
        export_path = tmp_path / "robot.urdf"

        # We need to mock the operator's filepath since it uses ExportHelper
        class MockExportOp:
            filepath = str(export_path)

            def report(self, level, message):
                pass

        # Execute export
        res = LINKFORGE_OT_export_robot_model.execute(MockExportOp(), bpy.context)
        assert res == {"FINISHED"}
        assert export_path.exists()

        # 3. Clear scene and Re-import
        from tests.blender_test_utils import cleanup_blender_scene

        cleanup_blender_scene(scene)
        assert len(bpy.data.objects) == 0

        # Execute import
        class MockImportOp:
            filepath = str(export_path)

            def report(self, level, message):
                pass

        res = LINKFORGE_OT_import_robot_model.execute(MockImportOp(), bpy.context)
        assert res == {"FINISHED"}

        # 4. Wait for asynchronous import to complete
        start_time = time.time()
        timeout = 10.0
        while time.time() - start_time < timeout:
            safe_update()
            if not lf_scene.is_importing and len(bpy.data.objects) > 0:
                break
            time.sleep(0.1)

        assert not lf_scene.is_importing, "Import timed out"

        # 5. Verify the imported structure
        # In Blender, links are Empties named after the link
        # Visuals are children of the Empties
        assert bpy.data.objects.get("base_link") is not None
        assert bpy.data.objects.get("child_link") is not None
        assert bpy.data.objects.get("joint1") is not None

        new_base = bpy.data.objects["base_link"]
        new_child = bpy.data.objects["child_link"]
        new_joint = bpy.data.objects["joint1"]

        # Verify Link Properties
        assert safe_get_linkforge(new_base).is_robot_link is True
        assert safe_get_linkforge(new_child).is_robot_link is True

        # Verify Joint Properties
        nj_props = safe_get_joint(new_joint)
        assert nj_props.is_robot_joint is True
        assert nj_props.parent_link == new_base
        assert nj_props.child_link == new_child
        assert nj_props.joint_type == "REVOLUTE"
        assert abs(nj_props.limit_lower + 1.57) < 1e-5
        assert abs(nj_props.limit_upper - 1.57) < 1e-5

    def test_import_with_missing_meshes_graceful_failure(
        self, blender_clean_scene, tmp_path: Path
    ) -> None:
        """Verify that importing a URDF with missing mesh files doesn't crash Blender."""
        urdf_content = """<?xml version="1.0"?>
<robot name="broken_bot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://non_existent_pkg/mesh.stl"/>
      </geometry>
    </visual>
  </link>
</robot>
"""
        urdf_path = tmp_path / "broken.urdf"
        urdf_path.write_text(urdf_content)

        class MockImportOp:
            filepath = str(urdf_path)

            def report(self, level, message):
                pass

        # Should still return FINISHED as it starts the background process
        res = LINKFORGE_OT_import_robot_model.execute(MockImportOp(), bpy.context)
        assert res == {"FINISHED"}

        # Wait for it to finish (or fail)
        lf_scene = safe_get_linkforge_scene(bpy.context.scene)
        start_time = time.time()
        while time.time() - start_time < 5.0:
            safe_update()
            if not lf_scene.is_importing:
                break
            time.sleep(0.1)

        # The link should still exist as an Empty even if mesh failed
        assert "base_link" in bpy.data.objects
        assert bpy.data.objects["base_link"].type == "EMPTY"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
