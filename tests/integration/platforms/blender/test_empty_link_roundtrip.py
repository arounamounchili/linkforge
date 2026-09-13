"""Integration test for empty link export/import roundtrip validation.

Verifies that creating an empty link frame (virtual link without geometry) jointed to
a geometric link passes validation, exports as a pure virtual link without inertial tag,
and remains valid without errors when imported back.
"""

from __future__ import annotations

import time
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
from linkforge.blender.operators.export_ops import (
    LINKFORGE_OT_export_robot_model,
    LINKFORGE_OT_validate_robot,
)
from linkforge.blender.operators.import_ops import LINKFORGE_OT_import_robot_model

from tests.blender_test_utils import (
    cleanup_blender_scene,
    create_robot_joint,
    create_robot_link,
    safe_get_linkforge_scene,
    safe_get_validation,
    safe_update,
)


class MockOperator:
    """Mock operator for file dialog / report handling."""

    def __init__(self, filepath: str = "") -> None:
        self.filepath = filepath

    def report(self, level: set[str], message: str) -> None:
        pass


class TestEmptyLinkRoundtrip:
    def test_empty_link_roundtrip_preserves_validity(
        self, blender_clean_scene, tmp_path: Path
    ) -> None:
        """Verify that an empty link jointed to a mesh link roundtrips without validation errors."""
        scene = bpy.context.scene
        lf_scene = safe_get_linkforge_scene(scene)
        lf_scene.robot_name = "test_empty_link_robot"
        lf_scene.export_format = "URDF"

        base_link = create_robot_link("base_link", scene, with_visual=False, with_collision=False)

        cube_link = create_robot_link("cube_link", scene, with_visual=True, with_collision=True)
        cube_link.location = (0, 0, 1.0)

        create_robot_joint("joint1", base_link, cube_link, scene, joint_type="revolute")
        safe_update()

        res = LINKFORGE_OT_validate_robot.execute(MockOperator(), bpy.context)
        assert res == {"FINISHED"}
        vm = safe_get_validation(bpy.context.window_manager)
        assert vm.is_valid is True
        assert vm.error_count == 0

        export_path = tmp_path / "robot.urdf"
        res = LINKFORGE_OT_export_robot_model.execute(MockOperator(str(export_path)), bpy.context)
        assert res == {"FINISHED"}
        urdf_text = export_path.read_text()
        root = ET.fromstring(urdf_text)
        base_elem = next(el for el in root.findall("link") if el.get("name") == "base_link")
        assert base_elem.find("inertial") is None
        cube_elem = next(el for el in root.findall("link") if el.get("name") == "cube_link")
        assert cube_elem.find("inertial") is not None

        cleanup_blender_scene(scene)
        assert len(bpy.data.objects) == 0

        res = LINKFORGE_OT_import_robot_model.execute(MockOperator(str(export_path)), bpy.context)
        assert res == {"FINISHED"}

        start_time = time.time()
        while time.time() - start_time < 10.0:
            safe_update()
            if not lf_scene.is_importing and len(bpy.data.objects) > 0:
                break
            time.sleep(0.05)

        assert not lf_scene.is_importing, "Import timed out"
        assert "base_link" in bpy.data.objects
        assert "cube_link" in bpy.data.objects

        res = LINKFORGE_OT_validate_robot.execute(MockOperator(), bpy.context)
        assert res == {"FINISHED"}
        vm = safe_get_validation(bpy.context.window_manager)
        assert vm.is_valid is True, f"Validation failed with errors: {[e.title for e in vm.errors]}"
        assert vm.error_count == 0
