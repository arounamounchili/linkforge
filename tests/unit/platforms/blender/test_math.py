"""Unit tests for Blender Math, Transforms, and Rotation normalization."""

from __future__ import annotations

import pytest
from linkforge.blender.utils.transform_utils import (
    clear_parent_keep_transform,
    set_parent_keep_transform,
)

from tests.blender_test_utils import create_test_object

# Transform Utilities


class TestTransformUtilities:
    def test_set_parent_keep_transform(self, scene, blender_context) -> None:
        """Test parenting while preserving world transform."""
        parent_obj = create_test_object("Parent", None, scene)
        parent_obj.location = (1, 2, 3)

        child_obj = create_test_object("Child", None, scene)
        child_obj.location = (5, 6, 7)

        original_world_loc = child_obj.matrix_world.translation.copy()

        set_parent_keep_transform(child_obj, parent_obj)

        assert child_obj.parent == parent_obj
        assert child_obj.matrix_world.translation.x == pytest.approx(original_world_loc.x, abs=1e-4)

    def test_clear_parent_keep_transform(self, scene, blender_context) -> None:
        """Test clearing parent while preserving world transform."""
        parent_obj = create_test_object("ParentClear", None, scene)
        parent_obj.location = (2, 3, 4)

        child_obj = create_test_object("ChildClear", None, scene)
        child_obj.location = (5, 6, 7)
        child_obj.parent = parent_obj

        original_world_loc = child_obj.matrix_world.translation.copy()

        clear_parent_keep_transform(child_obj)

        assert child_obj.parent is None
        assert child_obj.matrix_world.translation.x == pytest.approx(original_world_loc.x, abs=1e-4)


# Rotation Normalization


class TestRotationNormalization:
    def test_rotation_mode_normalization(self, scene, blender_context) -> None:
        """Verify that adding a new link frame forces XYZ rotation mode."""
        from tests.blender_test_utils import create_robot_link

        obj = create_robot_link("test_rotation_link", scene)
        assert obj is not None
        assert obj.rotation_mode == "XYZ"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
