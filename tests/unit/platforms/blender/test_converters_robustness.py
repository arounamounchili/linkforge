"""Unit tests for Blender Converter robustness and validation."""

from __future__ import annotations

from unittest import mock

import pytest
from linkforge.blender.adapters.blender_to_core import (
    blender_joint_to_core,
    detect_primitive_type,
    scene_to_robot,
)
from linkforge_core.exceptions import RobotValidationError, ValidationErrorCode

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
)

# =============================================================================
# Conversion Robustness
# =============================================================================


class TestConverterRobustness:
    def test_scene_to_robot_strict_mode(self, mock_context, scene, blender_context) -> None:
        """Verify that strict_mode=True raises exceptions on conversion errors."""
        scene.linkforge.strict_mode = True
        root = create_test_object("Root", None, scene)
        safe_get_linkforge(root).is_robot_link = True

        with (
            mock.patch(
                "linkforge.blender.adapters.blender_to_core.blender_link_to_core_with_origin",
                side_effect=RobotValidationError(ValidationErrorCode.INVALID_VALUE, "Link Fail"),
            ),
            pytest.raises(RobotValidationError),
        ):
            scene_to_robot(mock_context)

    def test_detect_primitive_type_robustness(self, scene, blender_context) -> None:
        """Test detect_primitive_type with invalid mesh edge cases."""
        # None object
        assert detect_primitive_type(None) is None

        # Empty object (no mesh)
        empty = create_test_object("Empty", None, scene)
        assert detect_primitive_type(empty) is None


# =============================================================================
# Joint Conversion Edge Cases
# =============================================================================


class TestJointRobustness:
    def test_joint_custom_axis_fallback(self, scene, blender_context) -> None:
        """Test custom axis fallbacks when values are zero."""
        p = create_test_object("Parent", None, scene)
        c = create_test_object("Child", None, scene)
        safe_get_linkforge(p).is_robot_link = True
        safe_get_linkforge(c).is_robot_link = True

        j = create_test_object("Joint", None, scene)
        props = safe_get_joint(j)
        props.is_robot_joint = True
        props.parent_link = p
        props.child_link = c
        props.axis = "CUSTOM"
        props.custom_axis_x = 0.0
        props.custom_axis_y = 0.0
        props.custom_axis_z = 0.0

        core = blender_joint_to_core(j)
        assert core.axis.z == 1.0  # Default fallback


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
