"""Unit tests for Blender Converter robustness and validation."""

from __future__ import annotations

from unittest import mock

import pytest
from linkforge.blender.adapters.blender_to_core import (
    detect_primitive_type,
    scene_to_robot,
)
from linkforge.blender.adapters.translator import JointTranslator, LinkTranslator
from linkforge_core.composer import RobotBuilder
from linkforge_core.exceptions import RobotValidationError, ValidationErrorCode

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
)


class TestConverterRobustness:
    def test_scene_to_robot_strict_mode(self, scene, blender_context) -> None:
        """Verify that strict_mode=True raises exceptions on conversion errors."""
        scene.linkforge.strict_mode = True
        root = create_test_object("Root", None, scene)
        safe_get_linkforge(root).is_robot_link = True

        with (
            mock.patch(
                "linkforge.blender.adapters.translator.LinkTranslator.translate",
                side_effect=RobotValidationError(ValidationErrorCode.INVALID_VALUE, "Link Fail"),
            ),
            pytest.raises(RobotValidationError),
        ):
            scene_to_robot(blender_context)

    def test_detect_primitive_type_robustness(self, scene, blender_context) -> None:
        """Test detect_primitive_type with invalid mesh edge cases."""
        # None object
        assert detect_primitive_type(None) is None

        # Empty object (no mesh)
        empty = create_test_object("Empty", None, scene)
        assert detect_primitive_type(empty) is None


class TestJointRobustness:
    def test_joint_custom_axis_fallback(self, scene, blender_context) -> None:
        """Test custom axis fallbacks when values are zero."""
        from unittest.mock import MagicMock

        p = create_test_object("Parent", None, scene)
        c = create_test_object("Child", None, scene)

        # Ensure mocks are fully initialized for Parent/Child
        for obj in [p, c]:
            if not hasattr(obj, "linkforge"):
                obj.linkforge = MagicMock()

        safe_get_linkforge(p).is_robot_link = True
        safe_get_linkforge(c).is_robot_link = True

        j = create_test_object("Joint", None, scene)
        # Establish hierarchy for converter
        j.parent = p
        c.parent = j

        # Ensure mock properties exist (defensive for unit tests)
        if not hasattr(j, "linkforge_joint"):
            j.linkforge_joint = MagicMock()

        props = safe_get_joint(j)
        assert props is not None, "Failed to initialize joint properties on mock object"
        props.is_robot_joint = True
        props.joint_type = "REVOLUTE"  # Must be REVOLUTE to test axis fallback
        props.parent_link = p
        props.child_link = c
        props.axis = "CUSTOM"
        props.custom_axis_x = 0.0
        props.custom_axis_y = 0.0
        props.custom_axis_z = 0.0

        builder = RobotBuilder("test_robot")
        lb_p = LinkTranslator().translate(p, builder, blender_context)
        if lb_p:
            lb_p.root()

        lb_c = builder.link("Child", parent="Parent")
        LinkTranslator().translate(c, builder, blender_context, lb=lb_c)
        JointTranslator().translate(j, builder, blender_context, lb=lb_c)
        if lb_c:
            lb_c.commit()

        core = builder.robot.get_joint("Joint")
        assert core is not None, "Joint 'Joint' not found in robot model"
        assert core.axis is not None, "Joint axis was not fell back to default"
        assert core.axis.z == 1.0  # Default fallback


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
