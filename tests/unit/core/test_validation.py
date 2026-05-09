"""Unit tests for Model Validation."""

from __future__ import annotations

import pytest
from linkforge_core.exceptions import RobotValidationError, ValidationErrorCode
from linkforge_core.models import Joint, JointType, Link, Robot


def test_tree_structure_validation() -> None:
    """Test that disconnected links are detected."""
    robot = Robot(name="disconnected")
    robot.add_link(Link(name="base"))
    robot.add_link(Link(name="island"))

    # Should raise error because multiple roots exist (base and island)
    with pytest.raises(RobotValidationError) as exc:
        robot.get_root_link()
    assert exc.value.code == ValidationErrorCode.MULTIPLE_ROOTS


def test_cyclic_dependency_validation() -> None:
    """Test that cycles in joint graph are detected."""
    robot = Robot(name="cyclic")
    robot.add_link(Link(name="l1"))
    robot.add_link(Link(name="l2"))

    robot.add_joint(Joint(name="j1", type=JointType.FIXED, parent="l1", child="l2"))
    robot.add_joint(Joint(name="j2", type=JointType.FIXED, parent="l2", child="l1"))

    # has_cycle property should be true
    assert robot.has_cycle is True

    # get_root_link should raise NO_ROOT if it's a pure cycle (l1 has parent l2, l2 has parent l1)
    with pytest.raises(RobotValidationError) as exc:
        robot.get_root_link()
    assert exc.value.code == ValidationErrorCode.NO_ROOT


def test_multiple_root_links_validation() -> None:
    """Test that multiple links without parents (roots) are detected."""
    robot = Robot(name="multi_root")
    robot.add_link(Link(name="root1"))
    robot.add_link(Link(name="root2"))

    with pytest.raises(RobotValidationError) as exc:
        robot.get_root_link()
    assert exc.value.code == ValidationErrorCode.MULTIPLE_ROOTS


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
