from unittest.mock import MagicMock, patch

from linkforge_core.composer.naming import (
    add_joint_with_renaming,
    add_link_with_renaming,
)
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models import Joint, JointType, Link, Robot


class TestNamingUtilities:
    def test_add_link_with_renaming_no_collision(self) -> None:
        """Test adding a link when no collision exists."""
        robot = Robot(name="test_robot")
        link = Link(name="base_link")

        add_link_with_renaming(robot, link)
        assert "base_link" in robot._link_index

    def test_add_link_with_renaming_collision(self) -> None:
        """Test renaming a link on collision."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link_1"))

        # Should rename link_1 to link_1_duplicate_1
        add_link_with_renaming(robot, Link(name="link_1"))
        assert "link_1_duplicate_1" in robot._link_index

    def test_add_link_unexpected_error(self) -> None:
        """Test handling unexpected RobotModelError (non-collision)."""
        robot = MagicMock(spec=Robot)
        robot.add_link.side_effect = RobotModelError("Fatal error")

        with patch("linkforge_core.composer.naming.logger") as mock_logger:
            add_link_with_renaming(robot, Link(name="l"))
            assert mock_logger.warning.called
            assert "Fatal error" in mock_logger.warning.call_args[0][0]

    def test_add_joint_with_renaming(self) -> None:
        """Test adding a joint with renaming support."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="l1"))
        robot.add_link(Link(name="l2"))

        joint = Joint(name="j1", parent="l1", child="l2", type=JointType.FIXED)
        robot.add_joint(joint)

        # Test collision on joint name
        new_joint = Joint(name="j1", parent="l1", child="l2", type=JointType.FIXED)
        add_joint_with_renaming(robot, new_joint)
        assert "j1_duplicate_1" in robot._joint_index

    def test_add_joint_fallback_name(self) -> None:
        """Test add_joint_with_renaming using fallback name in logs."""
        robot = Robot(name="test_robot")
        # Use a mock joint to bypass validation of empty name
        joint = MagicMock()
        joint.name = ""

        # Simulate RobotModelError when adding
        robot.add_joint = MagicMock(side_effect=RobotModelError("Invalid link"))

        with patch("linkforge_core.composer.naming.logger") as mock_logger:
            add_joint_with_renaming(robot, joint, fallback_name="original_joint")
            assert mock_logger.warning.called
            # Check that fallback name is mentioned in at least one warning
            found = any(
                "original_joint" in str(args[0]) for args, _ in mock_logger.warning.call_args_list
            )
            assert found
