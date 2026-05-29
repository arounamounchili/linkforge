"""Tests for SemanticConsistencyCheck (cross-layer semantic linter).

Each test targets exactly one of the five validation rules, with a clear
"invalid" case that produces the expected error/warning, and a "valid" case
that produces no issues for that rule.
"""

from __future__ import annotations

from linkforge.core.models.geometry import Vector3
from linkforge.core.models.joint import Joint, JointLimits, JointType
from linkforge.core.models.link import Link
from linkforge.core.models.robot import Robot
from linkforge.core.models.ros2_control import Ros2Control, Ros2ControlJoint
from linkforge.core.models.srdf import (
    Chain,
    CollisionPair,
    EndEffector,
    GroupState,
    PassiveJoint,
    PlanningGroup,
    SemanticRobotDescription,
)
from linkforge.core.validation.checks import SemanticConsistencyCheck
from linkforge.core.validation.result import ValidationResult

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _run(robot: Robot) -> ValidationResult:
    """Run SemanticConsistencyCheck only and return the result."""
    result = ValidationResult(robot_name=robot.name)
    SemanticConsistencyCheck().run(robot, result)
    return result


_AXIS = Vector3(0, 0, 1)  # Default axis for revolute/continuous joints in tests


def _make_arm_robot() -> Robot:
    """Create a minimal 2-joint arm for reuse across tests."""
    robot = Robot(name="arm")
    robot.add_link(Link(name="base_link"))
    robot.add_link(Link(name="link_1"))
    robot.add_link(Link(name="link_2"))
    robot.add_joint(
        Joint(
            name="joint_1",
            type=JointType.REVOLUTE,
            parent="base_link",
            child="link_1",
            axis=_AXIS,
            limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=2.0),
        )
    )
    robot.add_joint(
        Joint(
            name="joint_2",
            type=JointType.REVOLUTE,
            parent="link_1",
            child="link_2",
            axis=_AXIS,
            limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=2.0),
        )
    )
    return robot


# ===========================================================================
# Rule 1 — GroupState joint values within kinematic joint limits
# ===========================================================================


class TestGroupStateJointRange:
    def test_joint_value_exceeds_upper_limit_produces_error(self) -> None:
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[PlanningGroup(name="arm", joints=["joint_1"])],
            group_states=[
                GroupState(
                    name="too_far",
                    group="arm",
                    joint_values={"joint_1": 3.14},  # > upper limit of 1.57
                )
            ],
        )
        result = _run(robot)
        assert not result.is_valid
        error_titles = [e.title for e in result.errors]
        assert any("GroupState joint value out of range" in t for t in error_titles)

    def test_joint_value_below_lower_limit_produces_error(self) -> None:
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[PlanningGroup(name="arm", joints=["joint_1"])],
            group_states=[
                GroupState(
                    name="too_negative",
                    group="arm",
                    joint_values={"joint_1": -3.14},  # < lower limit of -1.57
                )
            ],
        )
        result = _run(robot)
        assert not result.is_valid
        assert any("GroupState joint value out of range" in e.title for e in result.errors)

    def test_joint_value_within_limits_is_valid(self) -> None:
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[PlanningGroup(name="arm", joints=["joint_1"])],
            group_states=[
                GroupState(
                    name="home",
                    group="arm",
                    joint_values={"joint_1": 0.0},  # within [-1.57, 1.57]
                )
            ],
        )
        result = _run(robot)
        assert not any("GroupState joint value out of range" in e.title for e in result.errors)

    def test_joint_without_limits_skips_range_check(self) -> None:
        """A continuous joint with no limits must not be range-checked."""
        robot = Robot(name="cont_arm")
        robot.add_link(Link(name="base"))
        robot.add_link(Link(name="link1"))
        robot.add_joint(
            Joint(
                name="continuous_j",
                type=JointType.CONTINUOUS,
                parent="base",
                child="link1",
                axis=_AXIS,
            )
        )
        robot.semantic = SemanticRobotDescription(
            groups=[PlanningGroup(name="g", joints=["continuous_j"])],
            group_states=[
                GroupState(
                    name="any_pose",
                    group="g",
                    joint_values={"continuous_j": 999.0},  # no limits → skip
                )
            ],
        )
        result = _run(robot)
        assert not any("GroupState joint value out of range" in e.title for e in result.errors)


# ===========================================================================
# Rule 2 — Chain reachability
# ===========================================================================


class TestChainReachability:
    def test_unreachable_chain_produces_error(self) -> None:
        """A chain whose tip is not a descendant of its base must fail."""
        robot = _make_arm_robot()
        # base_link → link_1 → link_2, so base_link→link_2 is valid,
        # but link_2→base_link is NOT (going upward)
        robot.semantic = SemanticRobotDescription(
            groups=[
                PlanningGroup(
                    name="bad_chain",
                    chains=[Chain(base_link="link_2", tip_link="base_link")],
                )
            ],
        )
        result = _run(robot)
        assert not result.is_valid
        assert any("Unreachable kinematic chain" in e.title for e in result.errors)

    def test_valid_chain_passes(self) -> None:
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[
                PlanningGroup(
                    name="arm_chain",
                    chains=[Chain(base_link="base_link", tip_link="link_2")],
                )
            ],
        )
        result = _run(robot)
        assert not any("Unreachable kinematic chain" in e.title for e in result.errors)

    def test_nonexistent_links_in_chain_produce_error(self) -> None:
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[
                PlanningGroup(
                    name="ghost_chain",
                    chains=[Chain(base_link="ghost_base", tip_link="ghost_tip")],
                )
            ],
        )
        result = _run(robot)
        assert not result.is_valid
        assert any("Unreachable kinematic chain" in e.title for e in result.errors)


# ===========================================================================
# Rule 3 — EndEffector parent link in group
# ===========================================================================


class TestEndEffectorParentInGroup:
    def test_end_effector_parent_not_in_group_produces_warning(self) -> None:
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[
                PlanningGroup(name="arm", links=["link_1"]),  # only link_1 listed
            ],
            end_effectors=[
                EndEffector(
                    name="gripper",
                    group="arm",
                    parent_link="link_2",  # NOT in group.links
                )
            ],
        )
        result = _run(robot)
        assert any("End effector parent link not in group" in w.title for w in result.warnings)

    def test_end_effector_parent_in_group_is_valid(self) -> None:
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[
                PlanningGroup(name="arm", links=["link_1", "link_2"]),
            ],
            end_effectors=[
                EndEffector(
                    name="gripper",
                    group="arm",
                    parent_link="link_2",  # IS in group.links
                )
            ],
        )
        result = _run(robot)
        assert not any("End effector parent link not in group" in w.title for w in result.warnings)

    def test_group_with_only_joints_skips_parent_link_check(self) -> None:
        """If group.links is empty (joints-only group), skip Rule 3 for that group."""
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[
                PlanningGroup(name="arm", joints=["joint_1"]),  # no explicit links
            ],
            end_effectors=[EndEffector(name="gripper", group="arm", parent_link="link_2")],
        )
        result = _run(robot)
        # Empty links set → check is skipped, no warning
        assert not any("End effector parent link not in group" in w.title for w in result.warnings)


# ===========================================================================
# Rule 4 — Passive joint vs. command_interface contradiction
# ===========================================================================


class TestPassiveCommandContradiction:
    def test_passive_joint_with_command_interface_is_error(self) -> None:
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[PlanningGroup(name="arm", joints=["joint_1", "joint_2"])],
            passive_joints=[PassiveJoint(name="joint_2")],
        )
        rc = Ros2Control(
            name="arm_ctrl",
            hardware_plugin="gazebo_ros2_control/GazeboSystem",
            joints=[
                Ros2ControlJoint(
                    name="joint_2",
                    command_interfaces=["position"],  # contradicts PassiveJoint
                    state_interfaces=["position"],
                )
            ],
        )
        robot.add_ros2_control(rc)
        result = _run(robot)
        assert not result.is_valid
        assert any("Passive joint has command interface" in e.title for e in result.errors)

    def test_passive_joint_with_only_state_interface_is_valid(self) -> None:
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[PlanningGroup(name="arm", joints=["joint_1", "joint_2"])],
            passive_joints=[PassiveJoint(name="joint_2")],
        )
        rc = Ros2Control(
            name="arm_ctrl",
            hardware_plugin="gazebo_ros2_control/GazeboSystem",
            joints=[
                Ros2ControlJoint(
                    name="joint_2",
                    command_interfaces=[],  # no command_interface → valid
                    state_interfaces=["position"],
                )
            ],
        )
        robot.add_ros2_control(rc)
        result = _run(robot)
        assert not any("Passive joint has command interface" in e.title for e in result.errors)

    def test_no_passive_joints_skips_check(self) -> None:
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[PlanningGroup(name="arm", joints=["joint_1"])],
        )
        result = _run(robot)
        assert not any("Passive joint has command interface" in e.title for e in result.errors)


# ===========================================================================
# Rule 5 — Collision pair link existence
# ===========================================================================


class TestCollisionPairLinkExistence:
    def test_disabled_collision_with_missing_link_produces_error(self) -> None:
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[PlanningGroup(name="arm", joints=["joint_1"])],
            disabled_collisions=[
                CollisionPair(link1="base_link", link2="nonexistent_link", reason="Adjacent")
            ],
        )
        result = _run(robot)
        assert not result.is_valid
        assert any("Collision rule references missing link" in e.title for e in result.errors)

    def test_enabled_collision_with_missing_link_produces_error(self) -> None:
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[PlanningGroup(name="arm", joints=["joint_1"])],
            enabled_collisions=[CollisionPair(link1="ghost_link", link2="link_1")],
        )
        result = _run(robot)
        assert not result.is_valid
        assert any("Collision rule references missing link" in e.title for e in result.errors)

    def test_valid_collision_pair_passes(self) -> None:
        robot = _make_arm_robot()
        robot.semantic = SemanticRobotDescription(
            groups=[PlanningGroup(name="arm", joints=["joint_1"])],
            disabled_collisions=[
                CollisionPair(link1="base_link", link2="link_1", reason="Adjacent")
            ],
        )
        result = _run(robot)
        assert not any("Collision rule references missing link" in e.title for e in result.errors)


# ===========================================================================
# Integration — A fully valid model produces no cross-layer errors
# ===========================================================================


class TestFullModelPasses:
    def test_complete_valid_model_has_no_consistency_errors(self) -> None:
        """End-to-end test: a well-formed robot with SRDF and ros2_control passes all rules."""
        robot = _make_arm_robot()

        robot.semantic = SemanticRobotDescription(
            groups=[
                PlanningGroup(
                    name="arm",
                    links=["link_1", "link_2"],
                    joints=["joint_1", "joint_2"],
                    chains=[Chain(base_link="base_link", tip_link="link_2")],
                )
            ],
            group_states=[
                GroupState(name="home", group="arm", joint_values={"joint_1": 0.0, "joint_2": 0.0}),
                GroupState(
                    name="reach",
                    group="arm",
                    joint_values={"joint_1": 1.0, "joint_2": -1.0},
                ),
            ],
            end_effectors=[EndEffector(name="gripper", group="arm", parent_link="link_2")],
            disabled_collisions=[
                CollisionPair(link1="base_link", link2="link_1", reason="Adjacent"),
                CollisionPair(link1="link_1", link2="link_2", reason="Adjacent"),
            ],
        )

        rc = Ros2Control(
            name="arm_ctrl",
            hardware_plugin="gazebo_ros2_control/GazeboSystem",
            joints=[
                Ros2ControlJoint(
                    name="joint_1",
                    command_interfaces=["position"],
                    state_interfaces=["position", "velocity"],
                ),
                Ros2ControlJoint(
                    name="joint_2",
                    command_interfaces=["position"],
                    state_interfaces=["position", "velocity"],
                ),
            ],
        )
        robot.add_ros2_control(rc)

        result = _run(robot)
        assert result.is_valid, (
            f"Expected no errors, got: {[e.title + ': ' + e.message for e in result.errors]}"
        )

    def test_empty_robot_skips_all_rules(self) -> None:
        """An empty robot (no links) must not produce any check-internal errors."""
        robot = Robot(name="empty")
        result = _run(robot)
        assert result.is_valid
