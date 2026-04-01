"""Unit tests for SRDF models."""

from linkforge_core.models.robot import Robot
from linkforge_core.models.srdf import (
    DisabledCollision,
    EndEffector,
    GroupState,
    PassiveJoint,
    PlanningGroup,
    SemanticRobotDescription,
    VirtualJoint,
)


def test_virtual_joint_creation():
    """Test creating a virtual joint."""
    vj = VirtualJoint(
        name="world_joint", type="fixed", parent_frame="world", child_link="base_link"
    )
    assert vj.name == "world_joint"
    assert vj.type == "fixed"
    assert vj.parent_frame == "world"
    assert vj.child_link == "base_link"


def test_planning_group_creation():
    """Test creating a planning group with various components."""
    group = PlanningGroup(
        name="arm",
        links=["link1", "link2"],
        joints=["joint1", "joint2"],
        chains=[("base_link", "tool0")],
        subgroups=["hand"],
    )
    assert group.name == "arm"
    assert "link1" in group.links
    assert "joint1" in group.joints
    assert ("base_link", "tool0") in group.chains
    assert "hand" in group.subgroups


def test_group_state_creation():
    """Test creating a named group state (pose)."""
    state = GroupState(name="home", group="arm", joint_values={"joint1": 0.0, "joint2": 1.57})
    assert state.name == "home"
    assert state.group == "arm"
    assert state.joint_values["joint1"] == 0.0
    assert state.joint_values["joint2"] == 1.57


def test_end_effector_creation():
    """Test creating an end effector definition."""
    ee = EndEffector(name="hand", group="hand_group", parent_link="link4", parent_group="arm")
    assert ee.name == "hand"
    assert ee.parent_group == "arm"


def test_passive_joint_creation():
    """Test creating a passive joint definition."""
    pj = PassiveJoint(name="wheel_joint")
    assert pj.name == "wheel_joint"


def test_disabled_collision_creation():
    """Test creating a disabled collision pair."""
    dc = DisabledCollision(link1="link1", link2="link2", reason="adjacent")
    assert dc.link1 == "link1"
    assert dc.link2 == "link2"
    assert dc.reason == "adjacent"


def test_semantic_robot_description_container():
    """Test the full SRDF container."""
    srdf = SemanticRobotDescription(
        virtual_joints=[
            VirtualJoint(
                name="world_joint", type="fixed", parent_frame="world", child_link="base_link"
            )
        ],
        groups=[PlanningGroup(name="arm", joints=["joint1"])],
        group_states=[GroupState(name="home", group="arm", joint_values={"joint1": 0.0})],
    )
    assert len(srdf.virtual_joints) == 1
    assert len(srdf.groups) == 1
    assert len(srdf.group_states) == 1
    assert srdf.groups[0].name == "arm"


def test_robot_semantic_integration():
    """Test that SRDF data can be attached to a Robot model."""
    srdf = SemanticRobotDescription(groups=[PlanningGroup(name="arm")])

    # Test via initial_semantic
    robot = Robot(name="test_robot", initial_semantic=srdf)
    assert robot.semantic is not None
    assert len(robot.semantic.groups) == 1
    assert robot.semantic.groups[0].name == "arm"

    # Test via property setter
    robot.semantic = None
    assert robot.semantic is None

    new_srdf = SemanticRobotDescription(passive_joints=[PassiveJoint(name="pj")])
    robot.semantic = new_srdf
    assert robot.semantic.passive_joints[0].name == "pj"
