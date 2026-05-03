"""Unit tests for SRDF models."""

from linkforge_core.models.robot import Robot
from linkforge_core.models.srdf import (
    Chain,
    CollisionPair,
    EndEffector,
    GroupState,
    JointProperty,
    LinkSphereApproximation,
    PassiveJoint,
    PlanningGroup,
    SemanticRobotDescription,
    SrdfSphere,
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
        chains=[Chain(base_link="base_link", tip_link="tool0")],
        subgroups=["hand"],
    )
    assert group.name == "arm"
    assert "link1" in group.links
    assert "joint1" in group.joints
    assert group.chains[0].base_link == "base_link"
    assert group.chains[0].tip_link == "tool0"
    assert "hand" in group.subgroups


def test_group_state_creation():
    """Test creating a named group state (pose)."""
    state = GroupState(
        name="home", group="arm", joint_values={"joint1": 0.0, "joint2": 1.57, "joint3": (1.0, 2.0)}
    )
    assert state.name == "home"
    assert state.group == "arm"
    assert state.joint_values["joint1"] == (0.0,)
    assert state.joint_values["joint2"] == (1.57,)
    assert state.joint_values["joint3"] == (1.0, 2.0)


def test_end_effector_creation():
    """Test creating an end effector definition."""
    ee = EndEffector(name="hand", group="hand_group", parent_link="link4", parent_group="arm")
    assert ee.name == "hand"
    assert ee.parent_group == "arm"


def test_passive_joint_creation():
    """Test creating a passive joint definition."""
    pj = PassiveJoint(name="wheel_joint")
    assert pj.name == "wheel_joint"


def test_collision_pair_creation():
    """Test creating a collision pair."""
    cp = CollisionPair(link1="link1", link2="link2", reason="adjacent")
    assert cp.link1 == "link1"
    assert cp.link2 == "link2"
    assert cp.reason == "adjacent"


def test_link_sphere_approximation_creation():
    """Test creating link sphere approximations."""
    sphere = SrdfSphere(center_x=1.0, center_y=2.0, center_z=3.0, radius=0.5)
    lsa = LinkSphereApproximation(link="link1", spheres=[sphere])
    assert lsa.link == "link1"
    assert len(lsa.spheres) == 1
    assert lsa.spheres[0].radius == 0.5
    assert lsa.spheres[0].center_x == 1.0


def test_joint_property_creation():
    """Test creating a joint property."""
    jp = JointProperty(joint_name="joint1", property_name="friction", value="0.5")
    assert jp.joint_name == "joint1"
    assert jp.property_name == "friction"
    assert jp.value == "0.5"


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
    srdf = SemanticRobotDescription(groups=[PlanningGroup(name="arm", joints=["joint1"])])

    # Test via initial_semantic
    robot = Robot(name="test_robot", initial_semantic=srdf)
    assert robot.semantic is not None
    assert len(robot.semantic.groups) == 1
    assert robot.semantic.groups[0].name == "arm"

    # Test via property setter
    robot.semantic = None
    assert len(robot.semantic.groups) == 0

    new_srdf = SemanticRobotDescription(passive_joints=[PassiveJoint(name="pj")])
    robot.semantic = new_srdf
    assert robot.semantic.passive_joints[0].name == "pj"
