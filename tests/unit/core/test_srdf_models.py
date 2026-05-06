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


def test_srdf_submodels_prefix() -> None:
    """Test prefixing for all SRDF sub-models."""
    vj = VirtualJoint(name="vj", child_link="l1", parent_frame="f1", type="fixed")
    assert vj.with_prefix("p_").name == "p_vj"
    assert vj.with_prefix("p_").child_link == "p_l1"

    gs = GroupState(name="gs", group="g1", joint_values={"j1": 0.5})
    pre_gs = gs.with_prefix("p_")
    assert pre_gs.name == "p_gs"
    assert pre_gs.group == "p_g1"
    assert "p_j1" in pre_gs.joint_values

    ee = EndEffector(name="ee", group="g1", parent_link="l1", parent_group="pg")
    pre_ee = ee.with_prefix("p_")
    assert pre_ee.name == "p_ee"
    assert pre_ee.group == "p_g1"
    assert pre_ee.parent_link == "p_l1"
    assert pre_ee.parent_group == "p_pg"

    pj = PassiveJoint(name="pj")
    assert pj.with_prefix("p_").name == "p_pj"

    cp = CollisionPair(link1="l1", link2="l2")
    pre_cp = cp.with_prefix("p_")
    assert pre_cp.link1 == "p_l1"
    assert pre_cp.link2 == "p_l2"

    chain = Chain(base_link="b1", tip_link="t1")
    pre_chain = chain.with_prefix("p_")
    assert pre_chain.base_link == "p_b1"
    assert pre_chain.tip_link == "p_t1"

    pg = PlanningGroup(name="g1", links=("l1",), joints=("j1",), subgroups=("s1",), chains=(chain,))
    pre_pg = pg.with_prefix("p_")
    assert pre_pg.name == "p_g1"
    assert "p_l1" in pre_pg.links
    assert "p_j1" in pre_pg.joints
    assert "p_s1" in pre_pg.subgroups
    assert pre_pg.chains[0].base_link == "p_b1"

    lsa = LinkSphereApproximation(link="l1", spheres=())
    assert lsa.with_prefix("p_").link == "p_l1"

    jp = JointProperty(joint_name="j1", property_name="pn", value="v")
    assert jp.with_prefix("p_").joint_name == "p_j1"


def test_semantic_description_prefix() -> None:
    """Test prefixing for the full SRDF container."""
    srdf = SemanticRobotDescription(
        robot_name="r1",
        groups=(PlanningGroup(name="g1", links=("l1",)),),
        virtual_joints=(VirtualJoint(name="vj", child_link="l1", parent_frame="f", type="fixed"),),
    )
    pre = srdf.with_prefix("arm_")
    assert pre.robot_name == "arm_r1"
    assert pre.groups[0].name == "arm_g1"
    assert pre.virtual_joints[0].name == "arm_vj"


def test_semantic_description_merge() -> None:
    """Test merging and deduplication of SRDF containers."""
    srdf1 = SemanticRobotDescription(
        groups=(PlanningGroup(name="g1", links=("l1",)),),
        disabled_collisions=(CollisionPair(link1="l1", link2="l2"),),
    )
    srdf2 = SemanticRobotDescription(
        groups=(PlanningGroup(name="g1", links=("l1",)), PlanningGroup(name="g2", links=("l2",))),
        disabled_collisions=(
            CollisionPair(link1="l1", link2="l2"),
            CollisionPair(link1="l2", link2="l3"),
        ),
    )

    merged = srdf1.merge_with(srdf2)
    # Deduplication check
    assert len(merged.groups) == 2
    assert {g.name for g in merged.groups} == {"g1", "g2"}
    assert len(merged.disabled_collisions) == 2


def test_semantic_description_merge_comprehensive() -> None:
    """Test merging and deduplication across all SRDF collection types."""
    srdf1 = SemanticRobotDescription(
        no_default_collision_links=("l1",),
        link_sphere_approximations=(LinkSphereApproximation(link="l1", spheres=()),),
        joint_properties=(JointProperty(joint_name="j1", property_name="p1", value="v1"),),
        disabled_collisions=(CollisionPair(link1="a", link2="b"),),
    )

    srdf2 = SemanticRobotDescription(
        no_default_collision_links=("l1", "l2"),
        link_sphere_approximations=(
            LinkSphereApproximation(link="l1", spheres=()),
            LinkSphereApproximation(link="l2", spheres=()),
        ),
        joint_properties=(
            JointProperty(joint_name="j1", property_name="p1", value="v1"),
            JointProperty(joint_name="j1", property_name="p2", value="v2"),
        ),
        # Test symmetric deduplication (a-b vs b-a)
        disabled_collisions=(CollisionPair(link1="b", link2="a"),),
    )

    merged = srdf1.merge_with(srdf2)

    # Check specialized collections
    assert len(merged.no_default_collision_links) == 2
    assert "l1" in merged.no_default_collision_links
    assert "l2" in merged.no_default_collision_links

    assert len(merged.link_sphere_approximations) == 2
    assert {lsa.link for lsa in merged.link_sphere_approximations} == {"l1", "l2"}

    assert len(merged.joint_properties) == 2
    assert {(jp.joint_name, jp.property_name) for jp in merged.joint_properties} == {
        ("j1", "p1"),
        ("j1", "p2"),
    }

    # Check symmetric collision deduplication
    # Even though srdf2 had (b, a), it is a duplicate of (a, b)
    assert len(merged.disabled_collisions) == 1
    assert merged.disabled_collisions[0].link1 == "a"
