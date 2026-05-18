"""Unit tests for SemanticBuilder."""

from __future__ import annotations

from linkforge.core import RobotBuilder
from linkforge.core.models.srdf import Chain, SrdfSphere


def test_semantic_builder_all_methods() -> None:
    builder = RobotBuilder("test_robot")

    # 1. group
    # A. standard group with links, joints, chains, subgroups
    builder.semantic.group(
        name="arm",
        links=["link1", "link2"],
        joints=["joint1"],
        chains=[Chain(base_link="base", tip_link="tip")],
        subgroups=["hand"],
    )
    # B. group with base_link and tip_link shorthand
    builder.semantic.group(
        name="leg",
        base_link="base",
        tip_link="tip",
    )

    # 2. group_state
    # Dictionary with float and tuple of floats
    builder.semantic.group_state(
        name="home",
        group="arm",
        values={"joint1": 0.0, "joint2": (1.0, 2.0)},
    )

    # 3. end_effector
    builder.semantic.end_effector(
        name="gripper",
        group="hand",
        parent_link="link2",
        parent_group="arm",
    )

    # 4. passive_joint
    builder.semantic.passive_joint(name="joint3")

    # 5. virtual_joint
    builder.semantic.virtual_joint(
        name="virtual_joint",
        child_link="link1",
        parent_frame="world",
        joint_type="fixed",
    )

    # 6. disable_collisions
    builder.semantic.disable_collisions(
        link1="link1",
        link2="link2",
        reason="Adjacent",
    )

    # 7. enable_collisions
    builder.semantic.enable_collisions(
        link1="link1",
        link2="link3",
        reason="User",
    )

    # 8. disable_default_collisions
    builder.semantic.disable_default_collisions(link="link1")

    # 9. joint_property
    builder.semantic.joint_property(
        joint_name="joint1",
        property_name="stiffness",
        value="100.0",
    )

    # 10. approximate_link_collision
    sphere1 = SrdfSphere(center_x=0.0, center_y=0.0, center_z=0.0, radius=0.1)
    builder.semantic.approximate_link_collision(
        link="link1",
        spheres=[sphere1],
    )

    robot = builder.build(validate=False)
    semantic = robot.semantic

    # Assert group A
    assert len(semantic.groups) == 2
    g1 = semantic.groups[0]
    assert g1.name == "arm"
    assert g1.links == ("link1", "link2")
    assert g1.joints == ("joint1",)
    assert len(g1.chains) == 1
    assert g1.chains[0].base_link == "base"
    assert g1.chains[0].tip_link == "tip"
    assert g1.subgroups == ("hand",)

    # Assert group B
    g2 = semantic.groups[1]
    assert g2.name == "leg"
    assert len(g2.chains) == 1
    assert g2.chains[0].base_link == "base"
    assert g2.chains[0].tip_link == "tip"

    # Assert group_state
    assert len(semantic.group_states) == 1
    gs = semantic.group_states[0]
    assert gs.name == "home"
    assert gs.group == "arm"
    assert gs.joint_values["joint1"] == (0.0,)
    assert gs.joint_values["joint2"] == (1.0, 2.0)

    # Assert end_effector
    assert len(semantic.end_effectors) == 1
    ee = semantic.end_effectors[0]
    assert ee.name == "gripper"
    assert ee.group == "hand"
    assert ee.parent_link == "link2"
    assert ee.parent_group == "arm"

    # Assert passive_joint
    assert len(semantic.passive_joints) == 1
    pj = semantic.passive_joints[0]
    assert pj.name == "joint3"

    # Assert virtual_joint
    assert len(semantic.virtual_joints) == 1
    vj = semantic.virtual_joints[0]
    assert vj.name == "virtual_joint"
    assert vj.child_link == "link1"
    assert vj.parent_frame == "world"
    assert vj.type == "fixed"

    # Assert disabled collisions
    assert len(semantic.disabled_collisions) == 1
    dc = semantic.disabled_collisions[0]
    assert dc.link1 == "link1"
    assert dc.link2 == "link2"
    assert dc.reason == "Adjacent"

    # Assert enabled collisions
    assert len(semantic.enabled_collisions) == 1
    ec = semantic.enabled_collisions[0]
    assert ec.link1 == "link1"
    assert ec.link2 == "link3"
    assert ec.reason == "User"

    # Assert disable default collisions
    assert len(semantic.no_default_collision_links) == 1
    assert semantic.no_default_collision_links[0] == "link1"

    # Assert joint property
    assert len(semantic.joint_properties) == 1
    jp = semantic.joint_properties[0]
    assert jp.joint_name == "joint1"
    assert jp.property_name == "stiffness"
    assert jp.value == "100.0"

    # Assert link sphere approximations
    assert len(semantic.link_sphere_approximations) == 1
    lsa = semantic.link_sphere_approximations[0]
    assert lsa.link == "link1"
    assert len(lsa.spheres) == 1
    assert lsa.spheres[0].radius == 0.1
