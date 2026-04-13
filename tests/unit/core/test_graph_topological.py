"""Unit tests for KinematicGraph topological sorting."""

from __future__ import annotations

import pytest
from linkforge_core.exceptions import RobotValidationError
from linkforge_core.models import Joint, JointType, KinematicGraph, Link


def test_get_topological_joints() -> None:
    """Test that joints are sorted correctly (parents before children)."""
    # Create links
    base = Link(name="base")
    link1 = Link(name="link1")
    link2 = Link(name="link2")
    link3 = Link(name="link3")
    links = [base, link1, link2, link3]

    # Create joints (out of order)
    # base -> link1
    # link1 -> link2
    # link1 -> link3
    j2 = Joint(name="j2", parent="link1", child="link2", type=JointType.FIXED)
    j1 = Joint(name="j1", parent="base", child="link1", type=JointType.FIXED)
    j3 = Joint(name="j3", parent="link1", child="link3", type=JointType.FIXED)

    joints = [j2, j1, j3]

    graph = KinematicGraph(links, joints)
    sorted_joints = graph.get_topological_joints()

    # j1 MUST come before j2 and j3
    assert sorted_joints[0].name == "j1"
    assert {sorted_joints[1].name, sorted_joints[2].name} == {"j2", "j3"}


def test_get_topological_joints_complex_tree() -> None:
    """Test topological sort with a deeper tree structure."""
    l0 = Link(name="l0")
    l1 = Link(name="l1")
    l2 = Link(name="l2")
    l3 = Link(name="l3")
    l4 = Link(name="l4")
    links = [l0, l1, l2, l3, l4]

    # l0 -> l1 -> l2
    # l0 -> l3 -> l4
    j4 = Joint(name="j4", parent="l3", child="l4", type=JointType.FIXED)
    j1 = Joint(name="j1", parent="l0", child="l1", type=JointType.FIXED)
    j3 = Joint(name="j3", parent="l0", child="l3", type=JointType.FIXED)
    j2 = Joint(name="j2", parent="l1", child="l2", type=JointType.FIXED)

    joints = [j4, j1, j3, j2]
    graph = KinematicGraph(links, joints)
    sorted_joints = graph.get_topological_joints()

    assert len(sorted_joints) == 4

    # Check dependencies
    indices = {j.name: i for i, j in enumerate(sorted_joints)}
    assert indices["j1"] < indices["j2"]  # l0->l1 before l1->l2
    assert indices["j3"] < indices["j4"]  # l0->l3 before l3->l4


def test_get_topological_joints_with_islands() -> None:
    """Test topological sort with disconnected robots (islands)."""
    # Robot 1: r1_base -> r1_link
    # Robot 2: r2_base -> r2_link
    l1 = Link(name="r1_base")
    l2 = Link(name="r1_link")
    l3 = Link(name="r2_base")
    l4 = Link(name="r2_link")

    j1 = Joint(name="j1", parent="r1_base", child="r1_link", type=JointType.FIXED)
    j2 = Joint(name="j2", parent="r2_base", child="r2_link", type=JointType.FIXED)

    graph = KinematicGraph([l1, l2, l3, l4], [j1, j2])
    sorted_joints = graph.get_topological_joints()

    assert len(sorted_joints) == 2
    # Since they are independent, any order is technically valid as long as
    # parent comes before child within each island.
    # In our implementation, they should both be present.
    joint_names = [j.name for j in sorted_joints]
    assert "j1" in joint_names
    assert "j2" in joint_names


def test_get_topological_joints_single_link() -> None:
    """Test topological sort with a robot that has no joints."""
    base = Link(name="base")
    graph = KinematicGraph([base], [])

    assert graph.get_topological_joints() == []
    assert graph.get_topological_link_names() == ["base"]


def test_get_topological_joints_cycle_error() -> None:
    """Test that get_topological_joints raises error on cycles."""
    l1 = Link(name="l1")
    l2 = Link(name="l2")

    # Cyclic dependency: l1 -> l2 -> l1
    j1 = Joint(name="j1", parent="l1", child="l2", type=JointType.FIXED)
    j2 = Joint(name="j2", parent="l2", child="l1", type=JointType.FIXED)

    graph = KinematicGraph([l1, l2], [j1, j2])

    with pytest.raises(RobotValidationError, match="cycles"):
        graph.get_topological_joints()

    with pytest.raises(RobotValidationError, match="cycles"):
        graph.get_topological_link_names()
