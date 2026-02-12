"""Unit tests for core kinematics utilities."""

from __future__ import annotations

from linkforge_core.models import Joint, JointType, Link
from linkforge_core.utils.kinematics import sort_joints_topological


def test_sort_joints_topological():
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

    sorted_joints = sort_joints_topological(joints, links)

    # j1 MUST come before j2 and j3
    assert sorted_joints[0].name == "j1"
    assert {sorted_joints[1].name, sorted_joints[2].name} == {"j2", "j3"}
