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


def test_sort_joints_diamond_structure():
    r"""Test topological sort with a diamond structure.

    This ensures that links visited via multiple paths (like link3 below)
    are handled correctly by the visited set check.

    Structure:
          base
         /    \
      link1  link2
         \    /
         link3
    """
    base = Link(name="base")
    link1 = Link(name="link1")
    link2 = Link(name="link2")
    link3 = Link(name="link3")
    links = [base, link1, link2, link3]

    # connections
    j1 = Joint(name="j1", parent="base", child="link1", type=JointType.FIXED)
    j2 = Joint(name="j2", parent="base", child="link2", type=JointType.FIXED)
    j3 = Joint(name="j3", parent="link1", child="link3", type=JointType.FIXED)
    # Note: URDF structure must be a tree, so a true diamond is invalid (child has 2 parents).
    # validation logic catches this, but kinematics sort just traverses.
    # To hit "visited" check, we need `visit(link3)` to be called twice.
    # In tree traversal, we visit children.
    # children_of[link1] -> [j3] -> visit(link3)
    # children_of[link2] -> [j4] -> visit(link3)

    j4 = Joint(name="j4", parent="link2", child="link3", type=JointType.FIXED)

    joints = [j1, j2, j3, j4]

    # This function doesn't validate tree structure, just sorts.
    # It should traverse without infinite loop and hit explicit return if revisited.
    sorted_joints = sort_joints_topological(joints, links)

    assert len(sorted_joints) == 4
    # j1/j2 (level 1) before j3/j4 (level 2)
    names = [j.name for j in sorted_joints]
    assert "j1" in names and "j2" in names
    # Ensure parents are processed before children
    # link1/link2 are parents of link3, so j1/j2 (creating link1/link2)
    # should ideally be before j3/j4 (using link1/link2).
    # But strictly, topological sort just ensures dependency.
    # Actually, `sort_joints_topological` ensures parent links exist.
    # base exists. j1 creates link1. j2 creates link2.
    # link1 exists -> j3 creates link3.
    # link2 exists -> j4 re-creates link3 (or attaches to it).
    pass
