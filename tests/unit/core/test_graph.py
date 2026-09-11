"""Unit tests for the KinematicGraph core model.

Verifies graph theory logic for robot structure validation and traversal.
"""

import pytest
from linkforge.core import (
    Joint,
    JointType,
    KinematicGraph,
    Link,
    RobotModelError,
    RobotValidationError,
)


def test_graph_simple_chain() -> None:
    """Verify linear chain: A -> B -> C."""
    links = [Link(name="A"), Link(name="B"), Link(name="C")]
    joints = [
        Joint(name="j1", parent="A", child="B", type=JointType.FIXED),
        Joint(name="j2", parent="B", child="C", type=JointType.FIXED),
    ]
    graph = KinematicGraph(links, joints)

    assert not graph.has_cycle()
    assert graph.get_root_links() == ["A"]
    assert graph.get_topological_link_names() == ["A", "B", "C"]
    assert len(graph.find_islands()) == 1


def test_graph_cycle_detection() -> None:
    """Verify detection of cyclic dependencies like A -> B -> C -> A."""
    links = [Link(name="A"), Link(name="B"), Link(name="C")]
    joints = [
        Joint(name="j1", parent="A", child="B", type=JointType.FIXED),
        Joint(name="j2", parent="B", child="C", type=JointType.FIXED),
        Joint(name="j3", parent="C", child="A", type=JointType.FIXED),
    ]
    graph = KinematicGraph(links, joints)

    assert graph.has_cycle()
    with pytest.raises(RobotModelError, match="cycles"):
        graph.get_topological_link_names()


def test_graph_invalid_joint_links() -> None:
    """Verify validation of joint links during initialization."""
    links = [Link(name="A"), Link(name="B")]

    # Joint referencing unknown parent
    with pytest.raises(RobotModelError, match="unknown"):
        KinematicGraph(links, [Joint(name="j1", parent="X", child="A", type=JointType.FIXED)])

    # Joint referencing unknown child
    with pytest.raises(RobotModelError, match="unknown"):
        KinematicGraph(links, [Joint(name="j1", parent="A", child="X", type=JointType.FIXED)])


def test_graph_isolated_link_root() -> None:
    """Verify that isolated links are correctly handled as roots."""
    links = [Link(name="A"), Link(name="B")]
    joints = [Joint(name="j1", parent="A", child="B", type=JointType.FIXED)]
    graph = KinematicGraph(links + [Link(name="C")], joints)

    # C is a root because it has no incoming edges (it has no edges at all)
    assert sorted(graph.get_root_links()) == ["A", "C"]
    assert len(graph.find_islands()) == 2


def test_graph_islands() -> None:
    """Verify discovery of disconnected robot components."""
    links = [Link(name="A"), Link(name="B"), Link(name="C"), Link(name="D")]
    joints = [
        Joint(name="j1", parent="A", child="B", type=JointType.FIXED),
        Joint(name="j2", parent="C", child="D", type=JointType.FIXED),
    ]
    graph = KinematicGraph(links, joints)

    islands = graph.find_islands()
    assert len(islands) == 2
    assert {"A", "B"} in islands
    assert {"C", "D"} in islands
    assert sorted(graph.get_root_links()) == ["A", "C"]


def test_graph_branching() -> None:
    """Verify branching structures: A -> B, A -> C."""
    links = [Link(name="A"), Link(name="B"), Link(name="C")]
    joints = [
        Joint(name="j1", parent="A", child="B", type=JointType.FIXED),
        Joint(name="j2", parent="A", child="C", type=JointType.FIXED),
    ]
    graph = KinematicGraph(links, joints)

    assert not graph.has_cycle()
    assert graph.get_root_links() == ["A"]
    order = graph.get_topological_link_names()
    assert order[0] == "A"
    assert set(order[1:]) == {"B", "C"}


def test_graph_empty_input() -> None:
    """Verify behavior with zero links or joints."""
    graph = KinematicGraph([], [])
    assert not graph.has_cycle()
    assert graph.get_root_links() == []
    assert graph.get_topological_link_names() == []
    assert graph.find_islands() == []


def test_graph_diamond_dag() -> None:
    """Verify diamond structure: A -> B, A -> C, B -> D, C -> D (no cycles)."""
    links = [Link(name="A"), Link(name="B"), Link(name="C"), Link(name="D")]
    joints = [
        Joint(name="j1", parent="A", child="B", type=JointType.FIXED),
        Joint(name="j2", parent="A", child="C", type=JointType.FIXED),
        Joint(name="j3", parent="B", child="D", type=JointType.FIXED),
        Joint(name="j4", parent="C", child="D", type=JointType.FIXED),
    ]
    graph = KinematicGraph(links, joints)

    # Child visited from multiple paths in a DAG without cycle
    assert not graph.has_cycle()
    assert graph.get_root_links() == ["A"]

    # Topological link ordering for diamond graph
    order = graph.get_topological_link_names()
    assert order == ["A", "B", "C", "D"]


def test_graph_topological_joints_dag() -> None:
    """Verify topological joints sorting with a diamond DAG, cyclic exceptions, and ghost joints."""
    links = [Link(name="A"), Link(name="B"), Link(name="C"), Link(name="D")]
    joints = [
        Joint(name="j1", parent="A", child="B", type=JointType.FIXED),
        Joint(name="j2", parent="A", child="C", type=JointType.FIXED),
        Joint(name="j3", parent="B", child="D", type=JointType.FIXED),
        Joint(name="j4", parent="C", child="D", type=JointType.FIXED),
    ]
    graph = KinematicGraph(links, joints)
    top_joints = graph.get_topological_joints()
    assert len(top_joints) == 4

    cyclic_links = [Link(name="A"), Link(name="B")]
    cyclic_joints = [
        Joint(name="j1", parent="A", child="B", type=JointType.FIXED),
        Joint(name="j2", parent="B", child="A", type=JointType.FIXED),
    ]
    cyclic_graph = KinematicGraph(cyclic_links, cyclic_joints)
    with pytest.raises(RobotModelError, match="cycles"):
        cyclic_graph.get_topological_joints()

    graph.adj["A"].append(("B", "ghost_joint"))
    top_joints_ghost = graph.get_topological_joints()
    # The length of returned joints is still 4 because ghost_joint is filtered out by `if joint:`
    assert len(top_joints_ghost) == 4


def test_get_topological_joints() -> None:
    """Test that joints are sorted correctly (parents before children)."""
    base = Link(name="base")
    link1 = Link(name="link1")
    link2 = Link(name="link2")
    link3 = Link(name="link3")
    links = [base, link1, link2, link3]

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

    indices = {j.name: i for i, j in enumerate(sorted_joints)}
    assert indices["j1"] < indices["j2"]  # l0->l1 before l1->l2
    assert indices["j3"] < indices["j4"]  # l0->l3 before l3->l4


def test_get_topological_joints_with_islands() -> None:
    """Test topological sort with disconnected robots (islands)."""
    # First disconnected chain: r1_base -> r1_link
    # Second disconnected chain: r2_base -> r2_link
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
