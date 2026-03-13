"""Unit tests for the KinematicGraph core model.

Verifies graph theory logic for robot structure validation and traversal.
"""

import pytest
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models import Joint, JointType, Link
from linkforge_core.models.graph import KinematicGraph


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
    assert graph.get_topological_order() == ["A", "B", "C"]
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
    with pytest.raises(RobotModelError, match="cycle"):
        graph.get_topological_order()


def test_graph_invalid_joint_links() -> None:
    """Verify validation of joint links during initialization."""
    links = [Link(name="A"), Link(name="B")]

    # Joint referencing unknown parent
    with pytest.raises(RobotModelError, match="references unknown parent"):
        KinematicGraph(links, [Joint(name="j1", parent="X", child="A", type=JointType.FIXED)])

    # Joint referencing unknown child
    with pytest.raises(RobotModelError, match="references unknown child"):
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
    order = graph.get_topological_order()
    assert order[0] == "A"
    assert set(order[1:]) == {"B", "C"}


def test_graph_empty_input() -> None:
    """Verify behavior with zero links or joints."""
    graph = KinematicGraph([], [])
    assert not graph.has_cycle()
    assert graph.get_root_links() == []
    assert graph.get_topological_order() == []
    assert graph.find_islands() == []


def test_graph_diamond_dag_coverage() -> None:
    """Verify diamond structure: A -> B, A -> C, B -> D, C -> D (no cycles)."""
    links = [Link(name="A"), Link(name="B"), Link(name="C"), Link(name="D")]
    joints = [
        Joint(name="j1", parent="A", child="B", type=JointType.FIXED),
        Joint(name="j2", parent="A", child="C", type=JointType.FIXED),
        Joint(name="j3", parent="B", child="D", type=JointType.FIXED),
        Joint(name="j4", parent="C", child="D", type=JointType.FIXED),
    ]
    graph = KinematicGraph(links, joints)

    # This hits the 'do nothing' branch in has_cycle when child is visited but not in rec_stack
    assert not graph.has_cycle()
    assert graph.get_root_links() == ["A"]

    # This hits the in_degree[child] != 0 branch in get_topological_order
    order = graph.get_topological_order()
    assert order == ["A", "B", "C", "D"]
