"""Unit tests for the KinematicGraph core model.

Verifies graph theory logic for robot structure validation and traversal.
"""

import pytest
from linkforge_core.models import Joint, JointType, Link
from linkforge_core.models.graph import KinematicGraph


def test_graph_simple_chain():
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


def test_graph_cycle_detection():
    """Verify detection of cyclic dependencies like A -> B -> C -> A."""
    links = [Link(name="A"), Link(name="B"), Link(name="C")]
    joints = [
        Joint(name="j1", parent="A", child="B", type=JointType.FIXED),
        Joint(name="j2", parent="B", child="C", type=JointType.FIXED),
        Joint(name="j3", parent="C", child="A", type=JointType.FIXED),
    ]
    graph = KinematicGraph(links, joints)

    assert graph.has_cycle()
    with pytest.raises(ValueError, match="cycle"):
        graph.get_topological_order()


def test_graph_islands():
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


def test_graph_branching():
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


def test_graph_empty_input():
    """Verify behavior with zero links or joints."""
    graph = KinematicGraph([], [])
    assert not graph.has_cycle()
    assert graph.get_root_links() == []
    assert graph.get_topological_order() == []
    assert graph.find_islands() == []
