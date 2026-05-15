"""Unit tests for Blender Scene analysis and utilities."""

from __future__ import annotations

from linkforge.blender.utils.scene_utils import (
    build_tree_from_stats,
    get_robot_statistics,
)

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
)

# Robot Statistics Analysis


class TestSceneAnalysis:
    def test_get_robot_statistics_basic(self, scene, blender_context) -> None:
        """Test gathering basic robot statistics from the scene."""
        # Create a link
        link_obj = create_test_object("link1", None, scene)
        safe_get_linkforge(link_obj).is_robot_link = True
        safe_get_linkforge(link_obj).link_name = "link1"
        safe_get_linkforge(link_obj).mass = 1.5

        stats = get_robot_statistics(scene)
        assert stats.num_links == 1
        assert stats.total_mass == 1.5
        assert "link1" in stats.link_objects

    def test_get_robot_statistics_excludes_invalid_mass(self, scene, blender_context) -> None:
        """Test that links with negative mass do not add up to total_mass."""
        link1 = create_test_object("valid_link", None, scene)
        safe_get_linkforge(link1).is_robot_link = True
        safe_get_linkforge(link1).mass = 10.0

        link2 = create_test_object("negative_link", None, scene)
        safe_get_linkforge(link2).is_robot_link = True
        safe_get_linkforge(link2).mass = -5.0

        stats = get_robot_statistics(scene)
        assert stats.num_links == 2
        assert stats.total_mass == 10.0


# Kinematic Tree Building


class TestTreeBuilding:
    def test_build_tree_from_stats_basic(self, scene, blender_context) -> None:
        """Test building a kinematic tree from robot statistics."""
        parent = create_test_object("parent", None, scene)
        safe_get_linkforge(parent).is_robot_link = True
        safe_get_linkforge(parent).link_name = "parent"

        child = create_test_object("child", None, scene)
        safe_get_linkforge(child).is_robot_link = True
        safe_get_linkforge(child).link_name = "child"

        joint = create_test_object("j1", None, scene)
        safe_get_joint(joint).is_robot_joint = True
        safe_get_joint(joint).parent_link = parent
        safe_get_joint(joint).child_link = child

        stats = get_robot_statistics(scene)
        tree, root_link, joints_dict, links_dict = build_tree_from_stats(stats)

        assert root_link == "parent"
        assert any(c[0] == "child" for c in tree["parent"])
        assert ("parent", "child") in joints_dict
