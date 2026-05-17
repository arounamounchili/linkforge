"""Unit tests for Blender Scene analysis and utilities."""

from __future__ import annotations

import os
from unittest.mock import patch

import bpy
from linkforge.blender.constants import (
    SUFFIX_COLLISION,
    TAG_COLLISION_GEOM,
    TAG_SOURCE_GEOM,
)
from linkforge.blender.utils.scene_utils import (
    build_tree_from_stats,
    clear_stats_cache,
    get_robot_statistics,
    is_robot_joint,
    is_robot_link,
    is_robot_sensor,
    is_robot_transmission,
    move_to_collection,
    sync_object_collections,
)
from linkforge.core.constants import GEOM_BOX, GEOM_MESH, GEOM_SPHERE

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_sensor,
    safe_get_transmission,
)


class TestSceneHelperChecks:
    def test_is_robot_link(self, scene, blender_context) -> None:
        """Verify is_robot_link check detects links and handles edge cases."""
        assert not is_robot_link(None)

        obj = create_test_object("not_a_link", None, scene)
        assert not is_robot_link(obj)

        safe_get_linkforge(obj).is_robot_link = True
        assert is_robot_link(obj)

    def test_is_robot_joint(self, scene, blender_context) -> None:
        """Verify is_robot_joint check detects joints only on empty objects and handles edge cases."""
        assert not is_robot_joint(None)

        obj = create_test_object("joint_mesh", None, scene)
        # By default create_test_object makes an EMPTY or MESH depending on arguments.
        # Ensure it is a MESH type.
        obj.type = "MESH"
        safe_get_joint(obj).is_robot_joint = True
        assert not is_robot_joint(obj)

        obj.type = "EMPTY"
        assert is_robot_joint(obj)

    def test_is_robot_sensor(self, scene, blender_context) -> None:
        """Verify is_robot_sensor check detects sensors only on empty objects."""
        assert not is_robot_sensor(None)

        obj = create_test_object("sensor_mesh", None, scene)
        obj.type = "MESH"
        safe_get_sensor(obj).is_robot_sensor = True
        assert not is_robot_sensor(obj)

        obj.type = "EMPTY"
        assert is_robot_sensor(obj)

    def test_is_robot_transmission(self, scene, blender_context) -> None:
        """Verify is_robot_transmission check detects transmissions only on empty objects."""
        assert not is_robot_transmission(None)

        obj = create_test_object("trans_mesh", None, scene)
        obj.type = "MESH"
        safe_get_transmission(obj).is_robot_transmission = True
        assert not is_robot_transmission(obj)

        obj.type = "EMPTY"
        assert is_robot_transmission(obj)


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

    def test_get_robot_statistics_caching(self, scene, blender_context) -> None:
        """Test statistics caching, force refresh, and disable cache env option."""

        class NoClearDict(dict):
            def clear(self) -> None:
                pass

        with patch.dict(os.environ, {"LINKFORGE_DISABLE_CACHE": "0"}):
            no_clear_cache = NoClearDict()
            with patch("linkforge.blender.utils.scene_utils._stats_cache", no_clear_cache):
                link_obj = create_test_object("link_c", None, scene)
                safe_get_linkforge(link_obj).is_robot_link = True
                safe_get_linkforge(link_obj).mass = 2.0

                # Initial call
                stats1 = get_robot_statistics(scene)
                assert stats1.num_links == 1
                assert stats1.total_mass == 2.0

                # Modify property of existing object but expect cache hit (since we ignore clear())
                safe_get_linkforge(link_obj).mass = 5.0
                stats2 = get_robot_statistics(scene)
                # Should return cached result (still mass 2.0, not 5.0)
                assert stats2.total_mass == 2.0

                # Force refresh
                stats3 = get_robot_statistics(scene, force_refresh=True)
                assert stats3.total_mass == 5.0

                # Disable cache via environment variable
                with patch.dict(os.environ, {"LINKFORGE_DISABLE_CACHE": "1"}):
                    safe_get_linkforge(link_obj).mass = 10.0
                    stats4 = get_robot_statistics(scene)
                    assert stats4.total_mass == 10.0

    def test_get_robot_statistics_cache_invalidation_on_reference_error(
        self, scene, blender_context
    ) -> None:
        """Test cache invalidation when a ReferenceError is raised (e.g. object deleted)."""
        with patch.dict(os.environ, {"LINKFORGE_DISABLE_CACHE": "0"}):
            clear_stats_cache()
            link_obj = create_test_object("link_del", None, scene)
            safe_get_linkforge(link_obj).is_robot_link = True

            stats1 = get_robot_statistics(scene)
            assert stats1.num_links == 1

            # Simulate object deletion by removing it from the scene (but keep count same to test cache lookup)
            scene.objects.remove(link_obj)
            create_test_object("dummy", None, scene)

            # Simulate object deletion by patching name to raise ReferenceError
            with patch.object(link_obj, "name", side_effect=ReferenceError("Object deleted")):
                # Should invalidate cache and re-scan
                stats2 = get_robot_statistics(scene)
                assert stats2.num_links == 0

    def test_get_robot_statistics_geometry_detection_explicit_tag(
        self, scene, blender_context
    ) -> None:
        """Test explicit geometry detection tag parsing."""
        link_obj = create_test_object("link_geom", None, scene)
        safe_get_linkforge(link_obj).is_robot_link = True

        collision_child = create_test_object(f"link_geom{SUFFIX_COLLISION}", None, scene)
        collision_child.parent = link_obj
        collision_child[TAG_SOURCE_GEOM] = GEOM_BOX

        stats = get_robot_statistics(scene, force_refresh=True)
        geo_info = stats.geometry_stats.get("link_geom")
        assert geo_info is not None
        assert geo_info[1] == GEOM_BOX
        assert geo_info[2] is True

    def test_get_robot_statistics_geometry_detection_generator_tag(
        self, scene, blender_context
    ) -> None:
        """Test generator tag parsing for geometry detection."""
        link_obj = create_test_object("link_gen", None, scene)
        safe_get_linkforge(link_obj).is_robot_link = True

        collision_child = create_test_object(f"link_gen{SUFFIX_COLLISION}", None, scene)
        collision_child.parent = link_obj
        collision_child[TAG_COLLISION_GEOM] = GEOM_SPHERE

        stats = get_robot_statistics(scene, force_refresh=True)
        geo_info = stats.geometry_stats.get("link_gen")
        assert geo_info is not None
        assert geo_info[1] == GEOM_SPHERE
        assert geo_info[2] is True

    def test_get_robot_statistics_geometry_heuristic_fallback_error(
        self, scene, blender_context
    ) -> None:
        """Test heuristic geometry detection resilient fallback when detecting raises an exception."""
        link_obj = create_test_object("link_heur", None, scene)
        safe_get_linkforge(link_obj).is_robot_link = True

        collision_child = create_test_object(f"link_heur{SUFFIX_COLLISION}", None, scene)
        collision_child.parent = link_obj
        collision_child[TAG_COLLISION_GEOM] = "INVALID"

        # Mock detect_primitive_type to raise an exception
        with patch(
            "linkforge.blender.utils.scene_utils.detect_primitive_type",
            side_effect=ValueError("Failed"),
        ):
            stats = get_robot_statistics(scene, force_refresh=True)
            geo_info = stats.geometry_stats.get("link_heur")
            assert geo_info is not None
            assert geo_info[1] == GEOM_MESH  # Fallback to GEOM_MESH
            assert geo_info[2] is False


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


# Collection Management


class TestCollectionManagement:
    def test_move_to_collection(self, scene) -> None:
        """Verify move_to_collection successfully moves objects between collections."""
        obj = create_test_object("test_move_obj", None, scene)

        col1 = bpy.data.collections.new("col1")
        col2 = bpy.data.collections.new("col2")

        # Initial link to col1
        col1.objects.link(obj)
        assert col1 in obj.users_collection

        # Move to col2
        move_to_collection(obj, col2)
        assert col2 in obj.users_collection
        assert col1 not in obj.users_collection

        # Null check safety
        move_to_collection(None, col2)  # Should not raise exception
        move_to_collection(obj, None)  # Should not raise exception

    def test_sync_object_collections(self, scene) -> None:
        """Verify sync_object_collections synchronizes target collections with source."""
        source = create_test_object("source_obj", None, scene)
        target = create_test_object("target_obj", None, scene)

        col1 = bpy.data.collections.new("col_sync_1")
        col2 = bpy.data.collections.new("col_sync_2")

        col1.objects.link(source)
        col2.objects.link(source)

        # Synchronize target to match source collections
        sync_object_collections(target, source)
        assert col1 in target.users_collection
        assert col2 in target.users_collection

        # Null check safety
        sync_object_collections(None, source)  # Should not raise exception
        sync_object_collections(target, None)  # Should not raise exception
