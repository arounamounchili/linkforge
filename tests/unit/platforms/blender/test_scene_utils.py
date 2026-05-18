"""Unit tests for Blender Scene analysis and utilities."""

from __future__ import annotations

import os
from unittest.mock import MagicMock, PropertyMock, patch

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

    def test_get_robot_statistics_cache_validation_errors(self, scene, blender_context) -> None:
        """Test cache validation for all object types (joints, sensors, etc.)."""
        from linkforge.blender.utils.scene_utils import RobotSceneStatistics, _stats_cache

        # Populate the cache manually with dummy stats
        cache_key = (id(scene), getattr(scene, "frame_current", 0), len(scene.objects))

        with patch.dict(os.environ, {"LINKFORGE_DISABLE_CACHE": "0"}):
            # 1. Test ReferenceError on joint_obj
            bad_joint = MagicMock()
            type(bad_joint).name = PropertyMock(side_effect=ReferenceError("deleted"))
            stats = RobotSceneStatistics(
                num_links=0,
                total_mass=0.0,
                total_dof=0,
                link_objects={},
                joint_objects=[bad_joint],
                sensor_objects=[],
                transmission_objects=[],
                root_link=None,
            )
            _stats_cache[cache_key] = stats
            # This lookup should detect ReferenceError, invalidate cache, and re-scan the scene
            get_robot_statistics(scene)
            assert cache_key not in _stats_cache or _stats_cache[cache_key] != stats

            # 2. Test ReferenceError on sensor_obj
            bad_sensor = MagicMock()
            type(bad_sensor).name = PropertyMock(side_effect=ReferenceError("deleted"))
            stats = RobotSceneStatistics(
                num_links=0,
                total_mass=0.0,
                total_dof=0,
                link_objects={},
                joint_objects=[],
                sensor_objects=[bad_sensor],
                transmission_objects=[],
                root_link=None,
            )
            _stats_cache[cache_key] = stats
            get_robot_statistics(scene)
            assert cache_key not in _stats_cache or _stats_cache[cache_key] != stats

            # 3. Test ReferenceError on transmission_obj
            bad_trans = MagicMock()
            type(bad_trans).name = PropertyMock(side_effect=ReferenceError("deleted"))
            stats = RobotSceneStatistics(
                num_links=0,
                total_mass=0.0,
                total_dof=0,
                link_objects={},
                joint_objects=[],
                sensor_objects=[],
                transmission_objects=[bad_trans],
                root_link=None,
            )
            _stats_cache[cache_key] = stats
            get_robot_statistics(scene)
            assert cache_key not in _stats_cache or _stats_cache[cache_key] != stats

            # 4. Test ReferenceError on geometry_stats
            bad_geo = MagicMock()
            type(bad_geo).name = PropertyMock(side_effect=ReferenceError("deleted"))
            stats = RobotSceneStatistics(
                num_links=0,
                total_mass=0.0,
                total_dof=0,
                link_objects={},
                joint_objects=[],
                sensor_objects=[],
                transmission_objects=[],
                root_link=None,
                geometry_stats={"some_link": (bad_geo, "box", True)},
            )
            _stats_cache[cache_key] = stats
            get_robot_statistics(scene)
            assert cache_key not in _stats_cache or _stats_cache[cache_key] != stats

            # 5. Test ReferenceError on manual_inertia_objects
            bad_inertia = MagicMock()
            type(bad_inertia).name = PropertyMock(side_effect=ReferenceError("deleted"))
            stats = RobotSceneStatistics(
                num_links=0,
                total_mass=0.0,
                total_dof=0,
                link_objects={},
                joint_objects=[],
                sensor_objects=[],
                transmission_objects=[],
                root_link=None,
                manual_inertia_objects=[bad_inertia],
            )
            _stats_cache[cache_key] = stats
            get_robot_statistics(scene)
            assert cache_key not in _stats_cache or _stats_cache[cache_key] != stats

    def test_scene_utils_edge_cases(self, scene) -> None:
        """Cover rare branch conditions, Falsy bounds, collection sync and heuristic tags."""
        from linkforge.blender.utils.scene_utils import (
            build_tree_from_stats,
            get_robot_statistics,
            move_to_collection,
            sync_object_collections,
        )

        # Empty scene or missing hasattr(scene, 'objects')
        assert get_robot_statistics(None).num_links == 0
        assert get_robot_statistics(MagicMock(spec=[])).num_links == 0

        # BadObjects to trigger length check TypeError/AttributeError
        class BadObjects:
            def __len__(self) -> int:
                raise TypeError("Bad Length")

            def __iter__(self):
                return iter([])

        bad_scene = MagicMock()
        bad_scene.objects = BadObjects()
        get_robot_statistics(bad_scene, force_refresh=True)

        # Heuristic / tags checks
        link_obj = create_test_object("link_geom_mesh", None, scene)
        safe_get_linkforge(link_obj).is_robot_link = True
        safe_get_linkforge(link_obj).use_auto_inertia = False
        collision_child = create_test_object("link_geom_mesh_collision", None, scene)
        collision_child.parent = link_obj

        # 1. stored_type == GEOM_MESH
        collision_child[TAG_COLLISION_GEOM] = GEOM_MESH
        stats = get_robot_statistics(scene, force_refresh=True)
        assert stats.geometry_stats["link_geom_mesh"][1] == GEOM_MESH
        assert link_obj in stats.manual_inertia_objects

        # 2. non-string stored_type
        collision_child[TAG_COLLISION_GEOM] = 123
        get_robot_statistics(scene, force_refresh=True)

        # 3. Heuristic primitive detection returning a value
        collision_child[TAG_COLLISION_GEOM] = "AUTO"
        with patch(
            "linkforge.blender.utils.scene_utils.detect_primitive_type", return_value="box"
        ) as mock_det:
            stats = get_robot_statistics(scene, force_refresh=True)
            assert stats.geometry_stats["link_geom_mesh"][1] == "box"

        # Joint props Falsy branches in loop
        joint_obj = create_test_object("joint_empty", None, scene)
        joint_obj.type = "EMPTY"
        safe_get_joint(joint_obj).is_robot_joint = True
        # jp.child_link is None
        stats = get_robot_statistics(scene, force_refresh=True)
        assert stats.total_dof == 1  # Continuous has 1 DOF

        # jp.parent_link has no link props
        non_link_parent = create_test_object("non_link_parent", None, scene)
        non_link_parent.linkforge = None
        child_link = create_test_object("child_link_real", None, scene)
        safe_get_linkforge(child_link).is_robot_link = True
        safe_get_linkforge(child_link).link_name = "child_link_real"
        safe_get_joint(joint_obj).child_link = child_link
        safe_get_joint(joint_obj).parent_link = non_link_parent
        stats = get_robot_statistics(scene, force_refresh=True)
        assert "child_link_real" in stats.link_objects

        # Sensors and Transmissions detection
        sensor_obj = create_test_object("sensor_test", None, scene)
        sensor_obj.type = "EMPTY"
        safe_get_sensor(sensor_obj).is_robot_sensor = True
        trans_obj = create_test_object("trans_test", None, scene)
        trans_obj.type = "EMPTY"
        safe_get_transmission(trans_obj).is_robot_transmission = True
        stats = get_robot_statistics(scene, force_refresh=True)
        assert sensor_obj in stats.sensor_objects
        assert trans_obj in stats.transmission_objects

        # build_tree_from_stats Falsy jp or parent_name not in tree
        from linkforge.blender.utils.scene_utils import RobotSceneStatistics

        bad_joint_obj = MagicMock()
        type(bad_joint_obj).name = "bad_joint"
        # Raise ReferenceError in get_joint_props
        with patch(
            "linkforge.blender.utils.scene_utils.get_joint_props",
            side_effect=ReferenceError("deleted"),
        ):
            stats_mock = RobotSceneStatistics(
                num_links=2,
                total_mass=1.0,
                total_dof=1,
                link_objects={"link_a": None, "link_b": None},
                joint_objects=[],
                sensor_objects=[],
                transmission_objects=[],
                root_link=None,
                joints_map={"link_b": ("link_a", bad_joint_obj)},
            )
            tree, root, joints, links = build_tree_from_stats(stats_mock)
            assert tree["link_a"] == []

        # Helper to construct clean mock objects with all flags False by default
        def create_clean_mock(name: str):
            obj = MagicMock()
            obj.name = name
            obj.linkforge = MagicMock(is_robot_link=False, mass=0.0, use_auto_inertia=True)
            obj.linkforge_joint = MagicMock(is_robot_joint=False)
            obj.linkforge_sensor = MagicMock(is_robot_sensor=False)
            obj.linkforge_transmission = MagicMock(is_robot_transmission=False)
            return obj

        # Cover jp := get_joint_props(obj) evaluates to False
        no_jp_scene = MagicMock()
        no_jp_joint = create_clean_mock("no_jp_joint")
        no_jp_scene.objects = [no_jp_joint]
        with (
            patch("linkforge.blender.utils.scene_utils.is_robot_joint", return_value=True),
            patch("linkforge.blender.utils.scene_utils.get_joint_props", return_value=None),
        ):
            get_robot_statistics(no_jp_scene, force_refresh=True)

        # Cover root link finding loop when there is no root link (every link is in joints_map)
        no_root_scene = MagicMock()

        no_root_link1 = create_clean_mock("loop_link1")
        no_root_link1.linkforge.is_robot_link = True
        no_root_link1.linkforge.mass = 1.0
        no_root_link1.linkforge.link_name = "loop_link1"

        no_root_link2 = create_clean_mock("loop_link2")
        no_root_link2.linkforge.is_robot_link = True
        no_root_link2.linkforge.mass = 1.0
        no_root_link2.linkforge.link_name = "loop_link2"

        no_root_joint1 = create_clean_mock("loop_joint1")
        no_root_joint1.type = "EMPTY"
        no_root_joint1.linkforge_joint.is_robot_joint = True
        no_root_joint1.linkforge_joint.joint_type = "continuous"
        no_root_joint1.linkforge_joint.child_link = no_root_link1
        no_root_joint1.linkforge_joint.parent_link = no_root_link2

        no_root_joint2 = create_clean_mock("loop_joint2")
        no_root_joint2.type = "EMPTY"
        no_root_joint2.linkforge_joint.is_robot_joint = True
        no_root_joint2.linkforge_joint.joint_type = "continuous"
        no_root_joint2.linkforge_joint.child_link = no_root_link2
        no_root_joint2.linkforge_joint.parent_link = no_root_link1

        no_root_scene.objects = [no_root_link1, no_root_link2, no_root_joint1, no_root_joint2]
        stats_no_root = get_robot_statistics(no_root_scene, force_refresh=True)
        assert stats_no_root.root_link is None

        # move_to_collection target same as current
        col_same = bpy.data.collections.new("col_same")
        obj_same = create_test_object("obj_same", None, scene)
        col_same.objects.link(obj_same)
        move_to_collection(obj_same, col_same)

        # sync_object_collections early return when empty source_cols
        obj_source = create_test_object("obj_source", None, scene)
        obj_target = create_test_object("obj_target", None, scene)
        # Ensure obj_source has NO collections
        for col in list(obj_source.users_collection):
            col.objects.unlink(obj_source)
        sync_object_collections(obj_target, obj_source)

        # sync_object_collections unlink from collections source is not in
        col_ref = bpy.data.collections.new("col_ref")
        col_extra = bpy.data.collections.new("col_extra")
        col_ref.objects.link(obj_source)
        col_extra.objects.link(obj_target)

        with patch("tests.mock_bpy_env.MockCollection.unlink") as mock_unlink:
            sync_object_collections(obj_target, obj_source)
            mock_unlink.assert_any_call(obj_target)
            assert col_ref in obj_target.users_collection


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

    def test_build_tree_from_stats_edge_cases(self) -> None:
        """Test build_tree_from_stats with various uncommon branches (missing parents, missing properties)."""
        from linkforge.blender.utils.scene_utils import RobotSceneStatistics, build_tree_from_stats

        # 1. parent_name not in tree (parent_name not in links)
        joint_obj1 = MagicMock()
        stats = RobotSceneStatistics(
            num_links=1,
            total_mass=0.0,
            total_dof=0,
            link_objects={"child": MagicMock()},
            joint_objects=[joint_obj1],
            sensor_objects=[],
            transmission_objects=[],
            root_link=None,
            joints_map={"child": ("unknown_parent", joint_obj1)},
        )
        # Should not raise exception, but skips tree population since parent is not in tree
        tree, root_link, joints_dict, links_dict = build_tree_from_stats(stats)
        assert "unknown_parent" not in tree

        # 2. get_joint_props returns None
        joint_obj2 = MagicMock()
        with patch("linkforge.blender.utils.scene_utils.get_joint_props", return_value=None):
            stats = RobotSceneStatistics(
                num_links=2,
                total_mass=0.0,
                total_dof=0,
                link_objects={"parent": MagicMock(), "child": MagicMock()},
                joint_objects=[joint_obj2],
                sensor_objects=[],
                transmission_objects=[],
                root_link=None,
                joints_map={"child": ("parent", joint_obj2)},
            )
            tree, root_link, joints_dict, links_dict = build_tree_from_stats(stats)
            assert len(tree["parent"]) == 0

        # 3. ReferenceError raised when accessing joint properties
        joint_obj3 = MagicMock()
        with patch(
            "linkforge.blender.utils.scene_utils.get_joint_props",
            side_effect=ReferenceError("deleted"),
        ):
            stats = RobotSceneStatistics(
                num_links=2,
                total_mass=0.0,
                total_dof=0,
                link_objects={"parent": MagicMock(), "child": MagicMock()},
                joint_objects=[joint_obj3],
                sensor_objects=[],
                transmission_objects=[],
                root_link=None,
                joints_map={"child": ("parent", joint_obj3)},
            )
            tree, root_link, joints_dict, links_dict = build_tree_from_stats(stats)
            assert len(tree["parent"]) == 0


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

        # Call move_to_collection again when already in col2 to cover "already there" branch
        move_to_collection(obj, col2)

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
