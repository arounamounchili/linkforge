from unittest.mock import MagicMock

import bpy
from linkforge.blender import handlers as handlers_pkg
from linkforge.blender.handlers import name_sync_handler
from linkforge.blender.utils.property_helpers import safe_set_id_name

from tests.blender_test_utils import (
    cleanup_blender_scene,
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_sensor,
    safe_update,
)


def test_flush_deferred_renames_all_paths(scene):
    """Test flush_deferred_renames covering all branches (success, no name attribute, exception)."""
    cleanup_blender_scene(scene)

    obj_ok = MagicMock()
    obj_ok.name = "old"

    obj_no_name = object()  # plain object without "name"

    class BadObject:
        @property
        def name(self):
            return "bad"

        @name.setter
        def name(self, val):
            raise RuntimeError("Read-only!")

    class DeletedObject:
        @property
        def name(self):
            raise ReferenceError("StructRNA of type Object has been removed")

    obj_bad = BadObject()
    obj_deleted = DeletedObject()

    name_sync_handler.PENDING_RENAMES[:] = [
        (obj_ok, "new_ok"),
        (obj_no_name, "ignored"),
        (obj_bad, "new_bad"),
        (obj_deleted, "dropped"),
    ]

    # Flush
    name_sync_handler.flush_deferred_renames()

    assert obj_ok.name == "new_ok"

    # Only obj_bad should remain in queue; obj_deleted must be dropped
    assert len(name_sync_handler.PENDING_RENAMES) == 1
    assert name_sync_handler.PENDING_RENAMES[0] == (obj_bad, "new_bad")

    # Clear queue
    name_sync_handler.PENDING_RENAMES.clear()


def test_on_depsgraph_update_post_all_branches(scene):
    """Verify both True and False branches for all component synchronizations in the depsgraph handler."""
    cleanup_blender_scene(scene)

    # Test cases:
    # - Updates that require sanitization (sanitized != current)
    # - Updates already sanitized (sanitized == current)

    link_true = create_test_object("link_true", None, scene=scene)
    lp_true = safe_get_linkforge(link_true, scene)
    lp_true.is_robot_link = True
    link_true.name = "new_link_true"
    lp_true._values["source_name_stored"] = "different_name"  # Bypass callback and trigger sync

    link_false = create_test_object("link_false", None, scene=scene)
    lp_false = safe_get_linkforge(link_false, scene)
    lp_false.is_robot_link = True
    link_false.name = "new_link_false"

    joint_true = create_test_object("joint_true", None, scene=scene)
    jp_true = safe_get_joint(joint_true, scene)
    jp_true.is_robot_joint = True
    joint_true.name = "new_joint_true"
    jp_true._values["source_name_stored"] = "different_name"

    joint_false = create_test_object("joint_false", None, scene=scene)
    jp_false = safe_get_joint(joint_false, scene)
    jp_false.is_robot_joint = True
    joint_false.name = "new_joint_false"

    sensor_true = create_test_object("sensor_true", None, scene=scene)
    sp_true = safe_get_sensor(sensor_true, scene)
    sp_true.is_robot_sensor = True
    sensor_true.name = "new_sensor_true"
    sp_true._values["source_name_stored"] = "different_name"

    sensor_false = create_test_object("sensor_false", None, scene=scene)
    sp_false = safe_get_sensor(sensor_false, scene)
    sp_false.is_robot_sensor = True
    sensor_false.name = "new_sensor_false"
    sp_false.sensor_name = "new_sensor_false"

    # Define mock update objects
    class MockUpdate:
        def __init__(self, id_obj):
            self.id = id_obj

    class MockDepsGraph:
        def __init__(self, updates):
            self.updates = updates

    non_object_datablock = object()  # Simulates Material, NodeTree, Mesh ID without .type

    updates = [
        MockUpdate(non_object_datablock),
        MockUpdate(link_true),
        MockUpdate(link_false),
        MockUpdate(joint_true),
        MockUpdate(joint_false),
        MockUpdate(sensor_true),
        MockUpdate(sensor_false),
    ]
    depsgraph = MockDepsGraph(updates)

    # Call depsgraph handler
    name_sync_handler.on_depsgraph_update_post(scene, depsgraph)

    assert lp_true.link_name == "new_link_true"
    assert jp_true.joint_name == "new_joint_true"
    assert sp_true.sensor_name == "new_sensor_true"


def test_register_unregister():
    """Test register and unregister functions of name_sync_handler and package-level handlers."""
    # Ensure it's not present initially
    if name_sync_handler.on_depsgraph_update_post in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(name_sync_handler.on_depsgraph_update_post)

    handlers_pkg.register()
    assert name_sync_handler.on_depsgraph_update_post in bpy.app.handlers.depsgraph_update_post

    initial_len = len(bpy.app.handlers.depsgraph_update_post)
    handlers_pkg.register()
    assert len(bpy.app.handlers.depsgraph_update_post) == initial_len

    handlers_pkg.unregister()
    assert name_sync_handler.on_depsgraph_update_post not in bpy.app.handlers.depsgraph_update_post

    handlers_pkg.unregister()
    assert name_sync_handler.on_depsgraph_update_post not in bpy.app.handlers.depsgraph_update_post


def test_safe_set_id_name_reference_error():
    """Verify safe_set_id_name and deferred_rename do not crash when StructRNA is removed."""

    class MockDeleted:
        @property
        def name(self):
            raise ReferenceError("StructRNA of type Object has been removed")

    # Should not raise
    safe_set_id_name(MockDeleted(), "test_name")

    # Test deferred timer handling of deleted object
    class MockDeferredDeleted:
        def __init__(self):
            self._removed = False

        @property
        def name(self):
            if self._removed:
                raise ReferenceError("StructRNA of type Object has been removed")
            return "old_name"

        @name.setter
        def name(self, val):
            if self._removed:
                raise ReferenceError("StructRNA of type Object has been removed")
            raise RuntimeError("RNA locked")

    deferred_obj = MockDeferredDeleted()
    # Trigger deferred timer
    safe_set_id_name(deferred_obj, "new_name")

    # Now simulate the object being removed before the timer fires
    deferred_obj._removed = True

    # Execute all scheduled timers in mock
    if hasattr(bpy.app.timers, "run_all"):
        bpy.app.timers.run_all()


class TestNameSynchronization:
    def test_link_name_tracks_object_rename(self, scene, blender_context) -> None:
        """Test that link_name auto-syncs when Blender renames the object.

        The name_sync_handler deliberately propagates obj.name → link_name
        so that robot identities stay consistent after Outliner renames.
        """
        obj = create_test_object("sync_link", None, scene)
        lf = safe_get_linkforge(obj)
        lf.is_robot_link = True
        lf.link_name = "sync_link"

        # Initial state: names should match
        assert lf.link_name == "sync_link"

        # Simulate Blender renaming: the handler should propagate the new name
        obj.name = "sync_link_renamed"
        safe_update(scene)

        # The handler should have updated link_name to match the new obj.name
        assert safe_get_linkforge(obj).link_name == "sync_link_renamed"

    def test_joint_name_tracks_object_rename(self, scene, blender_context) -> None:
        """Test that joint_name auto-syncs when Blender renames the object.

        The name_sync_handler deliberately propagates obj.name → joint_name
        so that joint identities stay consistent after Outliner renames.
        """
        obj = create_test_object("sync_joint", None, scene)
        jf = safe_get_joint(obj)
        jf.is_robot_joint = True
        jf.joint_name = "sync_joint"

        # Initial state: names should match
        assert jf.joint_name == "sync_joint"

        # Simulate Blender renaming: the handler should propagate the new name
        obj.name = "sync_joint_renamed"
        safe_update(scene)

        # The handler should have updated joint_name to match the new obj.name
        assert safe_get_joint(obj).joint_name == "sync_joint_renamed"

    def test_sync_scene_identities(self, scene) -> None:
        """Test batch synchronization of scene identities across links, joints, and sensors."""
        cleanup_blender_scene(scene)

        # Create link, joint, sensor with desynced source names
        link = create_test_object("arm_link", None, scene)
        lp = safe_get_linkforge(link, scene)
        lp.is_robot_link = True
        lp._values["source_name_stored"] = "old_arm_link"

        joint = create_test_object("arm_joint", None, scene)
        jp = safe_get_joint(joint, scene)
        jp.is_robot_joint = True
        jp._values["source_name_stored"] = "old_arm_joint"

        sensor = create_test_object("camera_sensor", None, scene)
        sp = safe_get_sensor(sensor, scene)
        sp.is_robot_sensor = True
        sp._values["source_name_stored"] = "old_camera_sensor"

        # Trigger batch synchronization
        name_sync_handler.sync_scene_identities(scene)

        assert lp.link_name == "arm_link"
        assert jp.joint_name == "arm_joint"
        assert sp.sensor_name == "camera_sensor"

    def test_sync_object_identity_unwraps_evaluated_original(self, scene) -> None:
        """Verify sync_object_identity targets the original object when passed an evaluated proxy."""
        real_obj = create_test_object("manipulator_forearm", None, scene)
        lp = safe_get_linkforge(real_obj, scene)
        lp.is_robot_link = True
        lp._values["source_name_stored"] = "arm_link"

        class MockEvaluatedProxy:
            def __init__(self, orig):
                self.original = orig
                self.type = orig.type
                self.name = orig.name

        proxy = MockEvaluatedProxy(real_obj)
        name_sync_handler.sync_object_identity(proxy)

        assert lp.link_name == "manipulator_forearm"

    def test_sync_scene_identities_edge_cases(self) -> None:
        """Verify sync_scene_identities handles None and scenes without objects attribute gracefully."""
        # Should not raise any errors
        name_sync_handler.sync_scene_identities(None)
        name_sync_handler.sync_scene_identities(object())

    def test_sync_object_identity_non_object_datablocks(self) -> None:
        """Verify sync_object_identity safely ignores datablocks without a type attribute."""
        # E.g. Mesh, Material, NodeTree or arbitrary python objects
        name_sync_handler.sync_object_identity(None)
        name_sync_handler.sync_object_identity(object())

    def test_sync_object_identity_reference_error(self) -> None:
        """Verify sync_object_identity gracefully handles deleted datablocks raising ReferenceError."""

        class MockDeletedObject:
            @property
            def original(self):
                raise ReferenceError("StructRNA of type Object has been removed")

        # Should not raise
        name_sync_handler.sync_object_identity(MockDeletedObject())
