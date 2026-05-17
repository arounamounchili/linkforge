from unittest.mock import MagicMock

import bpy
from linkforge.blender.handlers import name_sync_handler

from tests.blender_test_utils import (
    cleanup_blender_scene,
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_sensor,
    safe_get_transmission,
)


def test_flush_deferred_renames_all_paths(scene):
    """Test flush_deferred_renames covering all branches (success, no name attribute, exception)."""
    cleanup_blender_scene(scene)

    # 1. Success path
    obj_ok = MagicMock()
    obj_ok.name = "old"

    # 2. No name attribute path
    obj_no_name = object()  # plain object without "name"

    # 3. Exception path
    class BadObject:
        @property
        def name(self):
            return "bad"

        @name.setter
        def name(self, val):
            raise RuntimeError("Read-only!")

    obj_bad = BadObject()

    name_sync_handler.PENDING_RENAMES = [
        (obj_ok, "new_ok"),
        (obj_no_name, "ignored"),
        (obj_bad, "new_bad"),
    ]

    # Flush
    name_sync_handler.flush_deferred_renames()

    # Verify obj_ok was renamed
    assert obj_ok.name == "new_ok"

    # Verify only obj_bad was kept in PENDING_RENAMES (because it failed)
    assert len(name_sync_handler.PENDING_RENAMES) == 1
    assert name_sync_handler.PENDING_RENAMES[0] == (obj_bad, "new_bad")

    # Clear queue
    name_sync_handler.PENDING_RENAMES.clear()


def test_on_depsgraph_update_post_all_branches(scene):
    """Verify both True and False branches for all component synchronizations in the depsgraph handler."""
    cleanup_blender_scene(scene)

    # We need:
    # - Updates that trigger TRUE branch (sanitized != current)
    # - Updates that trigger FALSE branch (sanitized == current)

    # 1. Link objects
    link_true = create_test_object("link_true", None, scene=scene)
    lp_true = safe_get_linkforge(link_true, scene)
    lp_true.is_robot_link = True
    link_true.name = "new_link_true"
    lp_true._values["source_name_stored"] = "different_name"  # Bypass callback and trigger sync

    link_false = create_test_object("link_false", None, scene=scene)
    lp_false = safe_get_linkforge(link_false, scene)
    lp_false.is_robot_link = True
    link_false.name = "new_link_false"

    # 2. Joint objects
    joint_true = create_test_object("joint_true", None, scene=scene)
    jp_true = safe_get_joint(joint_true, scene)
    jp_true.is_robot_joint = True
    joint_true.name = "new_joint_true"
    jp_true._values["source_name_stored"] = "different_name"

    joint_false = create_test_object("joint_false", None, scene=scene)
    jp_false = safe_get_joint(joint_false, scene)
    jp_false.is_robot_joint = True
    joint_false.name = "new_joint_false"

    # 3. Sensor objects
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

    # 4. Transmission objects
    trans_true = create_test_object("trans_true", None, scene=scene)
    tp_true = safe_get_transmission(trans_true, scene)
    tp_true.is_robot_transmission = True
    trans_true.name = "new_trans_true"
    tp_true._values["source_name_stored"] = "different_name"

    trans_false = create_test_object("trans_false", None, scene=scene)
    tp_false = safe_get_transmission(trans_false, scene)
    tp_false.is_robot_transmission = True
    trans_false.name = "new_trans_false"
    tp_false.transmission_name = "new_trans_false"

    # Define mock update objects
    class MockUpdate:
        def __init__(self, id_obj):
            self.id = id_obj

    class MockDepsGraph:
        def __init__(self, updates):
            self.updates = updates

    updates = [
        MockUpdate(link_true),
        MockUpdate(link_false),
        MockUpdate(joint_true),
        MockUpdate(joint_false),
        MockUpdate(sensor_true),
        MockUpdate(sensor_false),
        MockUpdate(trans_true),
        MockUpdate(trans_false),
    ]
    depsgraph = MockDepsGraph(updates)

    # Call depsgraph handler
    name_sync_handler.on_depsgraph_update_post(scene, depsgraph)

    # Verify TRUE branches successfully updated properties to match sanitized names
    assert lp_true.link_name == "new_link_true"
    assert jp_true.joint_name == "new_joint_true"
    assert sp_true.sensor_name == "new_sensor_true"
    assert tp_true.transmission_name == "new_trans_true"


def test_register_unregister():
    """Test register and unregister functions of name_sync_handler and package-level handlers."""
    from linkforge.blender import handlers as handlers_pkg

    # Ensure it's not present initially
    if name_sync_handler.on_depsgraph_update_post in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.remove(name_sync_handler.on_depsgraph_update_post)

    # 1. Register via package-level
    handlers_pkg.register()
    assert name_sync_handler.on_depsgraph_update_post in bpy.app.handlers.depsgraph_update_post

    # 2. Register again (should not duplicate)
    initial_len = len(bpy.app.handlers.depsgraph_update_post)
    handlers_pkg.register()
    assert len(bpy.app.handlers.depsgraph_update_post) == initial_len

    # 3. Unregister via package-level
    handlers_pkg.unregister()
    assert name_sync_handler.on_depsgraph_update_post not in bpy.app.handlers.depsgraph_update_post

    # 4. Unregister again (should not crash if not present)
    handlers_pkg.unregister()
    assert name_sync_handler.on_depsgraph_update_post not in bpy.app.handlers.depsgraph_update_post
